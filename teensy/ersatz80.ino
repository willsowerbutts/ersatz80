#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "z80.h"
#include "serial.h"
#include "debug.h"
#include "rom.h"

#define SUPERVISOR_ESCAPE_KEYCODE 7 // Ctrl-G

typedef unsigned char uint8_t;

void z80_show_regs(void);

#define ROM_ADDR_MASK (0x3FFF) // 16KB

#define UART_RX_FIFO_BUFFER_SIZE 32
uint8_t uart_rx_fifo_waiting = 0;
uint8_t uart_rx_fifo_start = 0;
uint8_t uart_rx_fifo_buffer[UART_RX_FIFO_BUFFER_SIZE];

bool uart_rx_fifo_push(uint8_t keyin)
{
    if(uart_rx_fifo_waiting >= UART_RX_FIFO_BUFFER_SIZE)
        return false;

    // if we start using interrupts, consider atomicity
    uart_rx_fifo_buffer[(uart_rx_fifo_start + uart_rx_fifo_waiting) % UART_RX_FIFO_BUFFER_SIZE] = keyin;
    uart_rx_fifo_waiting++;
    return true;
}

uint8_t uart_rx_fifo_pop(void)
{
    uint8_t r;
    if(!uart_rx_fifo_waiting)
        return 0xff;

    // if we start using interrupts, consider atomicity
    r = uart_rx_fifo_buffer[uart_rx_fifo_start];
    uart_rx_fifo_waiting--;
    uart_rx_fifo_start = (uart_rx_fifo_start+1) % UART_RX_FIFO_BUFFER_SIZE;
    return r;
}

#define SBUFLEN 40
#define is_cmd(x) (!strcasecmp_P(buf, PSTR(x)))
void supervisor_menu(void)
{
    char buf[SBUFLEN];
    debug_boldon();
    while(true){
        report("Supervisor> ");
        serial_read_line((unsigned char*)buf, SBUFLEN);
        report("\r\n");
        if(is_cmd("trace bus"))
            z80_bus_trace = true;
        else if(is_cmd("no trace bus"))
            z80_bus_trace = false;
        else if(is_cmd("regs"))
            z80_show_regs();
        // else if(is_cmd("trace mem"))
        //     z80_mem_trace = ... it'd be nice to choose read/write/both ... ?
        // else if(is_cmd("trace io"))
        //     z80_io_trace = ... it'd be nice to choose read/write/both ... ?
        // would be nice to have a trace like:
        // 0421: 1f        inc hl
        // 0422: 83 02 30  ld de,(3002)
        //       3002: 9302
        // 0425: ...
        else if(is_cmd("quit") || is_cmd("exit"))
            break;
        else
            report("???\r\n");
        // trace all bus states
        // trace memory reads
        // trace memory writes
        // display memory
        // edit memory
        // upload code
        // set breakpoints (code, data)
        // reset cpu
        // jump cpu to address
        // report cpu registers
    }
    debug_boldoff();
}

uint8_t iodevice_read(uint16_t address)
{
    //report("IO Read %04x\n", address);
    switch(address & 0xFF){
        case 0x00: // UART status
            return (uart_rx_fifo_waiting ? 0x80 : 0x00) | (Serial.availableForWrite()>0 ? 0x00 : 0x40);
        case 0x01: // UART data
            return uart_rx_fifo_pop();
        case 0x78: // bank0 page select -- Zeta2 compatible
        case 0x79: // bank1 page select -- Zeta2 compatible
        case 0x7A: // bank2 page select -- Zeta2 compatible
        case 0x7B: // bank3 page select -- Zeta2 compatible
            return mmu[(address & 0xFF) - 0x78];
        default:
            report("[IOR %04x]", address);
            return 0xAA;
    }
}

void iodevice_write(uint16_t address, uint8_t value) // call ONLY when in DMA mode!
{
    //report("IO Write %04x %02x\n", address, value);
    switch(address & 0xFF){
        case 0x01: // UART data
            Serial.write(value);
            break;
        case 0x11: // LEDs
            user_led = (user_led & 0xFF00) | value;
            shift_register_update();
            break;
        case 0x12: // LEDs
            user_led = (user_led & 0xFF) | ((value & 0x0F) << 8);
            shift_register_update();
            break;
        case 0x78: // bank0 page select -- Zeta2 compatible
        case 0x79: // bank1 page select -- Zeta2 compatible
        case 0x7A: // bank2 page select -- Zeta2 compatible
        case 0x7B: // bank3 page select -- Zeta2 compatible
            z80_bus_master();
            z80_set_mmu((address & 0xFF) - 0x78, value);
            z80_bus_slave();
            break;
        default:
            report("[IOW %04x %02x]", address, value);
            break;
    }
}

uint8_t memory_read(uint16_t address)
{
    return basic_rom[address & ROM_ADDR_MASK];
}

void memory_write(uint16_t address, uint8_t value)
{
    // nop
}

void z80_tick_tock()
{
    if(z80_mreq_asserted()){
        if(z80_rd_asserted()){
            z80_clock_pulse_drive_data(memory_read(z80_bus_address()));
        }else if(z80_wr_asserted()){
            memory_write(z80_bus_address(), z80_bus_data());
            z80_clock_pulse_while_writing();
        }else
            z80_clock_pulse();
    }else if(z80_iorq_asserted()){
        z80_set_release_wait(true);
        if(z80_rd_asserted()){
            z80_clock_pulse_drive_data(iodevice_read(z80_bus_address()));
        }else if(z80_wr_asserted()){
            iodevice_write(z80_bus_address(), z80_bus_data());
            z80_clock_pulse_while_writing();
        }else
            z80_clock_pulse();
        z80_set_release_wait(false);
    }else{
        z80_clock_pulse();
    }
}

void synthesised_clock(void)
{
    int keyin;

    while(true){
        if(Serial.available() > 0){
            keyin = Serial.read();
            if(keyin == SUPERVISOR_ESCAPE_KEYCODE){
                supervisor_menu();
            }else{
                if(!uart_rx_fifo_push(keyin))
                    report("UART: rxdata buffer overflow\r\n");
            }
        }
        z80_tick_tock();
    }
}

void z80_send_instruction(uint8_t opcode)
{
    while(!(z80_mreq_asserted() && z80_rd_asserted()))
        z80_clock_pulse();
    z80_clock_pulse_drive_data(opcode);
}

uint16_t z80_send_instruction_read_stack(uint8_t opcode)
{
    uint16_t r;

    z80_send_instruction(opcode);
    // now it will write to the stack
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();
    r = z80_bus_data();
    z80_clock_pulse_while_writing();
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();
    r = (r << 8) | z80_bus_data();
    return r;
}

void z80_show_regs(void)
{
    uint16_t pc, sp, af, bc, de, hl, ix, iy, af_, bc_, de_, hl_;
    uint8_t i;

    /* NEEDS TO CHANGE CLOCK HERE */

    // this code does not deal with the situation where CPU is HALTed.
    // solution might be: wake CPU with an int/nmi, capture PC when it writes it to the
    //                    stack. capture regs as usual. then IRET and JP to PC-1, then 
    //                    feed it a HALT when it fetches PC-1. it will HALT with PC correct.

    // if we're partway through an M1 cycle, allow it to complete first
    while(z80_m1_asserted())
        z80_tick_tock();
    // wait for a new M1 cycle to start
    while(!z80_m1_asserted())
        z80_tick_tock();

    // now we feed it a synthesised instruction - F5 (PUSH AF)
    while(!(z80_mreq_asserted() && z80_rd_asserted()))
        z80_clock_pulse();
    pc = z80_bus_address();                      // (this gives us the PC register)
    z80_clock_pulse_drive_data(0xF5);            // PUSH AF

    // now it will write to the stack
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();
    sp = z80_bus_address() + 1;                  // (this gives us the SP register)
    af = z80_bus_data();
    z80_clock_pulse_while_writing();
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();
    af = (af << 8) | z80_bus_data();

    bc = z80_send_instruction_read_stack(0xC5);  // PUSH BC
    de = z80_send_instruction_read_stack(0xD5);  // PUSH DE
    hl = z80_send_instruction_read_stack(0xE5);  // PUSH HL
    z80_send_instruction(0x08);                  // EX AF, AF'
    af_ = z80_send_instruction_read_stack(0xF5); // PUSH AF
    z80_send_instruction(0x08);                  // EX AF, AF' again (swap back)
    z80_send_instruction(0xD9);                  // EXX
    bc_ = z80_send_instruction_read_stack(0xC5); // PUSH BC
    de_ = z80_send_instruction_read_stack(0xD5); // PUSH DE
    hl_ = z80_send_instruction_read_stack(0xE5); // PUSH HL
    z80_send_instruction(0xD9);                  // EXX again (swap back)
    z80_send_instruction(0xDD);                  // IX prefix
    ix  = z80_send_instruction_read_stack(0xE5); // PUSH IX
    z80_send_instruction(0xFD);                  // IY prefix
    iy  = z80_send_instruction_read_stack(0xE5); // PUSH IY
    z80_send_instruction(0xED);                  // ED prefix
    z80_send_instruction(0x57);                  // LD A,I - note this affects the flags register
    i = z80_send_instruction_read_stack(0xF5) >> 8; // PUSH AF - I is now in A (high bits)

    // finally we need to put AF, SP and PC back as they were before our tinkering
    z80_send_instruction(0xF1);                  // POP af
    z80_send_instruction(af & 0xFF);             //  ...
    z80_send_instruction(af >> 8);               //  ...
    z80_send_instruction(0x31);                  // LD SP, xxxx
    z80_send_instruction(sp & 0xFF);             //  ...
    z80_send_instruction(sp >> 8);               //  ...
    z80_send_instruction(0xC3);                  // JP xxxx
    z80_send_instruction(pc & 0xFF);             //  ...
    z80_send_instruction(pc >> 8);               //  ...

    report("PC=%04x SP=%04x\r\nAF=%04x AF'=%04x\r\n" \
           "BC=%04x BC'=%04x\r\nDE=%04x DE'=%04x\r\n" \
           "HL=%04x HL'=%04x\r\nIX=%04x IY=%04x I=%02x\r\n",
           pc, sp, af, af_,
           bc, bc_, de, de_,
           hl, hl_, ix, iy, i);
}

void setup() {
    z80_setup();
    Serial.begin(9600);
    while(!Serial.dtr()); // wait for someone to open the USB device
    report("ersatz80: init\r\n");
    report("ersatz80: reset Z80\r\n");
    z80_do_reset();
    report("Supervisor keycode is Ctrl+%c.\r\n", 'A' - 1 + SUPERVISOR_ESCAPE_KEYCODE);
}

void dma_test(void); // TEST

inline void z80_complete_read(uint8_t data)
{
    z80_setup_drive_data(data);
    z80_set_busrq(true);
    z80_set_release_wait(true);
    while(!z80_busack_asserted());
    z80_shutdown_drive_data();
    z80_set_release_wait(false);
    z80_set_busrq(false);
    // return with Z80 running
}

inline void z80_complete_write(void)
{
    z80_set_busrq(true);
    z80_set_release_wait(true);
    while(!z80_busack_asserted());
    z80_set_release_wait(false);
    // return with DMA capable -- caller must do z80_set_busrq(false);
}

void loop() {
    // put program in RAM
    dma_test();
    z80_do_reset();
    ram_ce = true;
    shift_register_update();
    z80_start_fast_clock();

    while(true){
        if(z80_wait_asserted()){
            if(z80_iorq_asserted()){
                if(z80_rd_asserted()){
                    z80_complete_read(iodevice_read(z80_bus_address()));
                }else if(z80_wr_asserted()){
                    z80_complete_write(); // leaves us in DMA mode
                    iodevice_write(z80_bus_address_low8(), z80_bus_data());
                    z80_set_busrq(false);
                }else
                    report("(iorq weird?)");
            } else if(z80_mreq_asserted()){
                if(z80_rd_asserted()){
                    z80_complete_read(memory_read(z80_bus_address()));
                }else if(z80_wr_asserted()){
                    z80_complete_write(); // leaves us in DMA mode
                    memory_write(z80_bus_address_low8(), z80_bus_data());
                    z80_set_busrq(false);
                }else
                    report("(mreq weird?)");
            } else
                report("(wait weird?)");
        }
        if((uart_rx_fifo_waiting < UART_RX_FIFO_BUFFER_SIZE) && (Serial.available() > 0)){
            int keyin = Serial.read();
            if(keyin == SUPERVISOR_ESCAPE_KEYCODE){
                supervisor_menu();
            }else{
                if(!uart_rx_fifo_push(keyin))
                    report("UART: rxdata buffer overflow\n");
            }
        }
    }
}
