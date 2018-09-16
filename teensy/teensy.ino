#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "z80.h"
#include "serial.h"
#include "debug.h"
#include "super.h"
#include "rom.h"

#define UART_RX_FIFO_BUFFER_SIZE 128
uint8_t uart_rx_fifo_waiting = 0;
uint8_t uart_rx_fifo_start = 0;
uint8_t uart_rx_fifo_buffer[UART_RX_FIFO_BUFFER_SIZE];
bool supervisor_input_mode = false;

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
            // we might be called in different contexts -- we might already be bus master? or Z80 still running?
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
                // supervisor_menu();
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

    z80_clk_switch_stop();                       // stop the fast clock

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

    // disable the RAM so we can control the data bus
    ram_ce = false;
    shift_register_update();

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

    ram_ce = true;                               // turn back on the RAM
    shift_register_update();
    z80_clk_switch_fast();                       // resume warp speed

    report("PC=%04x SP=%04x\r\nAF=%04x AF'=%04x\r\n" \
           "BC=%04x BC'=%04x\r\nDE=%04x DE'=%04x\r\n" \
           "HL=%04x HL'=%04x\r\nIX=%04x IY=%04x I=%02x\r\n",
           pc, sp, af, af_,
           bc, bc_, de, de_,
           hl, hl_, ix, iy, i);
}

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

void handle_z80_bus(void)
{
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
}

void handle_serial_input(void)
{
    int key;

    if((key = Serial.read()) >= 0){
        if(supervisor_input_mode){
            if(!supervisor_menu_key_in(key)){
                supervisor_input_mode = false;
                supervisor_menu_exit();
            }
        }else{
            if(key == SUPERVISOR_ESCAPE_KEYCODE){
                supervisor_input_mode = true;
                supervisor_menu_enter();
            }else if(!uart_rx_fifo_push(key)){
                Serial.write(0x07); // sound bell on overflow
                return; // ... and come back to this task later
            }
        }
    }
}

void setup() {
    z80_setup();
    Serial.begin(9600);
    while(!Serial.dtr()); // wait for a terminal to connect to the USB serial device
    report("ersatz80: init (%.1fMHz ARM, %.1fMHz bus)\r\n", F_CPU/1000000.0, F_BUS/1000000.0);
    z80_do_reset();
    mmu_setup();
    sram_setup();
    report("ersatz80: load ROM\r\n");
    load_program_to_sram(monitor_rom, MONITOR_ROM_START, MONITOR_ROM_SIZE, MONITOR_ROM_START);
    report("ersatz80: reset Z80\r\n");
    z80_do_reset();
    report("Supervisor keycode is Ctrl+%c.\r\n", 'A' - 1 + SUPERVISOR_ESCAPE_KEYCODE);
    // start up the Z80
    ram_ce = true;
    shift_register_update();
    z80_clk_switch_fast();
}

void loop() {
    while(true){
        if(z80_clk_running())
            handle_z80_bus();
        handle_serial_input();
    }
}
