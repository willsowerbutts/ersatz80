#include <stdbool.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include "debug.h"
#include "serial.h"
#include "version.h"
#include "z80.h"

#define SUPERVISOR_ESCAPE_KEYCODE 7 // Ctrl-G

void z80_show_regs(void);
extern const uint8_t z80_rom[] PROGMEM;
uint8_t z80_ram[4096];
#define RAM_ADDR_MASK (sizeof(z80_ram)-1)
#define ROM_ADDR_MASK (0x7FFF) // up to 32KB

#define uart_rx_fifo_BUFFER_SIZE 32
uint8_t uart_rx_fifo_waiting = 0;
uint8_t uart_rx_fifo_start = 0;
uint8_t uart_rx_fifo_buffer[uart_rx_fifo_BUFFER_SIZE];

bool uart_rx_fifo_push(uint8_t keyin)
{
    if(uart_rx_fifo_waiting >= uart_rx_fifo_BUFFER_SIZE)
        return false;

    // if we start using interrupts, consider atomicity
    uart_rx_fifo_buffer[(uart_rx_fifo_start + uart_rx_fifo_waiting) % uart_rx_fifo_BUFFER_SIZE] = keyin;
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
    uart_rx_fifo_start = (uart_rx_fifo_start+1) % uart_rx_fifo_BUFFER_SIZE;
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
        report("\n");
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
            report("???\n");
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

void memory_init(void)
{
    int i;

    // place a jump to ROM (0x8000) at the reset vecor
    i = 0;
    z80_ram[i++] = 0xC3;
    z80_ram[i++] = 0x00;
    z80_ram[i++] = 0x80;

    // fill remainder with 0
    while(i<sizeof(z80_ram))
        z80_ram[i++] = 0;
}

uint8_t iodevice_read(uint16_t address)
{
    //report("IO Read %04x\n", address);
    switch(address & 0xFF){
        case 0x00: // UART status
            return (uart_rx_fifo_waiting ? 0x80 : 0x00) | (serial_write_ready() ? 0x00 : 0x40);
        case 0x01: // UART data
            return uart_rx_fifo_pop();
        default:
            return 0xAA;
    }
}

void iodevice_write(uint16_t address, uint8_t value)
{
    //report("IO Write %04x %02x\n", address, value);
    switch(address & 0xFF){
        case 0x01: // UART data
            putchar(value);
            break;
        default:
            break;
    }
}

#if 1
uint8_t memory_read(uint16_t address)
{
    if(address & 0x8000){
        // ROM
        return pgm_read_byte(&z80_rom[address & ROM_ADDR_MASK]);
    }else{
        // RAM
        return z80_ram[address & RAM_ADDR_MASK];
    }
}
#else
uint8_t memory_read(uint16_t address)
{
    uint8_t r;
    if(address & 0x8000){
        r = pgm_read_byte(&z80_rom[address & ROM_ADDR_MASK]);
    }else{
        r = z80_ram[address & RAM_ADDR_MASK];
    }
    report("R%04x %02x\n", address, r);
    return r;
}
#endif

void memory_write(uint16_t address, uint8_t value)
{
    //report("W%04x %02x\n", address, value);
    z80_ram[address & RAM_ADDR_MASK] = value;
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
        if(z80_rd_asserted()){
            z80_clock_pulse_drive_data(iodevice_read(z80_bus_address()));
        }else if(z80_wr_asserted()){
            iodevice_write(z80_bus_address(), z80_bus_data());
            z80_clock_pulse_while_writing();
        }else
            z80_clock_pulse();
    }else{
        z80_clock_pulse();
    }
}

int main(void)
{
    int keyin;

    // initialise serial
    serial_init();
    debug_init();
    report("\nersatz80: init\n");
    report("Supervisor keycode is Ctrl+%c.\n", 'A' - 1 + SUPERVISOR_ESCAPE_KEYCODE);

    z80_init();
    memory_init();
    z80_reset();

    while(true){
        if(serial_read_ready()){
            keyin = serial_read_byte();
            if(keyin == SUPERVISOR_ESCAPE_KEYCODE){
                supervisor_menu();
            }else{
                if(!uart_rx_fifo_push(keyin))
                    report("UART: rxdata buffer overflow\n");
            }
        }
        z80_tick_tock();
    }

    return 0;
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

    report("PC=%04x SP=%04x\nAF=%04x AF'=%04x\n" \
           "BC=%04x BC'=%04x\nDE=%04x DE'=%04x\n" \
           "HL=%04x HL'=%04x\nIX=%04x IY=%04x I=%02x\n",
           pc, sp, af, af_,
           bc, bc_, de, de_,
           hl, hl_, ix, iy, i);
}

