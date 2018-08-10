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

void reboot(void)
{
    wdt_enable(WDTO_15MS);
    cli();
    while(1); // wait for watchdog to reset us
}

// uint8_t z80_rom[] = {
//     0xc3, 0x05, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00,  
//     0x23, 0x22, 0x03, 0x00, 0x18, 0xfa,
// };

extern const uint8_t z80_rom[] PROGMEM;
// extern const uint8_t z80_rom[] PROGMEM = { /* output values 32--127 to port 0x10 then loop and repeat */
//    0x3e, 0x20, 0xd3, 0x10, 0x3c, 0xfe, 0x7f, 0x20, 0xf9, 0x18, 0xf5,
// };

uint8_t z80_ram[4096];
#define RAM_ADDR_MASK (sizeof(z80_ram)-1)
#define ROM_ADDR_MASK (0x7FFF) // up to 32KB

void memory_init(void)
{
    int i;

    // jump to ROM at 0x8000
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
            return (serial_read_ready() ? 0x80 : 0x00) | (serial_write_ready() ? 0x00 : 0x40);
        case 0x01: // UART data
            return serial_read_byte();
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

void z80_demo(void)
{
    memory_init();

    /* how many clocks do we need here? */
    report("Z80: reset\n");
    z80_set_reset(true);
    for(int i=0; i<100; i++){
        z80_set_clk(false);
        z80_set_clk(true);
    }
    z80_set_reset(false);
    report("Z80: run\n");

    while(true){
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
}

int main(void)
{
    // initialise serial
    serial_init();
    debug_init();
    report("\nersatz80: init\n");

    z80_init();

    z80_demo();

    while(1){
        report("more stuff ...\n");
    }

    return 0;
}

