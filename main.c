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

uint8_t z80_rom[] = { /* output values 32--127 to port 0x10 then loop and repeat */
    0x3e, 0x20, 0xd3, 0x10, 0x3c, 0xfe, 0x7f, 0x20, 0xf9, 0x18, 0xf5,
};

#define RAM_ADDR_MASK 0x03FF
uint8_t z80_ram[1024];

void memory_init(void)
{
    int i;

    report("ROM %d bytes RAM %d bytes\n", sizeof(z80_rom), sizeof(z80_ram));

    // copy in ROM
    for(i=0; i<sizeof(z80_rom); i++)
        z80_ram[i] = z80_rom[i];
    // fill with 0s
    while(i<sizeof(z80_ram))
        z80_ram[i++] = 0;
}

uint8_t iodevice_read(uint16_t address)
{
    return 0;
}

void iodevice_write(uint16_t address, uint8_t value)
{
    switch(address & 0xFF){
        case 0x10:
            putchar(value);
            break;
        default:
            break;
    }
}

uint8_t memory_read(uint16_t address)
{
    return z80_ram[address & RAM_ADDR_MASK];
}

void memory_write(uint16_t address, uint8_t value)
{
    z80_ram[address & RAM_ADDR_MASK] = value;
}

// starting point - 11,952Hz
// optimized -     234,000Hz
// inline -        372,825Hz
// flto -          454,119Hz

void z80_demo(void)
{
    // uint8_t r;
    // z80_bus_state n;

    memory_init();

    /* setup 16-bit timer1 */
    // setup waveform generation mode 4 (CTC mode), prescaler CLK/256
    // TCCR1A = 0;
    // TCCR1B = _BV(WGM12) | _BV(CS12);
    // TCCR1C = 0;
    // OCR1A = ((F_CPU / 256)/1) - 1;     // set TOP value for 1Hz
    // TIMSK1 = 0;
    // TIFR1 = 0xFF; // reset all flags

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
        // once per second
        // if(TIFR1 & _BV(OCF1A)){
        //     TIFR1 = _BV(OCF1A);
        //     report("%luHz\n", z80_clk_counter);
        //     z80_clk_counter = 0;
        // }
        if(z80_mreq_asserted()){
            if(z80_rd_asserted()){
                z80_clock_pulse_drive_data(memory_read(z80_bus_address()));
            }else{
                if(z80_wr_asserted())
                    memory_write(z80_bus_address(), z80_bus_data());
                z80_clock_pulse_while_writing();
            }
        }else if(z80_iorq_asserted()){
            if(z80_rd_asserted()){
                z80_clock_pulse_drive_data(iodevice_read(z80_bus_address()));
            }else{
                if(z80_wr_asserted())
                    iodevice_write(z80_bus_address(), z80_bus_data());
                z80_clock_pulse_while_writing();
            }
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

