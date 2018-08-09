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

uint8_t z80_rom[] = {
    0xc3, 0x05, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00,  
    0x23, 0x22, 0x03, 0x00, 0x18, 0xfa,
};

#define RAM_ADDR_MASK 0x03FF
uint8_t z80_ram[1024];

void memory_init(void)
{
    int i;

    report("ROM %d bytes RAM %d bytes", sizeof(z80_rom), sizeof(z80_ram));

    // copy in ROM
    for(i=0; i<sizeof(z80_rom); i++)
        z80_ram[i] = z80_rom[i];
    // fill with 0s
    while(i<sizeof(z80_ram))
        z80_ram[i++] = 0;
}

uint8_t memory_read(uint16_t address)
{
    return z80_ram[address & RAM_ADDR_MASK];
}

void memory_write(uint16_t address, uint8_t value)
{
    z80_ram[address & RAM_ADDR_MASK] = value;
}

void z80_demo(void)
{
    uint8_t r;
    z80_bus_state n;

    memory_init();

    z80_set_reset(true);
    for(int i=0; i<100; i++){
        z80_set_clk(false);
        z80_set_clk(true);
    }
    z80_set_reset(false);

    while(true){
        // p = n;
        z80_read_bus(&n);

        /*
        report("data=0x%02x", n.data);

        if(n.data != p.data)
            report("! ");
        else
            report("  ");

        report("address=0x%04x", n.address);
        if(n.address != p.address)
            report("! ");
        else
            report("  ");

        report("%s%s %s%s %s%s %s%s %s%s",
                n.iorq ? "IORQ":"iorq",
                n.iorq == p.iorq ? " ":"!",
                n.mreq ? "MREQ":"mreq",
                n.mreq == p.mreq ? " ":"!",
                n.m1   ? "M1":"m1",
                n.m1 == p.m1 ? " ":"!",
                n.rd   ? "RD":"rd",
                n.rd == p.rd ? " ":"!",
                n.wr   ? "WR":"wr",
                n.wr == p.wr ? " ":"!");
        */

        if(n.mreq && n.rd){
            r = memory_read(n.address);
            report("R%04x=%02x%s\n", n.address, r, n.m1?"<":"");
            z80_clock_pulse_drive_data(r);
        }else{
            if(n.mreq && n.wr){
                report("W%04x:%02x\n", n.address, n.data);
                memory_write(n.address, n.data);
            }
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

