#include <Arduino.h>
#include "rom.h"
#include "debug.h"
#include "z80.h"

#if defined(KINETISK) // speed up Teensy3.5 a bit
#define GPIO_BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define GPIO_BITBAND_PTR(reg, bit) ((uint32_t *)GPIO_BITBAND_ADDR((reg), (bit)))
#endif

const int SHIFT_REGISTER_CLK    = 13;
const int SHIFT_REGISTER_LATCH  = 12;
const int SHIFT_REGISTER_DATA   = 11;
const int Z80_IORQ              = 0;
const int Z80_MREQ              = 1;
const int Z80_RD                = 2;
const int Z80_WR                = 3;
const int Z80_M1                = 4;
const int Z80_WAIT              = 5;
const int Z80_HALT              = 10;
const int WAIT_RESET            = 6;
const int CLK_STROBE            = 7;
const int CLK_FAST_ENABLE       = 8;
const int Z80_BUSACK            = 9;
const int Z80_BUSRQ             = 14;
const int MMU_EW                = 15;
const int Z80_A0                = 16;
const int Z80_A1                = 17;
const int Z80_A2                = 18;
const int Z80_A3                = 19;
const int Z80_A4                = 20;
const int Z80_A5                = 21;
const int Z80_A6                = 22;
const int Z80_A7                = 23;
const int Z80_A8                = 24;
const int Z80_A9                = 25;
const int Z80_A10               = 26;
const int Z80_A11               = 27;
const int Z80_A12               = 28;
const int Z80_A13               = 29;
const int Z80_A14               = 30;
const int Z80_A15               = 31;
const int Z80_D0                = 32;
const int Z80_D1                = 33;
const int Z80_D2                = 34;
const int Z80_D3                = 35;
const int Z80_D4                = 36;
const int Z80_D5                = 37;
const int Z80_D6                = 38;
const int Z80_D7                = 39;

unsigned short user_led = 0x000; // value to show on user-controlled LEDs
bool z80_reset = true;       // Z80 /RESET pin (z80_reset=true means /RESET is driven low ie asserted)
bool z80_irq = false;        // Z80 /IRQ pin
bool z80_nmi = false;        // Z80 /NMI pin
bool ram_ce = false;         // RAM /CE pin
bool z80_bus_trace = false;
uint8_t mmu[4];

void shift_register_update(void)
{
    int output = (user_led & 0xFFF) |       // the LEDs are all active high
                 (z80_reset ? 0 : 0x8000) | // these signals are all active low
                 (z80_nmi   ? 0 : 0x4000) |
                 (z80_irq   ? 0 : 0x2000) |
                 (ram_ce    ? 0 : 0x1000);

    for(int i=0; i<16; i++){
        digitalWrite(SHIFT_REGISTER_DATA, output & 1);
        digitalWrite(SHIFT_REGISTER_CLK, 1);
        output >>= 1;
        digitalWrite(SHIFT_REGISTER_CLK, 0);
    }
    digitalWrite(SHIFT_REGISTER_LATCH, 1);
    digitalWrite(SHIFT_REGISTER_LATCH, 0);
}

void z80_bus_master(void)
{
    // Z80
    pinMode(Z80_A0,    OUTPUT);
    pinMode(Z80_A1,    OUTPUT);
    pinMode(Z80_A2,    OUTPUT);
    pinMode(Z80_A3,    OUTPUT);
    pinMode(Z80_A4,    OUTPUT);
    pinMode(Z80_A5,    OUTPUT);
    pinMode(Z80_A6,    OUTPUT);
    pinMode(Z80_A7,    OUTPUT);
    pinMode(Z80_A8,    OUTPUT);
    pinMode(Z80_A9,    OUTPUT);
    pinMode(Z80_A10,   OUTPUT);
    pinMode(Z80_A11,   OUTPUT);
    pinMode(Z80_A12,   OUTPUT);
    pinMode(Z80_A13,   OUTPUT);
    pinMode(Z80_A14,   OUTPUT);
    pinMode(Z80_A15,   OUTPUT);
    pinMode(Z80_D0,    OUTPUT);
    pinMode(Z80_D1,    OUTPUT);
    pinMode(Z80_D2,    OUTPUT);
    pinMode(Z80_D3,    OUTPUT);
    pinMode(Z80_D4,    OUTPUT);
    pinMode(Z80_D5,    OUTPUT);
    pinMode(Z80_D6,    OUTPUT);
    pinMode(Z80_D7,    OUTPUT);
    digitalWrite(Z80_IORQ, 1);
    digitalWrite(Z80_MREQ, 1);
    digitalWrite(Z80_RD, 1);
    digitalWrite(Z80_WR, 1);
    pinMode(Z80_IORQ,  OUTPUT);
    pinMode(Z80_MREQ,  OUTPUT);
    pinMode(Z80_RD,    OUTPUT);
    pinMode(Z80_WR,    OUTPUT);
}

void z80_bus_slave(void)
{
    // Z80
    pinMode(Z80_A0,    INPUT);
    pinMode(Z80_A1,    INPUT);
    pinMode(Z80_A2,    INPUT);
    pinMode(Z80_A3,    INPUT);
    pinMode(Z80_A4,    INPUT);
    pinMode(Z80_A5,    INPUT);
    pinMode(Z80_A6,    INPUT);
    pinMode(Z80_A7,    INPUT);
    pinMode(Z80_A8,    INPUT);
    pinMode(Z80_A9,    INPUT);
    pinMode(Z80_A10,   INPUT);
    pinMode(Z80_A11,   INPUT);
    pinMode(Z80_A12,   INPUT);
    pinMode(Z80_A13,   INPUT);
    pinMode(Z80_A14,   INPUT);
    pinMode(Z80_A15,   INPUT);
    pinMode(Z80_D0,    INPUT);
    pinMode(Z80_D1,    INPUT);
    pinMode(Z80_D2,    INPUT);
    pinMode(Z80_D3,    INPUT);
    pinMode(Z80_D4,    INPUT);
    pinMode(Z80_D5,    INPUT);
    pinMode(Z80_D6,    INPUT);
    pinMode(Z80_D7,    INPUT);
    pinMode(Z80_IORQ,  INPUT);
    pinMode(Z80_MREQ,  INPUT);
    pinMode(Z80_RD,    INPUT);
    pinMode(Z80_WR,    INPUT);
}

void z80_setup(void)
{
    // shift register setup
    pinMode(SHIFT_REGISTER_CLK, OUTPUT);
    pinMode(SHIFT_REGISTER_LATCH, OUTPUT);
    pinMode(SHIFT_REGISTER_DATA, OUTPUT);

    digitalWrite(SHIFT_REGISTER_CLK, 0);
    digitalWrite(SHIFT_REGISTER_LATCH, 0);
    digitalWrite(SHIFT_REGISTER_DATA, 0);
    shift_register_update();

    z80_bus_slave();
    pinMode(Z80_M1, INPUT);
    pinMode(Z80_WAIT, INPUT);
    pinMode(Z80_HALT, INPUT);
    pinMode(Z80_BUSACK, INPUT);
    pinMode(Z80_BUSRQ, OUTPUT);

    // system
    pinMode(MMU_EW, OUTPUT);
    pinMode(WAIT_RESET, OUTPUT);
    pinMode(CLK_STROBE, OUTPUT);
    pinMode(CLK_FAST_ENABLE, OUTPUT);
    
    digitalWrite(Z80_BUSRQ, 1);
    digitalWrite(MMU_EW, 1);
    digitalWrite(WAIT_RESET, 1);
    digitalWrite(CLK_STROBE, 0);
    digitalWrite(CLK_FAST_ENABLE, 0);

    z80_set_clk(true);
    z80_set_reset(true);
}

void z80_start_fast_clock(void)
{
    digitalWrite(CLK_STROBE, 1); // we should be idling with CLK_STROBE=1 ... ?
    digitalWrite(CLK_FAST_ENABLE, 1);
}

void z80_bus_report_state(void)
{
    report("\r\n|%04x|%02x|%s|%s|%s|",
            z80_bus_address(), z80_bus_data(), 
            z80_mreq_asserted() ? "MREQ" : (z80_iorq_asserted() ? "IORQ" : "    "),
            z80_rd_asserted() ? "RD" : (z80_wr_asserted() ? "WR" : "  "),
            z80_m1_asserted() ? "M1" : "  ");
}

bool z80_wr_asserted(void)
{
    return !digitalRead(Z80_WR);
}

bool z80_busack_asserted(void)
{
    return !digitalRead(Z80_BUSACK);
}

bool z80_rd_asserted(void)
{
    return !digitalRead(Z80_RD);
}

bool z80_iorq_asserted(void)
{
    return !digitalRead(Z80_IORQ);
}

bool z80_wait_asserted(void)
{
    return !digitalRead(Z80_WAIT);
}

bool z80_m1_asserted(void)
{
    return !digitalRead(Z80_M1);
}

bool z80_mreq_asserted(void)
{
    return !digitalRead(Z80_MREQ);
}

uint16_t z80_bus_address(void)
{
    return (digitalRead(Z80_A0)  ? (1<<0)   : 0) |
           (digitalRead(Z80_A1)  ? (1<<1)   : 0) |
           (digitalRead(Z80_A2)  ? (1<<2)   : 0) |
           (digitalRead(Z80_A3)  ? (1<<3)   : 0) |
           (digitalRead(Z80_A4)  ? (1<<4)   : 0) |
           (digitalRead(Z80_A5)  ? (1<<5)   : 0) |
           (digitalRead(Z80_A6)  ? (1<<6)   : 0) |
           (digitalRead(Z80_A7)  ? (1<<7)   : 0) |
           (digitalRead(Z80_A8)  ? (1<<8)   : 0) |
           (digitalRead(Z80_A9)  ? (1<<9)   : 0) |
           (digitalRead(Z80_A10) ? (1<<10)  : 0) |
           (digitalRead(Z80_A11) ? (1<<11)  : 0) |
           (digitalRead(Z80_A12) ? (1<<12)  : 0) |
           (digitalRead(Z80_A13) ? (1<<13)  : 0) |
           (digitalRead(Z80_A14) ? (1<<14)  : 0) |
           (digitalRead(Z80_A15) ? (1<<15)  : 0);
}

uint8_t z80_bus_address_low8(void)
{
    return (digitalRead(Z80_A0)  ? (1<<0)   : 0) |
           (digitalRead(Z80_A1)  ? (1<<1)   : 0) |
           (digitalRead(Z80_A2)  ? (1<<2)   : 0) |
           (digitalRead(Z80_A3)  ? (1<<3)   : 0) |
           (digitalRead(Z80_A4)  ? (1<<4)   : 0) |
           (digitalRead(Z80_A5)  ? (1<<5)   : 0) |
           (digitalRead(Z80_A6)  ? (1<<6)   : 0) |
           (digitalRead(Z80_A7)  ? (1<<7)   : 0);
}

uint8_t z80_bus_data(void)
{
    return  digitalRead(Z80_D0)       | // digitalRead() returns only 0 or 1
           (digitalRead(Z80_D1) << 1) |
           (digitalRead(Z80_D2) << 2) |
           (digitalRead(Z80_D3) << 3) |
           (digitalRead(Z80_D4) << 4) |
           (digitalRead(Z80_D5) << 5) |
           (digitalRead(Z80_D6) << 6) |
           (digitalRead(Z80_D7) << 7);
}

void z80_set_busrq(bool request_dma)
{
    digitalWrite(Z80_BUSRQ, !request_dma);
}

void z80_set_release_wait(bool release)
{
    digitalWrite(WAIT_RESET, !release);
}

void z80_set_reset(bool active)
{
    z80_reset = active;
    shift_register_update();
}

void z80_set_clk(bool level)
{
    if(level){
        digitalWrite(CLK_STROBE, 0); // CLK high
        if(z80_bus_trace)
            z80_bus_report_state();
    }else
        digitalWrite(CLK_STROBE, 1); // CLK low
}

void z80_clock_pulse(void)
{
    z80_set_clk(false);
    z80_set_clk(true);
}

void z80_setup_drive_data(uint8_t data)
{
    pinMode(Z80_D0, OUTPUT);
    pinMode(Z80_D1, OUTPUT);
    pinMode(Z80_D2, OUTPUT);
    pinMode(Z80_D3, OUTPUT);
    pinMode(Z80_D4, OUTPUT);
    pinMode(Z80_D5, OUTPUT);
    pinMode(Z80_D6, OUTPUT);
    pinMode(Z80_D7, OUTPUT);
    digitalWrite(Z80_D0, data & (1<<0) ? 1 : 0);
    digitalWrite(Z80_D1, data & (1<<1) ? 1 : 0);
    digitalWrite(Z80_D2, data & (1<<2) ? 1 : 0);
    digitalWrite(Z80_D3, data & (1<<3) ? 1 : 0);
    digitalWrite(Z80_D4, data & (1<<4) ? 1 : 0);
    digitalWrite(Z80_D5, data & (1<<5) ? 1 : 0);
    digitalWrite(Z80_D6, data & (1<<6) ? 1 : 0);
    digitalWrite(Z80_D7, data & (1<<7) ? 1 : 0);
}

void z80_shutdown_drive_data(void)
{
    pinMode(Z80_D0,   INPUT);
    pinMode(Z80_D1,   INPUT);
    pinMode(Z80_D2,   INPUT);
    pinMode(Z80_D3,   INPUT);
    pinMode(Z80_D4,   INPUT);
    pinMode(Z80_D5,   INPUT);
    pinMode(Z80_D6,   INPUT);
    pinMode(Z80_D7,   INPUT);
}

void z80_clock_pulse_drive_data(uint8_t data)
{
    z80_setup_drive_data(data);

    do{
        z80_clock_pulse();
    }while( z80_rd_asserted() );

    z80_shutdown_drive_data();
}

void z80_clock_pulse_while_writing(void)
{
    do{
        z80_clock_pulse();
    }while( z80_wr_asserted() );
}

void z80_do_reset(void)
{
    // report("Z80 CPU Reset\r\n");
    z80_set_reset(true);
    z80_set_release_wait(true);
    for(int i=0; i<10; i++){ // Z80 requires at least 3 clocks to fully reset
        z80_set_clk(false);
        z80_set_clk(true);
    }
    z80_set_release_wait(false);
    z80_set_reset(false);
}

void z80_setup_address_data(unsigned short address, unsigned char data)
{
    digitalWrite(Z80_A0,  address & (1<<0)  ? 1 : 0);
    digitalWrite(Z80_A1,  address & (1<<1)  ? 1 : 0);
    digitalWrite(Z80_A2,  address & (1<<2)  ? 1 : 0);
    digitalWrite(Z80_A3,  address & (1<<3)  ? 1 : 0);
    digitalWrite(Z80_A4,  address & (1<<4)  ? 1 : 0);
    digitalWrite(Z80_A5,  address & (1<<5)  ? 1 : 0);
    digitalWrite(Z80_A6,  address & (1<<6)  ? 1 : 0);
    digitalWrite(Z80_A7,  address & (1<<7)  ? 1 : 0);
    digitalWrite(Z80_A8,  address & (1<<8)  ? 1 : 0);
    digitalWrite(Z80_A9,  address & (1<<9)  ? 1 : 0);
    digitalWrite(Z80_A10, address & (1<<10) ? 1 : 0);
    digitalWrite(Z80_A11, address & (1<<11) ? 1 : 0);
    digitalWrite(Z80_A12, address & (1<<12) ? 1 : 0);
    digitalWrite(Z80_A13, address & (1<<13) ? 1 : 0);
    digitalWrite(Z80_A14, address & (1<<14) ? 1 : 0);
    digitalWrite(Z80_A15, address & (1<<15) ? 1 : 0);

    digitalWrite(Z80_D0, data & (1<<0) ? 1 : 0);
    digitalWrite(Z80_D1, data & (1<<1) ? 1 : 0);
    digitalWrite(Z80_D2, data & (1<<2) ? 1 : 0);
    digitalWrite(Z80_D3, data & (1<<3) ? 1 : 0);
    digitalWrite(Z80_D4, data & (1<<4) ? 1 : 0);
    digitalWrite(Z80_D5, data & (1<<5) ? 1 : 0);
    digitalWrite(Z80_D6, data & (1<<6) ? 1 : 0);
    digitalWrite(Z80_D7, data & (1<<7) ? 1 : 0);
}

void z80_mmu_write(unsigned short address, unsigned char data)
{
	z80_setup_address_data(address, data);
    digitalWrite(MMU_EW, 0);
	delayMicroseconds(1);
    digitalWrite(MMU_EW, 1);
}

void z80_memory_write(unsigned short address, unsigned char data)
{
	z80_setup_address_data(address, data);
    digitalWrite(Z80_MREQ, 0);
    digitalWrite(Z80_WR, 0);
	delayMicroseconds(1);
    digitalWrite(Z80_WR, 1);
    digitalWrite(Z80_MREQ, 1);
}

void z80_set_mmu(int bank, uint8_t page) // call only in DMA mode
{
    if(bank < 0 || bank > 3){
        report("z80_set_mmu: bad bank %d!\r\n", bank);
        return;
    }
    if(mmu[bank] == page)
        return;
    mmu[bank] = page;
    z80_mmu_write(bank << 14, page);
}

void dma_test(void)
{
    // put our program into RAM
    digitalWrite(Z80_BUSRQ, 0);     // assert BUSRQ
    while(digitalRead(Z80_BUSACK))  // wait for BUSACK
        z80_clock_pulse();          // need to check /WAIT etc in here!
    // now we're in charge!
    ram_ce = true;
    shift_register_update();
    z80_bus_master();

	// setup the MMU
    for(int i=0; i<4; i++){
        mmu[i] = 0xaa;
        z80_set_mmu(i, i);
    }

    z80_memory_write(0, 0xc3);
    z80_memory_write(1, 0x00);
    z80_memory_write(2, 0x80);
    
	for(int i=0; i<MONITOR_ROM_SIZE; i++)
		z80_memory_write(0x8000 + i, monitor_rom[i]);

    // shut it down!
    z80_bus_slave();
    ram_ce = false;
    shift_register_update();
    digitalWrite(Z80_BUSRQ, 1);     // release BUSRQ to end DMA
}
