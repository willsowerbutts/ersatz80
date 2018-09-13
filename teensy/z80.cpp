#include <Arduino.h>
#include "rom.h"
#include "debug.h"
#include "z80.h"

uint16_t user_led = 0x000; // value to show on user-controlled LEDs
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
#ifdef KINETISK
    *portModeRegister(Z80_A0  ) = 1;
    *portModeRegister(Z80_A1  ) = 1;
    *portModeRegister(Z80_A2  ) = 1;
    *portModeRegister(Z80_A3  ) = 1;
    *portModeRegister(Z80_A4  ) = 1;
    *portModeRegister(Z80_A5  ) = 1;
    *portModeRegister(Z80_A6  ) = 1;
    *portModeRegister(Z80_A7  ) = 1;
    *portModeRegister(Z80_A8  ) = 1;
    *portModeRegister(Z80_A9  ) = 1;
    *portModeRegister(Z80_A10 ) = 1;
    *portModeRegister(Z80_A11 ) = 1;
    *portModeRegister(Z80_A12 ) = 1;
    *portModeRegister(Z80_A13 ) = 1;
    *portModeRegister(Z80_A14 ) = 1;
    *portModeRegister(Z80_A15 ) = 1;
    *portModeRegister(Z80_D0  ) = 1;
    *portModeRegister(Z80_D1  ) = 1;
    *portModeRegister(Z80_D2  ) = 1;
    *portModeRegister(Z80_D3  ) = 1;
    *portModeRegister(Z80_D4  ) = 1;
    *portModeRegister(Z80_D5  ) = 1;
    *portModeRegister(Z80_D6  ) = 1;
    *portModeRegister(Z80_D7  ) = 1;
    // careful with these
    *portOutputRegister(Z80_RD  ) = 1;
    *portModeRegister(Z80_RD    ) = 1;
    *portOutputRegister(Z80_WR  ) = 1;
    *portModeRegister(Z80_WR    ) = 1;
    *portOutputRegister(Z80_IORQ) = 1;
    *portModeRegister(Z80_IORQ  ) = 1;
    *portOutputRegister(Z80_MREQ) = 1;
    *portModeRegister(Z80_MREQ  ) = 1;
#else
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
#endif
}

void z80_bus_slave(void)
{
#ifdef KINETISK
    *portModeRegister(Z80_A0  ) = 0;
    *portModeRegister(Z80_A1  ) = 0;
    *portModeRegister(Z80_A2  ) = 0;
    *portModeRegister(Z80_A3  ) = 0;
    *portModeRegister(Z80_A4  ) = 0;
    *portModeRegister(Z80_A5  ) = 0;
    *portModeRegister(Z80_A6  ) = 0;
    *portModeRegister(Z80_A7  ) = 0;
    *portModeRegister(Z80_A8  ) = 0;
    *portModeRegister(Z80_A9  ) = 0;
    *portModeRegister(Z80_A10 ) = 0;
    *portModeRegister(Z80_A11 ) = 0;
    *portModeRegister(Z80_A12 ) = 0;
    *portModeRegister(Z80_A13 ) = 0;
    *portModeRegister(Z80_A14 ) = 0;
    *portModeRegister(Z80_A15 ) = 0;
    *portModeRegister(Z80_D0  ) = 0;
    *portModeRegister(Z80_D1  ) = 0;
    *portModeRegister(Z80_D2  ) = 0;
    *portModeRegister(Z80_D3  ) = 0;
    *portModeRegister(Z80_D4  ) = 0;
    *portModeRegister(Z80_D5  ) = 0;
    *portModeRegister(Z80_D6  ) = 0;
    *portModeRegister(Z80_D7  ) = 0;
    *portModeRegister(Z80_RD  ) = 0;
    *portModeRegister(Z80_WR  ) = 0;
    *portModeRegister(Z80_IORQ) = 0;
    *portModeRegister(Z80_MREQ) = 0;
#else
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
#endif
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

#ifdef KINETISK
    // note SRE = slew rate limiting
    // we set the config ONCE ONLY, suitable for both input and output. we need to 
    // avoid using pinMode() on these registers as it will overwrite our config!
    *portConfigRegister(Z80_A0  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A1  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A2  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A3  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A4  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A5  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A6  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A7  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A8  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A9  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A10 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A11 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A12 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A13 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A14 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A15 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D0  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D1  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D2  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D3  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D4  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D5  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D6  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D7  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_RD  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_WR  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_IORQ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_MREQ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
#endif
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
    digitalWrite(CLK_STROBE, 1);
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

uint16_t z80_bus_address(void)
{
#ifdef KINETISK
    return  *portInputRegister(Z80_A0)         |
           (*portInputRegister(Z80_A1)  <<  1) |
           (*portInputRegister(Z80_A2)  <<  2) |
           (*portInputRegister(Z80_A3)  <<  3) |
           (*portInputRegister(Z80_A4)  <<  4) |
           (*portInputRegister(Z80_A5)  <<  5) |
           (*portInputRegister(Z80_A6)  <<  6) |
           (*portInputRegister(Z80_A7)  <<  7) |
           (*portInputRegister(Z80_A8)  <<  8) |
           (*portInputRegister(Z80_A9)  <<  9) |
           (*portInputRegister(Z80_A10) << 10) |
           (*portInputRegister(Z80_A11) << 11) |
           (*portInputRegister(Z80_A12) << 12) |
           (*portInputRegister(Z80_A13) << 13) |
           (*portInputRegister(Z80_A14) << 14) |
           (*portInputRegister(Z80_A15) << 15);
#else
    return  digitalRead(Z80_A0)         |
           (digitalRead(Z80_A1)  <<  1) |
           (digitalRead(Z80_A2)  <<  2) |
           (digitalRead(Z80_A3)  <<  3) |
           (digitalRead(Z80_A4)  <<  4) |
           (digitalRead(Z80_A5)  <<  5) |
           (digitalRead(Z80_A6)  <<  6) |
           (digitalRead(Z80_A7)  <<  7) |
           (digitalRead(Z80_A8)  <<  8) |
           (digitalRead(Z80_A9)  <<  9) |
           (digitalRead(Z80_A10) << 10) |
           (digitalRead(Z80_A11) << 11) |
           (digitalRead(Z80_A12) << 12) |
           (digitalRead(Z80_A13) << 13) |
           (digitalRead(Z80_A14) << 14) |
           (digitalRead(Z80_A15) << 15);
#endif
}

uint8_t z80_bus_address_low8(void)
{
#ifdef KINETISK
    return  *portInputRegister(Z80_A0)       |
           (*portInputRegister(Z80_A1) << 1) |
           (*portInputRegister(Z80_A2) << 2) |
           (*portInputRegister(Z80_A3) << 3) |
           (*portInputRegister(Z80_A4) << 4) |
           (*portInputRegister(Z80_A5) << 5) |
           (*portInputRegister(Z80_A6) << 6) |
           (*portInputRegister(Z80_A7) << 7);
#else
    return  digitalRead(Z80_A0)       |
           (digitalRead(Z80_A1) << 1) |
           (digitalRead(Z80_A2) << 2) |
           (digitalRead(Z80_A3) << 3) |
           (digitalRead(Z80_A4) << 4) |
           (digitalRead(Z80_A5) << 5) |
           (digitalRead(Z80_A6) << 6) |
           (digitalRead(Z80_A7) << 7);
#endif
}

uint8_t z80_bus_data(void)
{
#ifdef KINETISK
    return  *portInputRegister(Z80_D0)       |
           (*portInputRegister(Z80_D1) << 1) |
           (*portInputRegister(Z80_D2) << 2) |
           (*portInputRegister(Z80_D3) << 3) |
           (*portInputRegister(Z80_D4) << 4) |
           (*portInputRegister(Z80_D5) << 5) |
           (*portInputRegister(Z80_D6) << 6) |
           (*portInputRegister(Z80_D7) << 7);
#else
    return  digitalRead(Z80_D0)       | // digitalRead() returns only 0 or 1
           (digitalRead(Z80_D1) << 1) |
           (digitalRead(Z80_D2) << 2) |
           (digitalRead(Z80_D3) << 3) |
           (digitalRead(Z80_D4) << 4) |
           (digitalRead(Z80_D5) << 5) |
           (digitalRead(Z80_D6) << 6) |
           (digitalRead(Z80_D7) << 7);
#endif
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
#ifdef KINETISK
    *portModeRegister(Z80_D0) = 1;
    *portModeRegister(Z80_D1) = 1;
    *portModeRegister(Z80_D2) = 1;
    *portModeRegister(Z80_D3) = 1;
    *portModeRegister(Z80_D4) = 1;
    *portModeRegister(Z80_D5) = 1;
    *portModeRegister(Z80_D6) = 1;
    *portModeRegister(Z80_D7) = 1;
    *portOutputRegister(Z80_D0) = data; // register only looks at lowest bit
    *portOutputRegister(Z80_D1) = data >> 1;
    *portOutputRegister(Z80_D2) = data >> 2;
    *portOutputRegister(Z80_D3) = data >> 3;
    *portOutputRegister(Z80_D4) = data >> 4;
    *portOutputRegister(Z80_D5) = data >> 5;
    *portOutputRegister(Z80_D6) = data >> 6;
    *portOutputRegister(Z80_D7) = data >> 7;
#else
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
#endif
}

void z80_shutdown_drive_data(void)
{
#ifdef KINETISK
    *portModeRegister(Z80_D0) = 0;
    *portModeRegister(Z80_D1) = 0;
    *portModeRegister(Z80_D2) = 0;
    *portModeRegister(Z80_D3) = 0;
    *portModeRegister(Z80_D4) = 0;
    *portModeRegister(Z80_D5) = 0;
    *portModeRegister(Z80_D6) = 0;
    *portModeRegister(Z80_D7) = 0;
#else
    pinMode(Z80_D0,   INPUT);
    pinMode(Z80_D1,   INPUT);
    pinMode(Z80_D2,   INPUT);
    pinMode(Z80_D3,   INPUT);
    pinMode(Z80_D4,   INPUT);
    pinMode(Z80_D5,   INPUT);
    pinMode(Z80_D6,   INPUT);
    pinMode(Z80_D7,   INPUT);
#endif
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

void z80_setup_address_data(uint16_t address, uint8_t data)
{
#ifdef KINETISK
    *portOutputRegister(Z80_A0 ) = address;
    *portOutputRegister(Z80_A1 ) = address >>  1;
    *portOutputRegister(Z80_A2 ) = address >>  2;
    *portOutputRegister(Z80_A3 ) = address >>  3;
    *portOutputRegister(Z80_A4 ) = address >>  4;
    *portOutputRegister(Z80_A5 ) = address >>  5;
    *portOutputRegister(Z80_A6 ) = address >>  6;
    *portOutputRegister(Z80_A7 ) = address >>  7;
    *portOutputRegister(Z80_A8 ) = address >>  8;
    *portOutputRegister(Z80_A9 ) = address >>  9;
    *portOutputRegister(Z80_A10) = address >> 10;
    *portOutputRegister(Z80_A11) = address >> 11;
    *portOutputRegister(Z80_A12) = address >> 12;
    *portOutputRegister(Z80_A13) = address >> 13;
    *portOutputRegister(Z80_A14) = address >> 14;
    *portOutputRegister(Z80_A15) = address >> 15;
    *portOutputRegister(Z80_D0 ) = data;
    *portOutputRegister(Z80_D1 ) = data >> 1;
    *portOutputRegister(Z80_D2 ) = data >> 2;
    *portOutputRegister(Z80_D3 ) = data >> 3;
    *portOutputRegister(Z80_D4 ) = data >> 4;
    *portOutputRegister(Z80_D5 ) = data >> 5;
    *portOutputRegister(Z80_D6 ) = data >> 6;
    *portOutputRegister(Z80_D7 ) = data >> 7;
#else
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
#endif
}

void z80_mmu_write(uint16_t address, uint8_t data)
{
	z80_setup_address_data(address, data);
    digitalWrite(MMU_EW, 0);
	delayMicroseconds(1);
    digitalWrite(MMU_EW, 1);
}

void z80_memory_write(uint16_t address, uint8_t data)
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
