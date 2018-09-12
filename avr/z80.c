#include <assert.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "debug.h"
#include "z80.h"

// bus control signal pins
// inputs:
//   /IORQ  PB1
//   /M1    PB2
//   /MREQ  PB3
//   /RD    PG1
//   /WR    PD7
// outputs:
//   /RESET PG2
//   CLK    PG0

//uint32_t z80_clk_counter = 0;
bool z80_bus_trace;

void z80_init(void)
{
    // data/address and pins
    DDR_DATA  = 0x00;
    DDR_ADDRL = 0x00;
    DDR_ADDRH = 0x00;

    // pull-ups all off
    PORT_DATA  = 0x00;
    PORT_ADDRL = 0x00;
    PORT_ADDRH = 0x00;

    // control signal - inputs (no pull-ups)
    DDRB &= ~(_BV(1) | _BV(2) | _BV(3));
    DDRD &= ~(_BV(7));
    DDRG &= ~(_BV(1));
    PORTB &= ~(_BV(1) | _BV(2) | _BV(3));
    PORTD &= ~(_BV(7));
    PORTG &= ~(_BV(1));

    // control signal - outputs
    DDRG |= (_BV(0) | _BV(2));
    z80_set_clk(true);
    z80_set_reset(true);
}

void z80_bus_report_state(void)
{
    report("\n|%04x|%02x|%s|%s|%s|",
            z80_bus_address(), z80_bus_data(), 
            z80_mreq_asserted() ? "MREQ" : (z80_iorq_asserted() ? "IORQ" : "    "),
            z80_rd_asserted() ? "RD" : (z80_wr_asserted() ? "WR" : "  "),
            z80_m1_asserted() ? "M1" : "  ");
}

void z80_read_bus(z80_bus_state *state)
{
    uint8_t b, d, g;

    state->data = PIN_DATA;
    state->address = (PIN_ADDRH << 8) | PIN_ADDRL;

    b = PINB;
    d = PIND;
    g = PING;

    state->iorq  = !(b & _BV(1));
    state->mreq  = !(b & _BV(3));
    state->m1    = !(b & _BV(2));
    state->wr    = !(d & _BV(7));
    state->clk   =!!(g & _BV(0));
    state->rd    = !(g & _BV(1)); // note this is also tested in z80_clock_pulse_drive_data()
    state->reset = !(g & _BV(2));
}

inline bool z80_wr_asserted(void)       { return !(PIND & _BV(7)); } 
inline bool z80_rd_asserted(void)       { return !(PING & _BV(1)); } 
inline bool z80_iorq_asserted(void)     { return !(PINB & _BV(1)); } 
inline bool z80_m1_asserted(void)       { return !(PINB & _BV(2)); } 
inline bool z80_mreq_asserted(void)     { return !(PINB & _BV(3)); } 
inline uint16_t z80_bus_address(void)   { return  (PIN_ADDRH << 8) | PIN_ADDRL; } 
inline uint8_t z80_bus_data(void)       { return   PIN_DATA; } 

inline void z80_set_reset(bool active)
{
    if(active)
        PORTG &= ~(_BV(2)); // /RESET low (asserted)
    else
        PORTG |= (_BV(2));  // /RESET high
}

inline void z80_set_clk(bool level)
{
    if(level){
        PORTG |= (_BV(0));  // CLK high
        if(z80_bus_trace)
            z80_bus_report_state();
    }else
        PORTG &= ~(_BV(0)); // CLK low
}

inline void z80_clock_pulse(void)
{
    z80_set_clk(false);
    z80_set_clk(true);
}

inline void z80_clock_pulse_drive_data(uint8_t data)
{
    DDR_DATA   = 0xFF;
    PORT_DATA  = data;

    do{
        z80_clock_pulse();
    }while( z80_rd_asserted() );

    DDR_DATA   = 0x00;
    PORT_DATA  = 0x00;
}

inline void z80_clock_pulse_while_writing(void)
{
    do{
        z80_clock_pulse();
    }while( z80_wr_asserted() );
}

void z80_reset(void)
{
    report("Z80 CPU Reset\n");
    z80_set_reset(true);
    for(int i=0; i<10; i++){ // Z80 requires at least 3 clocks to fully reset
        z80_set_clk(false);
        z80_set_clk(true);
    }
    z80_set_reset(false);
}
