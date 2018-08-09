#include <assert.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "z80.h"

#define Z80_DATA_BUS_PORT  C
#define Z80_ADDRL_BUS_PORT A
#define Z80_ADDRH_BUS_PORT L

#define DDR_DATA        DDRC
#define DDR_ADDRL       DDRA
#define DDR_ADDRH       DDRL

#define PORT_DATA       PORTC
#define PORT_ADDRL      PORTA
#define PORT_ADDRH      PORTL

#define PIN_DATA        PINC
#define PIN_ADDRL       PINA
#define PIN_ADDRH       PINL

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

bool bus_master;
bool bus_drive_data;

void z80_init(void)
{
    bus_master = false;
    bus_drive_data = false;

    // data/address and pins
    DDR_DATA  = 0x00;
    DDR_ADDRL = 0x00;
    DDR_ADDRH = 0x00;
    // pull-ups on data bus only
    PORT_DATA  = 0x00;
    PORT_ADDRL = 0x00;
    PORT_ADDRH = 0x00;

    // control signal - inputs
    DDRB &= ~(_BV(1) | _BV(2) | _BV(3));
    DDRD &= ~(_BV(7));
    DDRG &= ~(_BV(1));
    PORTB &= ~(_BV(1) | _BV(2) | _BV(3));
    PORTD &= ~(_BV(7));
    PORTG &= ~(_BV(1));

    // control signal - outputs
    DDRG |= (_BV(0) | _BV(2));
    PORTG |= (_BV(0)); // CLK high
    PORTG &= ~(_BV(2)); // /RESET asserted
}

void z80_read_bus(z80_bus_state *state)
{
    uint8_t b, d, g;

    assert(bus_drive_data == false);
    assert(bus_master == false);

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

void z80_set_clk(bool level)
{
    if(level)
        PORTG |= (_BV(0)); // CLK high
    else
        PORTG &= ~(_BV(0)); // CLK low
}

void z80_set_reset(bool active)
{
    if(active)
        PORTG &= ~(_BV(2)); // /RESET low (asserted)
    else
        PORTG |= (_BV(2)); // /RESET high
}

void z80_clock_pulse(void)
{
    z80_set_clk(false);
    z80_set_clk(true);
}

void z80_clock_pulse_drive_data(uint8_t data)
{
    DDR_DATA   = 0xFF;
    PORT_DATA  = data;
    do{
        z80_clock_pulse();
    }while( !(PING & _BV(1)) ); // while /RD is asserted

    PORT_DATA  = 0x00;
    DDR_DATA   = 0x00;
}
