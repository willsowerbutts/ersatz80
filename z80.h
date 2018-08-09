#ifndef __Z80_DOT_H__
#define __Z80_DOT_H__

typedef struct {
    uint8_t data;
    uint16_t address;
    uint8_t iorq : 1;
    uint8_t mreq : 1;
    uint8_t m1 : 1;
    uint8_t rd : 1;
    uint8_t wr : 1;
    uint8_t reset : 1;
    uint8_t clk : 1;
} z80_bus_state;

void z80_init(void);
void z80_read_bus(z80_bus_state *state);
void z80_set_clk(bool level);
void z80_set_reset(bool active);
void z80_clock_pulse_drive_data(uint8_t data);
void z80_clock_pulse(void);

#endif
