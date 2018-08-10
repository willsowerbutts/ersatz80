#ifndef __Z80_DOT_H__
#define __Z80_DOT_H__

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

//extern uint32_t z80_clk_counter;
void z80_init(void);
void z80_read_bus(z80_bus_state *state);
void z80_set_clk(bool level);
void z80_set_reset(bool active);
void z80_clock_pulse_drive_data(uint8_t data);
void z80_clock_pulse_while_writing(void);
void z80_clock_pulse(void);
bool z80_wr_asserted(void);
bool z80_rd_asserted(void);
bool z80_iorq_asserted(void);
bool z80_m1_asserted(void);
bool z80_mreq_asserted(void);
uint16_t z80_bus_address(void);
uint8_t z80_bus_data(void);

#endif
