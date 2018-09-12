#ifndef __Z80_DOT_H__
#define __Z80_DOT_H__

void shift_register_update(void);
void z80_setup(void);
void z80_do_reset(void);
void z80_set_clk(bool level);
void z80_set_reset(bool active);
void z80_clock_pulse_drive_data(uint8_t data);
void z80_clock_pulse_while_writing(void);
void z80_clock_pulse(void);
bool z80_wr_asserted(void);
bool z80_rd_asserted(void);
bool z80_iorq_asserted(void);
bool z80_wait_asserted(void);
bool z80_busack_asserted(void);
bool z80_m1_asserted(void);
bool z80_mreq_asserted(void);
void z80_set_busrq(bool request_dma);
uint16_t z80_bus_address(void);
uint8_t z80_bus_address_low8(void);
uint8_t z80_bus_data(void);
void z80_set_release_wait(bool release);
void z80_start_fast_clock(void);
void z80_setup_drive_data(uint8_t data);
void z80_shutdown_drive_data(void);
void z80_set_mmu(int bank, uint8_t page);

extern unsigned short user_led;
extern bool z80_reset;
extern bool z80_irq;
extern bool z80_nmi;
extern bool ram_ce;
extern bool z80_bus_trace;
extern uint8_t mmu[4];

#endif
