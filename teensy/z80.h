#ifndef __Z80_DOT_H__
#define __Z80_DOT_H__

#include <Arduino.h>

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


void shift_register_update(void);
void z80_bus_master(void);
void z80_bus_slave(void);
void z80_setup(void);
void z80_do_reset(void);
void z80_set_clk(bool level);
void z80_set_reset(bool active);
void z80_clock_pulse_drive_data(uint8_t data);
void z80_clock_pulse_while_writing(void);
void z80_clock_pulse(void);
void z80_set_busrq(bool request_dma);
uint16_t z80_bus_address(void);
uint8_t z80_bus_address_low8(void);
uint8_t z80_bus_data(void);
void z80_set_release_wait(bool release);
void z80_start_fast_clock(void);
void z80_setup_drive_data(uint8_t data);
void z80_shutdown_drive_data(void);
void z80_set_mmu(int bank, uint8_t page);

extern uint16_t user_led;
extern bool z80_reset;
extern bool z80_irq;
extern bool z80_nmi;
extern bool ram_ce;
extern bool z80_bus_trace;
extern uint8_t mmu[4];

#ifdef KINETISK
inline bool z80_wr_asserted(void)      { return !*portInputRegister(Z80_WR);     } 
inline bool z80_busack_asserted(void)  { return !*portInputRegister(Z80_BUSACK); } 
inline bool z80_rd_asserted(void)      { return !*portInputRegister(Z80_RD);     } 
inline bool z80_iorq_asserted(void)    { return !*portInputRegister(Z80_IORQ);   } 
inline bool z80_wait_asserted(void)    { return !*portInputRegister(Z80_WAIT);   } 
inline bool z80_m1_asserted(void)      { return !*portInputRegister(Z80_M1);     } 
inline bool z80_mreq_asserted(void)    { return !*portInputRegister(Z80_MREQ);   } 
inline void z80_set_busrq(bool request_dma)    { *portOutputRegister(Z80_BUSRQ)  = !request_dma; }
inline void z80_set_release_wait(bool release) { *portOutputRegister(WAIT_RESET) = !release; }
#else
inline bool z80_wr_asserted(void)      { return !digitalRead(Z80_WR);     } 
inline bool z80_busack_asserted(void)  { return !digitalRead(Z80_BUSACK); } 
inline bool z80_rd_asserted(void)      { return !digitalRead(Z80_RD);     } 
inline bool z80_iorq_asserted(void)    { return !digitalRead(Z80_IORQ);   } 
inline bool z80_wait_asserted(void)    { return !digitalRead(Z80_WAIT);   } 
inline bool z80_m1_asserted(void)      { return !digitalRead(Z80_M1);     } 
inline bool z80_mreq_asserted(void)    { return !digitalRead(Z80_MREQ);   } 
inline void z80_set_busrq(bool request_dma)    { digitalWrite(Z80_BUSRQ,  !request_dma); } 
inline void z80_set_release_wait(bool release) { digitalWrite(WAIT_RESET, !release); } 
#endif

#endif
