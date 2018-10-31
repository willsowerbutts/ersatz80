#ifndef __Z80_DOT_H__
#define __Z80_DOT_H__

#include <Arduino.h>

// uncomment this line for rev1 PCBs only:
// #define ERSATZ80_PCB_REV1

#ifdef ERSATZ80_PCB_REV1
// 2018-09-03 PCB rev1
const int CLK_FAST_ENABLE       = 8;        // GPIO D 3
const int CLK_STROBE            = 7;        // GPIO D 2
const int MMU_EW                = 15;       // GPIO C 0
const int SHIFT_REGISTER_DATA   = 11;       // GPIO C 6
const int SHIFT_REGISTER_LATCH  = 12;       // GPIO C 7
const int SHIFT_REGISTER_CLK    = 13;       // GPIO C 5
const int WAIT_RESET            = 6;        // GPIO D 4
const int Z80_A0                = 16;       // GPIO B 0
const int Z80_A1                = 17;       // GPIO B 1
const int Z80_A2                = 18;       // GPIO B 3
const int Z80_A3                = 19;       // GPIO B 2
const int Z80_A4                = 20;       // GPIO D 5
const int Z80_A5                = 21;       // GPIO D 6
const int Z80_A6                = 22;       // GPIO C 1
const int Z80_A7                = 23;       // GPIO C 2
const int Z80_A8                = 24;       // GPIO E 26
const int Z80_A9                = 25;       // GPIO A 5
const int Z80_A10               = 26;       // GPIO A 14
const int Z80_A11               = 27;       // GPIO A 15
const int Z80_A12               = 28;       // GPIO A 16
const int Z80_A13               = 29;       // GPIO B 18
const int Z80_A14               = 30;       // GPIO B 19
const int Z80_A15               = 31;       // GPIO B 10
const int Z80_BUSACK            = 9;        // GPIO C 3
const int Z80_BUSRQ             = 14;       // GPIO D 1
const int Z80_D0                = 32;       // GPIO B 11
const int Z80_D1                = 33;       // GPIO E 24
const int Z80_D2                = 34;       // GPIO E 25
const int Z80_D3                = 35;       // GPIO C 8
const int Z80_D4                = 36;       // GPIO C 9
const int Z80_D5                = 37;       // GPIO C 10
const int Z80_D6                = 38;       // GPIO C 11
const int Z80_D7                = 39;       // GPIO A 17
const int Z80_M1                = 4;        // GPIO A 13
const int Z80_RD                = 2;        // GPIO D 0
const int Z80_WR                = 3;        // GPIO A 12
const int Z80_IORQ              = 0;        // GPIO B 16
const int Z80_MREQ              = 1;        // GPIO B 17
const int Z80_HALT              = 10;       // GPIO C 4
const int Z80_WAIT              = 5;        // GPIO D 7
#else
// 2018-10-26 PCB rev2: pin mapping was optimised for simpler Teensy software
const int CLK_FAST_ENABLE       = 30;       // GPIO B 19
const int CLK_STROBE            = 29;       // GPIO B 18
const int MMU_EW                = 39;       // GPIO A 17
const int SHIFT_REGISTER_DATA   = 34;       // GPIO E 25
const int SHIFT_REGISTER_LATCH  = 24;       // GPIO E 26
const int SHIFT_REGISTER_CLK    = 33;       // GPIO E 24
const int WAIT_RESET            = 27;       // GPIO A 15
const int Z80_A0                = 15;       // GPIO C 0
const int Z80_A1                = 22;       // GPIO C 1
const int Z80_A2                = 23;       // GPIO C 2
const int Z80_A3                = 9;        // GPIO C 3
const int Z80_A4                = 10;       // GPIO C 4
const int Z80_A5                = 13;       // GPIO C 5
const int Z80_A6                = 11;       // GPIO C 6
const int Z80_A7                = 12;       // GPIO C 7
const int Z80_A8                = 35;       // GPIO C 8
const int Z80_A9                = 36;       // GPIO C 9
const int Z80_A10               = 37;       // GPIO C 10
const int Z80_A11               = 38;       // GPIO C 11
const int Z80_A12               = 16;       // GPIO B 0
const int Z80_A13               = 17;       // GPIO B 1
const int Z80_A14               = 19;       // GPIO B 2
const int Z80_A15               = 18;       // GPIO B 3
const int Z80_BUSACK            = 4;        // GPIO A 13
const int Z80_BUSRQ             = 3;        // GPIO A 12
const int Z80_D0                = 2;        // GPIO D 0
const int Z80_D1                = 14;       // GPIO D 1
const int Z80_D2                = 7;        // GPIO D 2
const int Z80_D3                = 8;        // GPIO D 3
const int Z80_D4                = 6;        // GPIO D 4
const int Z80_D5                = 20;       // GPIO D 5
const int Z80_D6                = 21;       // GPIO D 6
const int Z80_D7                = 5;        // GPIO D 7
const int Z80_M1                = 25;       // GPIO A 5
const int Z80_RD                = 1;        // GPIO B 17
const int Z80_WR                = 0;        // GPIO B 16
const int Z80_IORQ              = 31;       // GPIO B 10
const int Z80_MREQ              = 32;       // GPIO B 11
const int Z80_HALT              = 26;       // GPIO A 14
const int Z80_WAIT              = 28;       // GPIO A 16
#endif

void shift_register_update(void);
void z80_bus_master(void);
void z80_bus_slave(void);
void z80_setup(void);
void mmu_setup(void);
void sram_setup(void);
void z80_memory_write_block(uint16_t address, const uint8_t *dataptr, uint16_t count);
void z80_memory_read_block(uint16_t address, uint8_t *dataptr, uint16_t count);
void load_program_to_sram(const uint8_t *program, uint16_t address, uint16_t length, uint16_t start_address);
int load_file_to_sram(char *filename, uint16_t address);
void z80_do_reset(void);
void z80_set_reset(bool active);
void z80_clock_pulse_drive_data(uint8_t data);
void z80_clock_pulse_while_writing(void);
void z80_set_busrq(bool request_dma);
uint16_t z80_bus_address(void);
uint8_t z80_bus_address_low8(void);
uint8_t z80_bus_data(void);
void z80_set_release_wait(bool release);
void z80_setup_drive_data(uint8_t data);
void z80_shutdown_drive_data(void);
void z80_set_mmu(int bank, uint8_t page);
void z80_mmu_switch_context_local(void);
void z80_mmu_switch_context_foreign(void);

void z80_show_regs(void);
void z80_show_pin_states(void);
void z80_bus_trace_state(void);
void handle_z80_bus(void);
void begin_dma(void);
void end_dma(void);
void z80_set_pc(uint16_t address);

extern uint16_t user_led;
extern bool z80_reset;
extern bool z80_irq;
extern bool z80_nmi;
extern bool ram_ce;
extern int z80_bus_trace;
extern uint8_t mmu[4];
extern uint8_t mmu_foreign[4];
extern uint8_t ram_pages; // count of 16KB SRAM pages

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

// teensy.ino:
uint8_t iodevice_read(uint16_t address);
void iodevice_write(uint16_t address, uint8_t value); // call ONLY when in DMA mode!
uint8_t memory_read(uint16_t address);
void memory_write(uint16_t address, uint8_t value);

#endif
