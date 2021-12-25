#ifndef __INTERFACE_DOT_H__
#define __INTERFACE_DOT_H__
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

bool check_pcb_revision(void);              // attempt to determine if ERSATZ80_PCB_REV1 is set correctly
void z80_setup(void);                       // setup Z80 pins
void z80_show_pin_states(void);             // debugging -- report state of all Teensy pins connected to Z80

void z80_set_ram_ce(bool active);           // control SRAM CE pin
void z80_set_irq(bool active);              // control Z80 IRQ pin
void z80_set_nmi(bool active);              // control Z80 NMI pin
void z80_set_reset(bool active);            // control Z80 RESET pin
bool z80_get_ram_ce(void);
bool z80_get_irq(void);
bool z80_get_nmi(void);
bool z80_get_reset(void);


void z80_set_user_leds(uint16_t leds);      // write 12-bit user LEDs bitmask
uint16_t z80_get_user_leds(void);           // read 12-bit user LEDs bitmask

uint16_t z80_read_bus_address(void);        // read address bus (16 bits)
uint8_t z80_read_bus_address_low8(void);    // read address bus (low 8 bits only)
uint8_t z80_read_bus_data(void);            // read data bus

void z80_bus_master(bool writing);          // make control, address (and optionally data) pins outputs
void z80_bus_slave(void);                   // make control, data and address pins inputs
void z80_bus_data_outputs(void);            // make data pins outputs
void z80_bus_data_inputs(void);             // make data pins inputs

void z80_bus_set_data(uint8_t data);        // set value on data pins (must be outputs)
void z80_bus_set_address(uint16_t address); // set value on address pins (must be outputs)
void z80_bus_set_address_data(uint16_t address, uint8_t data); // set value on address and data pins (must be outputs)

#ifdef KINETISK
inline bool z80_wr_asserted(void)              { return !*portInputRegister(Z80_WR);             }
inline bool z80_busack_asserted(void)          { return !*portInputRegister(Z80_BUSACK);         }
inline bool z80_rd_asserted(void)              { return !*portInputRegister(Z80_RD);             }
inline bool z80_iorq_asserted(void)            { return !*portInputRegister(Z80_IORQ);           }
inline bool z80_wait_asserted(void)            { return !*portInputRegister(Z80_WAIT);           }
inline bool z80_m1_asserted(void)              { return !*portInputRegister(Z80_M1);             }
inline bool z80_mreq_asserted(void)            { return !*portInputRegister(Z80_MREQ);           }
inline bool z80_halt_asserted(void)            { return !*portInputRegister(Z80_HALT);           }
inline void z80_set_busrq(bool request_dma)    { *portOutputRegister(Z80_BUSRQ)  = !request_dma; }
inline void z80_set_release_wait(bool release) { *portOutputRegister(WAIT_RESET) = !release;     }
inline void z80_set_mmu_ew(bool write)         { *portOutputRegister(MMU_EW)     = !write;       }
inline void z80_set_mreq(bool enable)          { *portOutputRegister(Z80_MREQ)   = !enable;      }
inline void z80_set_iorq(bool enable)          { *portOutputRegister(Z80_IORQ)   = !enable;      }
inline void z80_set_rd(bool enable)            { *portOutputRegister(Z80_RD)     = !enable;      }
inline void z80_set_wr(bool enable)            { *portOutputRegister(Z80_WR)     = !enable;      }
#else // not KINETISK
inline bool z80_wr_asserted(void)              { return !digitalRead(Z80_WR);                    }
inline bool z80_busack_asserted(void)          { return !digitalRead(Z80_BUSACK);                }
inline bool z80_rd_asserted(void)              { return !digitalRead(Z80_RD);                    }
inline bool z80_iorq_asserted(void)            { return !digitalRead(Z80_IORQ);                  }
inline bool z80_wait_asserted(void)            { return !digitalRead(Z80_WAIT);                  }
inline bool z80_m1_asserted(void)              { return !digitalRead(Z80_M1);                    }
inline bool z80_mreq_asserted(void)            { return !digitalRead(Z80_MREQ);                  }
inline bool z80_halt_asserted(void)            { return !digitalRead(Z80_HALT);                  }
inline void z80_set_busrq(bool request_dma)    { digitalWrite(Z80_BUSRQ,  !request_dma);         }
inline void z80_set_release_wait(bool release) { digitalWrite(WAIT_RESET, !release);             }
inline void z80_set_mmu_ew(bool write)         { digitalWrite(MMU_EW,     !write);               }
inline void z80_set_mreq(bool enable)          { digitalWrite(Z80_MREQ,   !enable);              }
inline void z80_set_iorq(bool enable)          { digitalWrite(Z80_IORQ,   !enable);              }
inline void z80_set_rd(bool enable)            { digitalWrite(Z80_RD,     !enable);              }
inline void z80_set_wr(bool enable)            { digitalWrite(Z80_WR,     !enable);              }
#endif // KINETISK

#endif
