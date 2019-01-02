#ifndef __Z80_DOT_H__
#define __Z80_DOT_H__

#include <Arduino.h>
#include "interface.h"

enum z80_register_t {
    Z80_REG_AF,
    Z80_REG_BC,
    Z80_REG_DE,
    Z80_REG_HL,
    Z80_REG_IX,
    Z80_REG_IY,
    Z80_REG_AF_ALT,
    Z80_REG_BC_ALT,
    Z80_REG_DE_ALT,
    Z80_REG_HL_ALT,
    Z80_REG_PC,
    Z80_REG_SP,
    Z80_REG_I
};

void z80_clock_pulse_drive_data(uint8_t data); // send clock pulses and drive data bus while Z80 asserts RD
void z80_clock_pulse_while_writing(void);      // send clock pulses while Z80 asserts WR
void z80_do_reset(void);                       // reset Z80 CPU
uint8_t z80_memory_read(uint16_t address);
void z80_memory_read_block(uint16_t address, uint8_t *dataptr, uint16_t count);
void z80_memory_write(uint16_t address, uint8_t data);
void z80_memory_write_block(uint16_t address, const uint8_t *dataptr, uint16_t count);

void z80_set_register(z80_register_t reg, uint16_t value);
void mmu_setup(void);
void sram_setup(void);
void load_program_to_sram(const uint8_t *program, uint16_t address, uint16_t length, uint16_t start_address);
int load_file_to_sram(char *filename, uint16_t address);
void z80_set_busrq(bool request_dma);
void z80_set_release_wait(bool release);

void z80_show_regs(void);
void z80_bus_trace_state(void);
void handle_z80_bus(void);

// TR_SILENT: trace bus states but do not report anything
// TR_INST:   trace bus states and print decoded instructions
// TR_BUS:    trace and print bus states and decoded instructions
typedef enum { TR_SILENT, TR_INST, TR_BUS } z80_bus_trace_t;
extern z80_bus_trace_t z80_bus_trace;
extern int instruction_clock_cycles; // updated only when z80_bus_trace != TR_OFF

typedef enum {                   // Z80 clock   | Tracing | Bus Master | SRAM 
// ------------------------------//-------------+---------+------------+------
    MODE_UNSUPERVISED,           // independent | no      | Z80        | yes  
    MODE_UNSUPERVISED_DMA_IDLE,  // independent | no      | none       | yes  
    MODE_UNSUPERVISED_DMA_READ,  // independent | no      | Teensy (R) | yes  
    MODE_UNSUPERVISED_DMA_WRITE, // independent | no      | Teensy (W) | yes  
    MODE_SUPERVISED,             // synthesised | yes     | Z80        | yes  
    MODE_SUPERVISED_DMA_IDLE,    // synthesised | yes     | none       | yes  
    MODE_SUPERVISED_DMA_READ,    // synthesised | yes     | Teensy (R) | yes  
    MODE_SUPERVISED_DMA_WRITE,   // synthesised | yes     | Teensy (W) | yes  
    MODE_ENCHANTED,              // synthesised | no      | Z80        | no   
} z80_mode_t;

extern z80_mode_t z80_mode;
z80_mode_t z80_set_mode(z80_mode_t new_mode); // returns previous mode
z80_mode_t z80_set_dma_mode(bool writing);    // returns previous mode
z80_mode_t z80_end_dma_mode(void);            // returns previous mode
bool z80_supervised_mode(void);               // is the clock synthesised in this mode?
const char *z80_mode_name(z80_mode_t mode);

extern int ram_pages;     // count of 16KB SRAM pages

// console
void uart_setup(int baud);
extern bool uart0_on_console;

#endif
