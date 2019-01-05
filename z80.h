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
extern int instruction_clock_cycles; // not updated in Z80_UNSUPERVISED mode

typedef enum {         // Z80 clock   | Tracing | SRAM 
// --------------------//-------------+---------+------
    Z80_UNSUPERVISED,  // independent | no      | yes  
    Z80_SUPERVISED,    // synthesised | yes     | yes  
    Z80_ENCHANTED,     // synthesised | no      | no   
} z80_mode_t;
extern z80_mode_t z80_mode;

typedef enum {         // Bus master | Control | Address | Data
// --------------------//------------+---------+---------+--------
    DMA_SLAVE,         // Z80        | input   | input   | input
    DMA_IDLE,          // Teensy     | input   | input   | input
    DMA_READ,          // Teensy     | output  | output  | input
    DMA_WRITE,         // Teensy     | output  | output  | output
} dma_mode_t;
extern dma_mode_t dma_mode;

z80_mode_t z80_set_mode(z80_mode_t new_mode); // returns previous mode
void z80_enter_dma_mode(bool writing);
void z80_end_dma_mode(void);
const char *z80_mode_name(z80_mode_t mode);
const char *dma_mode_name(dma_mode_t mode);

uint8_t z80_enchanted_cpu_read(uint16_t *address=NULL);
uint16_t z80_enchanted_cpu_write(uint8_t data);
uint16_t z80_enchanted_cpu_read16(uint16_t *address=NULL);
uint16_t z80_enchanted_cpu_write16(uint16_t value);

extern int ram_pages;     // count of 16KB SRAM pages

// console
void uart_setup(int baud);
extern bool uart0_on_console;

#endif
