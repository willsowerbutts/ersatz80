#include <Arduino.h>
#include <assert.h>
#include "rom.h"
#include "debug.h"
#include "clock.h"
#include "z80.h"
#include "z80io.h"
#include "mmu.h"
#include "irq.h"
#include "disk.h"
#include "disasm.h"

z80_bus_trace_t z80_bus_trace = TR_SILENT;
int instruction_clock_cycles = 0;

enum bus_cycle_t { MEM_READ, MEM_WRITE, IO_READ, IO_WRITE, NO_CYCLE };

typedef struct {
    bus_cycle_t cycle;
    uint16_t address;
    uint8_t data;
} bus_trace_t;

#define MAX_BUS_STATES 8 // presumably this can be lower??
static bus_trace_t bus_trace[MAX_BUS_STATES];
static bool bus_trace_wait_cycle_end = false;
static int bus_trace_count = 0;
static int bus_trace_di = 0;

static uint8_t read_bus_trace_bytes(void)
{
    return bus_trace[bus_trace_di++].data;
}

static void z80_instruction_ended(void)
{
    char output[20];
    uint16_t addr;
    bus_cycle_t cycle;

    if(z80_bus_trace >= TR_INST){
        bus_trace_di = 0;
        z80ctrl_disasm(read_bus_trace_bytes, output);
        report("%-14s %2d  ", output, instruction_clock_cycles);

        addr = ~bus_trace[0].address;
        for(int i=0; i<bus_trace_count; i++){
            if(bus_trace[i].address != addr || cycle != bus_trace[i].cycle){
                addr = bus_trace[i].address;
                cycle = bus_trace[i].cycle;
                switch(bus_trace[i].cycle){
                    case MEM_READ:  report("%04x: ", bus_trace[i].address); break;
                    case MEM_WRITE: report("%04x<-", bus_trace[i].address); break;
                    case IO_READ:   report("io%04x: ", bus_trace[i].address); break;
                    case IO_WRITE:  report("io%04x<-", bus_trace[i].address); break;
                    case NO_CYCLE:  break; // should never happen
                }
            }
            report("%02x%s", bus_trace[i].data, i == (bus_trace_count-1) ? "" : (i == (bus_trace_di-1) ? " / " : " "));
            addr++;
        }
        report("\r\n");
    }

    instruction_clock_cycles = 0;
}

void z80_bus_trace_state(void)
{
    bus_cycle_t type = NO_CYCLE;

    instruction_clock_cycles++;
    if(bus_trace_wait_cycle_end && !(z80_mreq_asserted() || z80_iorq_asserted()))
        bus_trace_wait_cycle_end = false;
    else if(!bus_trace_wait_cycle_end && !z80_wait_asserted()){
        if(z80_mreq_asserted()){
            if(z80_rd_asserted()){
                type = MEM_READ;
            }else if(z80_wr_asserted()){
                type = MEM_WRITE;
            }
        }else if(z80_iorq_asserted()){
            if(z80_rd_asserted()){
                type = IO_READ;
            }else if(z80_wr_asserted()){
                type = IO_WRITE;
            }
        }
        if(type != NO_CYCLE){
            if(z80_m1_asserted() && !(bus_trace_count == 1 && (bus_trace[0].data == 0xcb || bus_trace[0].data == 0xdd || bus_trace[0].data == 0xed || bus_trace[0].data == 0xfd))){
                z80_instruction_ended();
                bus_trace_count = 0;
            }
            bus_trace_wait_cycle_end = true;
            bus_trace[bus_trace_count].cycle = type;
            bus_trace[bus_trace_count].data = z80_read_bus_data();
            bus_trace[bus_trace_count].address = z80_read_bus_address();
            if(bus_trace_count < (MAX_BUS_STATES-1))
                bus_trace_count++;
        }
    }

    if(z80_bus_trace >= TR_BUS)
        report("\r\n|%04x|%02x|%s|%s|%s|",
                z80_read_bus_address(), z80_read_bus_data(),
                z80_mreq_asserted() ? "MREQ" : (z80_iorq_asserted() ? "IORQ" : "    "),
                z80_rd_asserted() ? "RD" : (z80_wr_asserted() ? "WR" : "  "),
                z80_m1_asserted() ? "M1" : "  ");
}

bool z80_supervised_mode(void)
{
    switch(z80_mode){
        case Z80_UNSUPERVISED:
            return false;
        case Z80_SUPERVISED:
        case Z80_ENCHANTED:
            return true;
    }
    assert(false);
}
