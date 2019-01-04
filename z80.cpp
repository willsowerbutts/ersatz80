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

#undef MODE_SWITCH_DEBUGGING   // debug switching between Z80 interface modes
/*
 * === Z80 modes ===
 * +----------------- +-------------+---------+------+
 * |                  | Z80 clock   | Tracing | SRAM |
 * +----------------- +-------------+---------+------+
 * | Z80_UNSUPERVISED | independent | no      | yes  |
 * | Z80_SUPERVISED   | synthesised | yes     | yes  |
 * | Z80_ENCHANTED    | synthesised | no      | no   |
 * +----------------- +-------------+---------+------+
 *
 * Z80_UNSUPERVISED: the Z80 runs from an asynchronous clock which is generated
 * by an external crystal oscillator or the Teensy FTM.
 * 
 * Z80_SUPERVISED: the Z80 runs from a synthesised clock generated in software
 * on the Teensy. Clock edges appear under Teensy program control but the
 * spacing between them (ie max clock frequency) is still moderated by the FTM.
 *
 * Z80_ENCHANTED: Like Z80_SUPERVISED, but additionally the SRAM is disabled so
 * the Teensy can feed in synthesised instructions. This is used to access the
 * Z80 internal state without disrupting the contents of SRAM.
 *
 *
 * === DMA modes ===
 * +-----------+------------+---------+---------+--------+
 * |           | Bus master | Control | Address | Data   |
 * +-----------+------------+---------+---------+--------+
 * | DMA_SLAVE | Z80        | input   | input   | input  |
 * | DMA_IDLE  | Teensy     | input   | input   | input  |
 * | DMA_READ  | Teensy     | output  | output  | input  |
 * | DMA_WRITE | Teensy     | output  | output  | output |
 * +-----------+------------+---------+---------+--------+
 *
 * DMA_SLAVE: Normally the Z80 is bus master, ie in control of the memory and
 * I/O control signal, address and data lines.
 *
 * DMA_IDLE: The Z80 is in DMA mode, we can control the bus but we are not
 * driving any control, address or data lines. We end up here at the end of
 * an I/O instruction because DMA is used to synchronise the release of the
 * Z80 WAIT signal.
 *
 * DMA_READ: We are driving control and address lines.
 *
 * DMA_WRITE: We are driving control, address and data lines.
 */

z80_mode_t z80_mode = Z80_SUPERVISED;
dma_mode_t dma_mode = DMA_SLAVE;
int ram_pages;     // count of 16KB SRAM pages

#ifdef MODE_SWITCH_DEBUGGING
void z80_check_mode_correct(void)
{
    // this just checks that the interface between the Z80 and the Teensy
    // appears to be correctly configured for the mode we think we're in.
    switch(z80_mode){
        case Z80_UNSUPERVISED:
            assert(!z80_clk_is_supervised());
            assert(!z80_get_reset());
            assert(z80_get_ram_ce());
            break;
        case Z80_SUPERVISED:
            assert(z80_clk_is_supervised());
            assert(!z80_get_reset());
            assert(z80_get_ram_ce());
            break;
        case Z80_ENCHANTED:
            assert(z80_clk_is_supervised());
            assert(!z80_get_reset());
            assert(!z80_get_ram_ce());
            break;
    }

    switch(dma_mode){
        case DMA_SLAVE:
            assert(*portOutputRegister(Z80_BUSRQ));
            assert(!z80_busack_asserted());
            assert(!*portModeRegister(Z80_A0));
            assert(!*portModeRegister(Z80_D0));
            break;
        case DMA_IDLE:
            assert(!*portOutputRegister(Z80_BUSRQ));
            assert(z80_busack_asserted());
            assert(!*portModeRegister(Z80_A0));
            assert(!*portModeRegister(Z80_D0));
            break;
        case DMA_READ:
            assert(!*portOutputRegister(Z80_BUSRQ));
            assert(z80_busack_asserted());
            assert(*portModeRegister(Z80_A0));
            assert(!*portModeRegister(Z80_D0));
            break;
        case DMA_WRITE:
            assert(!*portOutputRegister(Z80_BUSRQ));
            assert(z80_busack_asserted());
            assert(*portModeRegister(Z80_A0));
            assert(*portModeRegister(Z80_D0));
            break;
    }
}
#endif

void z80_clock_pulse_drive_data(uint8_t data)
{
    z80_bus_data_outputs();
    z80_bus_set_data(data);

    do{
        z80_clock_pulse();
    }while( z80_rd_asserted() );

    z80_bus_data_inputs();
}

void z80_clock_pulse_while_writing(void)
{
    do{
        z80_clock_pulse();
    }while( z80_wr_asserted() );
}

void z80_do_reset(void)
{
    bool supervised;

    supervised = clk_is_supervised();

    if(!supervised)
        clk_set_supervised(true);

    z80_set_reset(true);
    z80_set_release_wait(true);
    for(int i=0; i<10; i++) // Z80 requires at least 3 clocks to fully reset
        z80_clock_pulse();
    z80_set_release_wait(false);
    z80_set_reset(false);

    if(!supervised)
        clk_set_supervised(false);
}

void z80_memory_write(uint16_t address, uint8_t data)
{
    z80_enter_dma_mode(true);
    z80_bus_set_address_data(address, data);
    z80_set_mreq(true);
    z80_set_wr(true);
    delayMicroseconds(1);
    z80_set_wr(false);
    z80_set_mreq(false);
}

uint8_t z80_memory_read(uint16_t address)
{
    uint8_t byte;
    z80_enter_dma_mode(false);
    z80_bus_set_address(address);
    z80_set_mreq(true);
    z80_set_rd(true);
    delayMicroseconds(1);
    byte = z80_read_bus_data();
    z80_set_rd(false);
    z80_set_mreq(false);
    return byte;
}

void z80_memory_write_block(uint16_t address, const uint8_t *dataptr, uint16_t count)
{
    if(count == 0)
        return;

    z80_enter_dma_mode(true);

    z80_set_mreq(true);
    while(count){
        z80_bus_set_address_data(address, *(dataptr++));
        z80_set_wr(true);
        count--;
        address++;
        z80_set_wr(false);
    }
    z80_set_mreq(false);
}

void z80_memory_read_block(uint16_t address, uint8_t *dataptr, uint16_t count)
{
    if(count == 0)
        return;

    z80_enter_dma_mode(false);

    z80_set_mreq(true);
    z80_set_rd(true);
    while(count){
        z80_bus_set_address(address);
        address++;
        count--;
        *(dataptr++) = z80_read_bus_data();
    }
    z80_set_rd(false);
    z80_set_mreq(false);
}

inline void z80_complete_read(uint8_t data)
{
    z80_bus_data_outputs();
    z80_bus_set_data(data);
    z80_set_busrq(true);
    z80_set_release_wait(true);
    while(!z80_busack_asserted())
        if(z80_supervised_mode())
            z80_clock_pulse();
    z80_bus_data_inputs();
    z80_set_release_wait(false);
    dma_mode = DMA_IDLE;
}

inline void z80_complete_write(void)
{
    z80_set_busrq(true);
    z80_set_release_wait(true);
    while(!z80_busack_asserted())
        if(z80_supervised_mode())
            z80_clock_pulse();
    z80_set_release_wait(false);
    dma_mode = DMA_IDLE;
}

void handle_z80_bus(void)
{
    if(z80_wait_asserted()){
        if(z80_iorq_asserted()){
            if(z80_rd_asserted()){       // I/O read
                z80_complete_read(iodevice_read(z80_read_bus_address()));
            }else if(z80_wr_asserted()){ // I/O write
                z80_complete_write();
                iodevice_write(z80_read_bus_address_low8(), z80_read_bus_data());
            }else if(z80_m1_asserted()){ // Interrupt acknowledge
                z80_complete_read(z80_irq_vector());
            }else
                report("ersatz80: iorq weird?\r\n");
        } else if(z80_mreq_asserted()){
            if(z80_rd_asserted()){       // Memory read
                z80_complete_read(memory_read(z80_read_bus_address()));
            }else if(z80_wr_asserted()){ // Memory write
                z80_complete_write();
                memory_write(z80_read_bus_address_low8(), z80_read_bus_data());
            }else
                report("ersatz80: mreq weird?\r\n");
        } else
            report("ersatz80: wait weird?\r\n");
        // z80_end_dma_mode(); // Hmmmmmm. Can we safely get rid of this?
    }
}

const char *z80_mode_name(z80_mode_t mode)
{
    switch(mode){
        case Z80_SUPERVISED:   return "Z80_SUPERVISED";
        case Z80_UNSUPERVISED: return "Z80_UNSUPERVISED";
        case Z80_ENCHANTED:    return "Z80_ENCHANTED";
        default: assert(false);
    }
}

const char *dma_mode_name(dma_mode_t mode)
{
    switch(mode){
        case DMA_SLAVE:         return "DMA_SLAVE";
        case DMA_IDLE:          return "DMA_IDLE";
        case DMA_READ:          return "DMA_READ";
        case DMA_WRITE:         return "DMA_WRITE";
        default: assert(false);
    }
}

void z80_enter_dma_mode(bool writing)
{
    switch(dma_mode){
        case DMA_WRITE:
            if(!writing){
                z80_bus_data_inputs();
                dma_mode = DMA_READ;
            }
            break;
        case DMA_READ:
            if(writing){
                z80_bus_data_outputs();
                dma_mode = DMA_WRITE;
            }
            break;
        case DMA_SLAVE:
            switch(z80_mode){
                case Z80_UNSUPERVISED:
                    do{
                        z80_set_busrq(true); // handle_z80_bus() may clear this
                        handle_z80_bus();
                    } while(!z80_busack_asserted());
                    break;
                case Z80_SUPERVISED:
                    z80_set_ram_ce(false);
                    z80_enchanted_cpu_write(0x18); // JR ...
                    z80_enchanted_cpu_write(0xFE); // ... -2
                    z80_set_busrq(true);
                    z80_set_ram_ce(true);
                    while(!z80_busack_asserted())
                        z80_clock_pulse();
                    break;
                case Z80_ENCHANTED:
                    assert(false); // this isn't supported (yet...)
                    break;
            }
            // fall through ...
        case DMA_IDLE:
            if(writing){
                z80_bus_master(true);
                dma_mode = DMA_WRITE;
            }else{
                z80_bus_master(false);
                dma_mode = DMA_READ;
            }
            break;
    }
#ifdef MODE_SWITCH_DEBUGGING
    z80_check_mode_correct();
#endif
}

void z80_end_dma_mode(void)
{
    switch(dma_mode){
        case DMA_SLAVE:
            break;
        case DMA_WRITE:
        case DMA_READ:
            z80_bus_slave();
            // fall through ...
        case DMA_IDLE:
            z80_set_busrq(false);
            dma_mode = DMA_SLAVE;
            break;
    }
    switch(z80_mode){
        case Z80_UNSUPERVISED:
            break;
        case Z80_SUPERVISED:
            z80_bus_slave();
            z80_set_busrq(false);
            while(z80_busack_asserted())
                z80_clock_pulse();
            // TODO may need to clock forward here until M1 T1 is reached?
            break;
        case Z80_ENCHANTED:
            assert(false); // this isn't supported (yet...)
    }
#ifdef MODE_SWITCH_DEBUGGING
    z80_check_mode_correct();
#endif
}

z80_mode_t z80_set_mode(z80_mode_t new_mode)
{
#ifdef MODE_SWITCH_DEBUGGING
    z80_check_mode_correct();
#endif

    if(z80_mode == new_mode)
        return z80_mode;

    z80_end_dma_mode();

    switch(z80_mode){
        case Z80_UNSUPERVISED:
            switch(new_mode){
                case Z80_SUPERVISED:
                    clk_set_supervised(true);
                    // TODO: trace a few opcodes until we know we are in sync with the Z80's decoder
                    break;
                default:
                    assert(false);
            }
            break;

        case Z80_SUPERVISED:
            switch(new_mode){
                case Z80_ENCHANTED:
                    z80_set_ram_ce(false);
                    break;
                case Z80_UNSUPERVISED:
                    clk_set_supervised(false);
                    break;
                default:
                    assert(false);
            }
            break;

        case Z80_ENCHANTED:
            switch(new_mode){
                case Z80_UNSUPERVISED:
                    z80_set_ram_ce(true);
                    break;
                default:
                    assert(false);
            }
            break;

        default:
            assert(false);
    }

    z80_mode_t prev_mode = z80_mode;
    z80_mode = new_mode;
#ifdef MODE_SWITCH_DEBUGGING
    z80_check_mode_correct();
#endif
    return prev_mode;
}

bool z80_supervised_mode(void)
{
    return (z80_mode != Z80_UNSUPERVISED);
}
