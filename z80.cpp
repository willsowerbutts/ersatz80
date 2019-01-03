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
    z80_clk_pause(false);
    z80_set_reset(true);
    z80_set_release_wait(true);
    for(int i=0; i<10; i++){ // Z80 requires at least 3 clocks to fully reset
        z80_set_clk(false);
        z80_set_clk(true);
    }
    z80_set_release_wait(false);
    z80_set_reset(false);
    z80_clk_resume();
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
        if(!z80_clk_is_independent())
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
        if(!z80_clk_is_independent())
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
                    z80_clk_set_supervised(0.0);
                    // TODO: handle HALT etc
                    // TODO: trace a few opcodes until we know we are in sync with the decoder
                    while(!z80_m1_asserted() || z80_mreq_asserted()){ // stop at start of M1
                        z80_clock_pulse();
                        handle_z80_bus();
                    }
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
                    z80_clk_set_independent(CLK_FAST_FREQUENCY);
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

// Z80 CPU conducts a write to memory
uint8_t z80_enchanted_cpu_read(uint16_t *address)
{
    uint8_t data;

    // wait for write cycle to begin
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();

    // capture the cycle
    if(address)
        *address = z80_read_bus_address();
    data = z80_read_bus_data();

    // wait for write cycle to complete
    do{
        z80_clock_pulse();
    }while( z80_wr_asserted() );

    // return value written
    return data;
}

// Z80 CPU conducts a read from memory
uint16_t z80_enchanted_cpu_write(uint8_t data)
{
    uint16_t address;

    // wait for read cycle to begin
    while(!(z80_mreq_asserted() && z80_rd_asserted()))
        z80_clock_pulse();

    // capture address
    address = z80_read_bus_address();

    // put the byte on the data bus
    z80_bus_data_outputs();
    z80_bus_set_data(data);

    // wait for the read cycle to complete
    do{
        z80_clock_pulse();
    }while( z80_rd_asserted() );

    // release the data bus
    z80_bus_data_inputs();

    // return address read
    return address;
}

uint16_t z80_enchanted_cpu_read16(uint16_t *address)
{
    return (z80_enchanted_cpu_read(address) << 8) | z80_enchanted_cpu_read();
}

uint16_t z80_enchanted_cpu_write16(uint16_t value)
{
    uint16_t r;
    r = z80_enchanted_cpu_write(value);
    z80_enchanted_cpu_write(value >> 8);
    return r;
}

static uint16_t z80_set_register_swap_registers(z80_register_t reg)
{
    switch(reg){
        case Z80_REG_AF_ALT:
            return z80_enchanted_cpu_write(0x08);   // EX AF, AF'
        case Z80_REG_BC_ALT:
        case Z80_REG_DE_ALT:
        case Z80_REG_HL_ALT:
            return z80_enchanted_cpu_write(0xD9);   // EXX
        default:
            return 0;
    }
}

void z80_set_register(z80_register_t reg, uint16_t value)
{
    uint16_t orig_pc, orig_sp, orig_af;

    // Sequence of instructions for loading each register:
    //
    //     PC:                                                        JP xx xx
    //     SP:                                          LD SP, xx xx; JP xx xx
    //     AF:              POP AF <xx xx>;             LD SP, xx xx; JP xx xx
    //     BC:              LD BC, xx xx;                             JP xx xx
    //     DE:              LD DE, xx xx;                             JP xx xx
    //     HL:              LD HL, xx xx;                             JP xx xx
    //     IX:              LD IX, xx xx;                             JP xx xx
    //     IY:              LD IY, xx xx;                             JP xx xx
    // AF_ALT:  EX AF, AF'; POP AF <xx xx>; EX AF, AF'; LD SP, xx xx; JP xx xx
    // BC_ALT:  EXX;        LD BC, xx xx;   EXX;                      JP xx xx
    // DE_ALT:  EXX;        LD DE, xx xx;   EXX;                      JP xx xx
    // HL_ALT:  EXX;        LD HL, xx xx;   EXX;                      JP xx xx
    //      I:  PUSH AF <xx xx>; LD A, xx; LD I, a; POP AF <xx xx>; LD SP, xx xx; JP xx xx
    z80_mode_t prev_mode = z80_set_mode(Z80_ENCHANTED);

    switch(reg){
        case Z80_REG_I:
            orig_pc = z80_enchanted_cpu_write(0xF5);  // PUSH AF
            orig_af = z80_enchanted_cpu_read16(&orig_sp);
            z80_enchanted_cpu_write(0x3E);  // LD A, xx
            z80_enchanted_cpu_write(value); // new value for I
            z80_enchanted_cpu_write(0xED);  // LD I, A
            z80_enchanted_cpu_write(0x47);  // (2-byte instruction)
            z80_enchanted_cpu_write(0xF1);  // POP AF
            z80_enchanted_cpu_write16(orig_af);
            break;
        case Z80_REG_PC:
            orig_pc = value;
            break;
        case Z80_REG_SP:
            orig_pc = z80_enchanted_cpu_write(0x31);     // LD SP
            break;
        case Z80_REG_AF:
            orig_pc = z80_enchanted_cpu_write(0xF1);     // POP AF -- can't do LD AF
            break;
        case Z80_REG_BC:
            orig_pc = z80_enchanted_cpu_write(0x01);     // LD BC
            break;
        case Z80_REG_DE:
            orig_pc = z80_enchanted_cpu_write(0x11);     // LD DE
            break;
        case Z80_REG_HL:
            orig_pc = z80_enchanted_cpu_write(0x21);     // LD HL
            break;
        case Z80_REG_AF_ALT:
            orig_pc = z80_set_register_swap_registers(reg);
            z80_enchanted_cpu_write(0xF1);               // POP AF -- can't do LD AF
            break;
        case Z80_REG_BC_ALT:
            orig_pc = z80_set_register_swap_registers(reg);
            z80_enchanted_cpu_write(0x01);               // LD BC
            break;
        case Z80_REG_DE_ALT:
            orig_pc = z80_set_register_swap_registers(reg);
            z80_enchanted_cpu_write(0x11);               // LD DE
            break;
        case Z80_REG_HL_ALT:
            orig_pc = z80_set_register_swap_registers(reg);
            z80_enchanted_cpu_write(0x21);               // LD HL
            break;
        case Z80_REG_IX:
            orig_pc = z80_enchanted_cpu_write(0xDD);     // ...
            z80_enchanted_cpu_write(0xE1);               // POP IX
            break;
        case Z80_REG_IY:
            orig_pc = z80_enchanted_cpu_write(0xFD);     // ...
            z80_enchanted_cpu_write(0xE1);               // POP IY
            break;
    }

    // where required, feed in the data value (and capture SP for POP instructions)
    switch(reg){
        case Z80_REG_I:
        case Z80_REG_PC:
            break;
        default:
            orig_sp = z80_enchanted_cpu_write16(value);
            break;
    }

    // where required, restore SP
    switch(reg){
        case Z80_REG_AF:
        case Z80_REG_AF_ALT:
        case Z80_REG_IX:
        case Z80_REG_IY:
        case Z80_REG_I:
            z80_enchanted_cpu_write(0x31);      // LD SP, xxxx
            z80_enchanted_cpu_write16(orig_sp + 1);
            break;
        default:
            break;
    }

    // switch register set back (NOP if not required)
    z80_set_register_swap_registers(reg);

    // pick up in real memory back where we started
    z80_enchanted_cpu_write(0xC3);              // JP, xxxx
    z80_enchanted_cpu_write16(orig_pc);

    z80_set_mode(prev_mode);
}

void z80_show_regs(void)
{
    uint16_t pc, sp, af, bc, de, hl, ix, iy, af_, bc_, de_, hl_;
    uint8_t i;

    // this code does not deal with the situation where CPU is HALTed.
    // solution might be: wake CPU with an int/nmi, capture PC when it writes it to the
    //                    stack. capture regs as usual. then IRET and JP to PC-1, then
    //                    feed it a HALT when it fetches PC-1. it will HALT with PC correct.

    // take over the CPU
    z80_mode_t prev_mode = z80_set_mode(Z80_ENCHANTED);

    // we feed it an F5 (PUSH AF) instruction
    pc = z80_enchanted_cpu_write(0xF5); // PUSH AF; capture PC

    af = z80_enchanted_cpu_read16(&sp); // capture both SP and AF
    sp += 1;                            // adjust SP to previous value

    z80_enchanted_cpu_write(0xC5);      // PUSH BC
    bc = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xD5);      // PUSH DE
    de = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xE5);      // PUSH HL
    hl = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0x08);      // EX AF, AF'
    z80_enchanted_cpu_write(0xF5);      // PUSH AF
    af_ = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0x08);      // EX AF, AF' again (swap back)
    z80_enchanted_cpu_write(0xD9);      // EXX
    z80_enchanted_cpu_write(0xC5);      // PUSH BC
    bc_ = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xD5);      // PUSH DE
    de_ = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xE5);      // PUSH HL
    hl_ = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xD9);      // EXX again (swap back)
    z80_enchanted_cpu_write(0xDD);      // IX prefix
    z80_enchanted_cpu_write(0xE5);      // PUSH IX
    ix = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xFD);      // IY prefix
    z80_enchanted_cpu_write(0xE5);      // PUSH IY
    iy = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xED);      // ED prefix
    z80_enchanted_cpu_write(0x57);      // LD A,I - note this affects the flags register

    z80_enchanted_cpu_write(0xF5);      // PUSH AF
    i = z80_enchanted_cpu_read16()>>8;  // I was copied into A (top bits)

    // finally we need to put AF, SP and PC back as they were before our tinkering
    z80_enchanted_cpu_write(0xF1);      // POP af
    z80_enchanted_cpu_write16(af);
    z80_enchanted_cpu_write(0x31);      // LD SP, xxxx
    z80_enchanted_cpu_write16(sp);
    z80_enchanted_cpu_write(0xC3);      // JP xxxx
    z80_enchanted_cpu_write16(pc);

    // release the CPU
    z80_set_mode(prev_mode);

    report("PC=%04x SP=%04x\r\nAF=%04x AF'=%04x\r\n" \
           "BC=%04x BC'=%04x\r\nDE=%04x DE'=%04x\r\n" \
           "HL=%04x HL'=%04x\r\nIX=%04x IY=%04x I=%02x\r\n",
           pc, sp, af, af_,
           bc, bc_, de, de_,
           hl, hl_, ix, iy, i);
}
