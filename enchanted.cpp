#include <Arduino.h>
#include <assert.h>
#include "debug.h"
#include "clock.h"
#include "z80.h"

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
        default:
            orig_pc = 0;
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
