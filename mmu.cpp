#include <Arduino.h>
#include <assert.h>
#include "mmu.h"
#include "debug.h"
#include "z80.h"

static uint8_t mmu[4];         // Z80 local MMU context
static uint8_t mmu_foreign[4]; // Z80 foreign MMU context
static uint8_t mmu_shadow[4];  // Actual MMU register contents
static bool mmu_context_foreign = false;

void mmu_setup(void)
{
    mmu_context_foreign = false;
    for(int i=0; i<4; i++){
        mmu_foreign[i] = i;
        mmu_shadow[i] = ~i; // force an update
        z80_set_mmu(i, i);
        z80_set_mmu_foreign(i, i);
    }
}

static void z80_mmu_write(uint8_t bank, uint8_t page)
{
    if(mmu_shadow[bank] == page)
        return;
    mmu_shadow[bank] = page;
    z80_set_dma_mode(true);
    z80_bus_set_address_data(bank << 14, page);
    z80_set_mmu_ew(true);
    delayMicroseconds(1);
    z80_set_mmu_ew(false);
}

uint8_t z80_get_mmu(int bank)
{
    assert(bank >= 0 && bank <= 3);
    return mmu[bank];
}

uint8_t z80_get_mmu_foreign(int bank)
{
    assert(bank >= 0 && bank <= 3);
    return mmu_foreign[bank];
}

void z80_set_mmu(int bank, uint8_t page)
{
    assert(bank >= 0 && bank <= 3);
    mmu[bank] = page;
    if(!mmu_context_foreign)
        z80_mmu_write(bank, page);
}

void z80_set_mmu_foreign(int bank, uint8_t page)
{
    assert(bank >= 0 && bank <= 3);
    mmu_foreign[bank] = page;
    if(mmu_context_foreign)
        z80_mmu_write(bank, page);
}

void z80_mmu_switch_context_local(void)
{
    mmu_context_foreign = false;
    for(int b=0; b<4; b++)
        z80_mmu_write(b, mmu[b]);
}

void z80_mmu_switch_context_foreign(void)
{
    mmu_context_foreign = true;
    for(int b=0; b<4; b++)
        z80_mmu_write(b, mmu_foreign[b]);
}
