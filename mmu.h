#ifndef __MMU_DOT_H__
#define __MMU_DOT_H__

void z80_set_mmu(int bank, uint8_t page);
void z80_set_mmu_foreign(int bank, uint8_t page);
uint8_t z80_get_mmu(int bank);
uint8_t z80_get_mmu_foreign(int bank);
void z80_mmu_switch_context_local(void);
void z80_mmu_switch_context_foreign(void);

#endif
