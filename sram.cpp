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

#define MEMORY_WIPE_VALUE 0x76 // at reset, fill RAM with Z80 HALT instructions

static void z80_wipe_page(void) // wipe the 16KB memory page in bank 0 (0x0000 -- 0x3FFF)
{
    z80_enter_dma_mode(true);

    z80_bus_set_address_data(0, MEMORY_WIPE_VALUE);
    z80_set_mreq(true);
    z80_set_wr(true);
    for(int i=1; i<0x4000; i++){ // address 0 has already been written
        z80_bus_set_address(i);
    }
    z80_set_wr(false);
    z80_set_mreq(false);
}

void sram_setup(void)
{
    int i;
    uint8_t old_mmu = z80_get_mmu(0); // stash current MMU state

    report("ersatz80: wipe SRAM ");

    // we test the full 256*16KB=4MB address space
    for(i=0; i<256; i++){
        z80_set_mmu(0, i);
        // write a test pattern into each page of SRAM
        z80_memory_write(0, 0xaa);
        z80_memory_write(1, 0x55);
        z80_memory_write(2, 0x00);
        z80_memory_write(3, i);
        z80_memory_write(16380, i);
        z80_memory_write(16381, 0x01);
        z80_memory_write(16382, 0x56);
        z80_memory_write(16383, 0xab);
    }

    for(i=0; i<256; i++){
        z80_set_mmu(0, i);
        // is the test pattern there?
        if(z80_memory_read(0) != 0xaa ||
           z80_memory_read(1) != 0x55 ||
           z80_memory_read(2) != 0x00 ||
           z80_memory_read(3) != i ||
           z80_memory_read(16383) != 0xab ||
           z80_memory_read(16382) != 0x56 ||
           z80_memory_read(16381) != 0x01 ||
           z80_memory_read(16380) != i)
            break;
        // yup, it seems to be there! wipe it clean
        z80_wipe_page();
    }

    ram_pages = i;
    report("%d pages (%dKB)\r\n", ram_pages, 16*ram_pages);

    // return machine to previous state
    z80_set_mmu(0, old_mmu);
}

void load_program_to_sram(const uint8_t *program, uint16_t address, uint16_t length, uint16_t start_address)
{
    z80_enter_dma_mode(true);

    z80_memory_write_block(address, program, length);

    if(start_address != 0 && address != 0){
        z80_memory_write(0, 0xc3); // JP instruction
        z80_memory_write(1, start_address & 0xFF);
        z80_memory_write(2, start_address >> 8);
    }

    z80_end_dma_mode();
}

#define LOAD_BUFFER_SIZE 512
int load_file_to_sram(char *filename, uint16_t address)
{
    SdBaseFile file;
    uint8_t buffer[LOAD_BUFFER_SIZE];
    int r, total;

    // Open file
    if(!file.open(&sdcard, filename, O_RDONLY))
        return -1;

    // enter DMA mode
    z80_enter_dma_mode(true);

    // Load the file block by block
    total = 0;
    while(true){
        r = file.read(buffer, LOAD_BUFFER_SIZE);
        if(r <= 0)
            break;
        z80_memory_write_block(address, buffer, r);
        address += r;
        total += r;
    }

    file.close();
    z80_end_dma_mode();

    return total;
}
