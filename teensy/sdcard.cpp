#include <Arduino.h>
#include "debug.h"
#include "sdcard.h"

#define NUM_DISK_DRIVES 16
#define MAX_SECTOR_SIZE 1024
SdFatSdioEX sdcard; // can also try using SdFatSdio here -- it may be slower?

// Z80 accessible register map:
//   base + 0 -- R/W: disk sector number (bits 24--31)
//   base + 1 -- R/W: disk sector number (bits 16--23)
//   base + 2 -- R/W: disk sector number (bits 8--15)
//   base + 3 -- R/W: disk sector number (bits 0--7)
//   base + 4 -- R/W: DMA address (bits 16--23; top 2 bits are flags)
//   base + 5 -- R/W: DMA address (bits 8--15)
//   base + 6 -- R/W: DMA address (bits 0--7)
//   base + 7 -- R/W: sector count
//   base + 8 -- R: status register, W: command register
//               status register bits:
//               0--3 - selected disk number (0--15)
//               4--5 - selected sector size
//                        00 = 128 bytes
//                        01 = 256 bytes
//                        10 = 512 bytes
//                        11 = 1024 bytes
//                  6 - read-write flag (0 = read-only, 1 = read-write)
//                  7 - error flag (0 = OK, 1 = error)
//                  would be nice if there was some way to read out an error code also?

typedef struct {
    SdBaseFile file;
    uint32_t   sector_number;
    uint32_t   dma_address;
    uint8_t    sector_size_log;  // stored as power of 2; 7=128, 8=256, 9=512, 10=1024.
    uint8_t    sector_count;
    bool       error;
    bool       writable;
} disk_info_t;

disk_info_t disk[NUM_DISK_DRIVES];
uint8_t disk_selected;

uint8_t disk_sector_read(uint16_t address)
{
    return disk[disk_selected].sector_number >> (8 * (3 - (address & 3)));
}

void disk_sector_write(uint16_t address, uint8_t value)
{
    disk[disk_selected].sector_number = (disk[disk_selected].sector_number
        & ~(0xff << (8 * (3 - (address & 3)))))  // mask off the relevant octet
        | (value << (8 * (3 - (address & 3))));  // and set it to the new value
}

uint8_t disk_address_read(uint16_t address)
{
    return disk[disk_selected].dma_address >> (8 * (3 - (address & 3)));
}

void disk_address_write(uint16_t address, uint8_t value)
{
    disk[disk_selected].dma_address = (disk[disk_selected].dma_address 
        & ~(0xff << (8 * (3 - (address & 3)))))  // mask out the relevant octet
        | (value << (8 * (3 - (address & 3))));  // and set it to the new value
}

uint8_t disk_seccount_read(uint16_t address)
{
    return disk[disk_selected].sector_count;
}

void disk_seccount_write(uint16_t address, uint8_t value)
{
    disk[disk_selected].sector_count = value;
}

uint8_t disk_status_read(uint16_t address)
{
    return (disk_selected & 0x0F) |                                 // bits 0--3: selected disk number
           ((disk[disk_selected].sector_size_log-7) << 4) |         // bits 4--5: selected sector size
           (disk[disk_selected].writable ? 0x40: 0x00) |            // bit 6: read/write flag
           (disk[disk_selected].error ? 0x80 : 0x00);               // bit 7: error flag
}

void disk_transfer_read(void)
{
    uint8_t iobuf[MAX_SECTOR_SIZE];
    int remain, count, maxsec, bytes, r;

    begin_dma();

    maxsec = MAX_SECTOR_SIZE >> disk[disk_selected].sector_size_log;
    disk[disk_selected].file.seekSet(disk[disk_selected].sector_number << disk[disk_selected].sector_size_log);
    remain = disk[disk_selected].sector_count;
    while(remain){
        count = (remain > maxsec) ? maxsec : remain;
        bytes = count << disk[disk_selected].sector_size_log;
        r = disk[disk_selected].file.read(iobuf, bytes);
        if(r < 0){
            disk[disk_selected].error = true; // report error condition
            r = 0; // indicate 0 bytes in sector buffer are valid for memset
        }
        if(r < bytes)
            memset(&iobuf[r], 0, bytes - r); // wipe unread portion of sector buffer
        z80_memory_write_block(disk[disk_selected].dma_address, iobuf, bytes);
        // advance our pointers
        disk[disk_selected].dma_address += bytes;
        disk[disk_selected].sector_number += count;
        remain -= count;
    }

    end_dma();
}

void disk_transfer_write(void)
{
    // TODO
}

void disk_mount(void)
{
    disk[disk_selected].file.open(&sdcard, "basic.rom", FILE_READ);
    // TODO error handling!
}

void disk_unmount(void)
{
    disk[disk_selected].file.close();
}

void disk_seek_final_sector(void)
{
    // TODO
}

void disk_command_write(uint16_t address, uint8_t value)
{
    switch(value){
        case 0x00 ... 0x0F: // select active drive number
            if(value < NUM_DISK_DRIVES)
                disk_selected = value;
            else
                report("disk: attempt to select unavailable drive %d!\r\n", value);
                // do we need a global error flag also?
            break;
        case 0x10: // set sector size
            disk[disk_selected].sector_size_log = 7;
            break;
        case 0x11: // set sector size
            disk[disk_selected].sector_size_log = 8;
            break;
        case 0x12: // set sector size
            disk[disk_selected].sector_size_log = 9;
            break;
        case 0x13: // set sector size
            disk[disk_selected].sector_size_log = 10;
            break;
        case 0x20: // perform read operation
            disk_transfer_read();
            break;
        case 0x21: // perform write operation
            disk_transfer_write();
            break;
        case 0x22: // perform mount operation
            disk_mount();
            break;
        case 0x23: // perform unmount operation
            disk_unmount();
            break;
        case 0x24: // seek to final sector (useful to determine device size)
            disk_seek_final_sector();
            break;
        case 0x80: // clear error flag
            disk[disk_selected].error = false;
            break;
        default:
            report("disk: unimplemented command 0x%02x!\r\n", value);
            // do we need a global error flag also?
    }
}

void sdcard_init() {
    for(int d=0; d<NUM_DISK_DRIVES; d++) {
        disk[d].sector_number = 0;
        disk[d].dma_address = 0;
        disk[d].sector_size_log = 9; // 2^9 = 512 bytes
        disk[d].sector_count = 1;
        disk[d].error = false;
        disk[d].writable = true;
    }

    SdioCard *card;

    report("ersatz80: initializing SD card: ");

    if (!sdcard.begin()) {
        report("failed!\r\n");
        return;
    }

    card = sdcard.card();
    switch(card->type()){
        case 0:
            report("SDv1");
            break;
        case 1:
            report("SDv2");
            break;
        case 3:
            report("SDHC");
            break;
        default:
            report("(unknown card type)");
    }

    report(" %d blocks (%.1fGB) FAT%d\r\n", card->cardSize(), (float)card->cardSize() / 2097152.0, sdcard.fatType());
}
