#include <Arduino.h>
#include "debug.h"
#include "disk.h"

#define MAX_SECTOR_SIZE 1024
SdFatSdioEX sdcard;

// Z80 accessible register map:
//   base + 0 -- unimplemented; reserved for future expansion
//   base + 1 -- R/W: selected disk sector number (bits 16--23)
//   base + 2 -- R/W: selected disk sector number (bits 8--15)
//   base + 3 -- R/W: selected disk sector number (bits 0--7)
//   base + 4 -- R/W: selected disk DMA address (bits 16--23; top 2 bits are flags)
//   base + 5 -- R/W: selected disk DMA address (bits 8--15)
//   base + 6 -- R/W: selected disk DMA address (bits 0--7)
//   base + 7 -- R/W: selected disk sector count
//   base + 8 -- R: controller status register, W: controller command register
//               status register bits:
//               0--3 - selected disk drive number (0--15)
//               4--5 - selected disk sector size
//                        00 = 128 bytes
//                        01 = 256 bytes
//                        10 = 512 bytes
//                        11 = 1024 bytes
//                  6 - selected disk read-write flag (0 = read-only, 1 = read-write)
//                  7 - selected disk error flag (0 = OK, 1 = error)
//                  would be nice if there was some way to read out an error code also? put in sec num?

disk_info_t disk[NUM_DISK_DRIVES];
uint8_t disk_selected;

uint8_t disk_sector_read(uint16_t address)
{
    if(disk_selected >= NUM_DISK_DRIVES)
        return 0;
    return disk[disk_selected].sector_number >> (8 * (3 - (address & 3)));
}

void disk_sector_write(uint16_t address, uint8_t value)
{
    if(disk_selected >= NUM_DISK_DRIVES)
        return;
    disk[disk_selected].sector_number = (disk[disk_selected].sector_number
        & ~(0xff << (8 * (3 - (address & 3)))))  // mask off the relevant octet
        | (value << (8 * (3 - (address & 3))));  // and set it to the new value
}

uint8_t disk_address_read(uint16_t address)
{
    if(disk_selected >= NUM_DISK_DRIVES)
        return 0;
    return disk[disk_selected].dma_address >> (8 * (2 - (address & 3)));
}

void disk_address_write(uint16_t address, uint8_t value)
{
    if(disk_selected >= NUM_DISK_DRIVES)
        return;
    disk[disk_selected].dma_address = (disk[disk_selected].dma_address 
        & ~(0xff << (8 * (2 - (address & 3)))))  // mask out the relevant octet
        | (value << (8 * (2 - (address & 3))));  // and set it to the new value
}

uint8_t disk_seccount_read(uint16_t address)
{
    if(disk_selected >= NUM_DISK_DRIVES)
        return 0;
    return disk[disk_selected].sector_count;
}

void disk_seccount_write(uint16_t address, uint8_t value)
{
    if(disk_selected >= NUM_DISK_DRIVES)
        return;
    disk[disk_selected].sector_count = value;
}

uint8_t disk_status_read(uint16_t address)
{
    if(disk_selected >= NUM_DISK_DRIVES)
        return 0x80; // error flag always set for non-existent drives
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
    uint8_t iobuf[MAX_SECTOR_SIZE];
    int remain, count, maxsec, bytes, r;

    if(!disk[disk_selected].writable){
        disk[disk_selected].error = true;
        return;
    }

    begin_dma();

    maxsec = MAX_SECTOR_SIZE >> disk[disk_selected].sector_size_log;
    disk[disk_selected].file.seekSet(disk[disk_selected].sector_number << disk[disk_selected].sector_size_log);
    remain = disk[disk_selected].sector_count;
    while(remain){
        count = (remain > maxsec) ? maxsec : remain;
        bytes = count << disk[disk_selected].sector_size_log;
        z80_memory_read_block(disk[disk_selected].dma_address, iobuf, bytes);
        r = disk[disk_selected].file.write(iobuf, bytes);
        if(r < bytes)
            disk[disk_selected].error = true; // report error condition
        // advance our pointers
        disk[disk_selected].dma_address += bytes;
        disk[disk_selected].sector_number += count;
        remain -= count;
    }

    end_dma();
}

void disk_mount(void)
{
    bool okay;
    char filename[64];
    // we need to read our params from the DMA address. being lazy for now.
    // NOTE *we* need to ensure that no two disks have the same image mounted concurrently!
    sprintf(filename, "test%d.dsk", disk_selected);
    okay = disk[disk_selected].file.open(&sdcard, filename, O_RDWR | O_CREAT);
    disk[disk_selected].sector_number = 0;
    disk[disk_selected].error = !okay;
    disk[disk_selected].mounted = okay;
    disk[disk_selected].writable = okay;
}

void disk_unmount(void)
{
    disk[disk_selected].file.close();
    disk[disk_selected].mounted = false;
    disk[disk_selected].writable = false;
    disk[disk_selected].error = true; // error=true always for unmounted drives
}

void disk_seek_final_sector(void)
{
    disk[disk_selected].sector_number = (disk[disk_selected].file.fileSize()-1) >> disk[disk_selected].sector_size_log;
}

void disk_command_write(uint16_t address, uint8_t value)
{
    // for non-existent drives, only allow the select drive commands (0x00 ... 0x0F)
    if(disk_selected >= NUM_DISK_DRIVES && value > 0x0F)
        return;

    switch(value){
        case 0x00 ... 0x0F: // select active drive number (only 0...NUM_DISK_DRIVES-1 exist)
            disk_selected = value;
            break;
        case 0x10: // set sector size 128 bytes (2^7)
        case 0x11: // set sector size 256 bytes (2^8)
        case 0x12: // set sector size 512 bytes (2^9)
        case 0x13: // set sector size 1024 bytes (2^10)
            disk[disk_selected].sector_size_log = value - 9; // 0x10 ... 0x13 -> 7 ... 10
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
        case 0x25: // perform sync operation
            disk_sync(); // this syncs all the drives, but the command guarantees only syncing the selected drive
            break;
        case 0x80: // clear error flag
            // unmounted drives always indicate error
            disk[disk_selected].error = !disk[disk_selected].mounted;
            break;
        default:
            report("disk %d: unimplemented command 0x%02x!\r\n", disk_selected, value);
    }
}

// this is called periodically from the main loop
void disk_sync(void){
    for(int d=0; d<NUM_DISK_DRIVES; d++)
        if(disk[d].mounted)
            disk[d].file.sync();
}

void disk_init(void) {
    for(int d=0; d<NUM_DISK_DRIVES; d++) {
        disk[d].sector_number = 0;
        disk[d].dma_address = 0;
        disk[d].sector_size_log = 9; // 2^9 = 512 bytes
        disk[d].sector_count = 1;
        disk[d].error = true;       // error=true always for unmounted drives
        disk[d].writable = false;
        disk[d].mounted = false;
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
