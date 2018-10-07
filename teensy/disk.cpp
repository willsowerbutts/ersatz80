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

void disk_error(const char *opname, const char *msg)
{
    report("disk %d %s (%d-byte sector 0x%x, count %d): %s\r\n",
            disk_selected, opname, 
            1 << disk[disk_selected].sector_size_log,
            disk[disk_selected].sector_number,
            disk[disk_selected].sector_count,
            msg);
    disk[disk_selected].error = true;
}

void disk_transfer(bool write)
{
    uint8_t iobuf[MAX_SECTOR_SIZE];
    uint32_t offset;
    int remain, count, maxsec, bytes, r;
    const char *opname = write ? "write" : "read";

    if(!disk[disk_selected].mounted){
        disk_error(opname, "access to unmounted disk");
        return;
    }

    if(write && !disk[disk_selected].writable){
        disk_error(opname, "write to read-only disk");
        return;
    }

    offset = disk[disk_selected].sector_number << disk[disk_selected].sector_size_log;
    if(offset + (disk[disk_selected].sector_count << disk[disk_selected].sector_size_log) > disk[disk_selected].size_bytes){
        disk_error(opname, "access beyond end of disk");
        return;
    }

    if(!disk[disk_selected].file.seekSet(offset)){
        disk_error(opname, "file seek failed");
        return;
    }

    begin_dma();

    maxsec = MAX_SECTOR_SIZE >> disk[disk_selected].sector_size_log;
    remain = disk[disk_selected].sector_count;
    while(remain){
        count = (remain > maxsec) ? maxsec : remain;
        bytes = count << disk[disk_selected].sector_size_log;
        if(write){
            z80_memory_read_block(disk[disk_selected].dma_address, iobuf, bytes);
            r = disk[disk_selected].file.write(iobuf, bytes);
            //report("disk %d: write %d bytes = %d\r\n", disk_selected, bytes, r);
            if(r < bytes){
                disk_error(opname, "file write failed");
                remain = 0; // no more iterations
            }
        }else{
            r = disk[disk_selected].file.read(iobuf, bytes);
            // report("disk %d: read %d bytes = %d\r\n", disk_selected, bytes, r);
            if(r < 0){
                disk_error(opname, "file read failed");
                remain = 0; // no more iterations
                r = 0; // indicate 0 bytes in sector buffer are valid for memset
            }
            if(r < bytes){
                report("disk %d: WARNING: padding incomplete read %d/%d?\r\n", disk_selected, r, bytes);
                memset(&iobuf[r], 0, bytes - r); // wipe unread portion of sector buffer
            }
            z80_memory_write_block(disk[disk_selected].dma_address, iobuf, bytes);
        }
        // advance our pointers
        disk[disk_selected].dma_address += bytes;
        disk[disk_selected].sector_number += count;
        remain -= count;
    }

    end_dma();
}

void disk_unmount(void)
{
    disk[disk_selected].file.sync();
    disk[disk_selected].file.close();
    disk[disk_selected].mounted = false;
    disk[disk_selected].writable = false;
    disk[disk_selected].error = true; // error=true always for unmounted drives
    disk[disk_selected].size_bytes = 0;
}

void disk_mount(void)
{
    bool okay;
    char filename[64];

    sprintf(filename, "test%d.dsk", disk_selected); // we need to read our params from the DMA address. being lazy for now.

    if(disk[disk_selected].mounted)
        disk_unmount();

    if(disk_file_mounted(filename)){
        report("disk %d: cannot mount file \"%s\": already mounted.\r\n", disk_selected, filename);
        return;
    }

    okay = disk[disk_selected].file.open(&sdcard, filename, O_RDWR | O_CREAT);
    disk[disk_selected].sector_number = 0;
    disk[disk_selected].error = !okay;
    disk[disk_selected].mounted = okay;
    disk[disk_selected].writable = okay;
    if(okay){
        disk[disk_selected].size_bytes = disk[disk_selected].file.fileSize();
        if(disk[disk_selected].size_bytes & 0x3FF)
            report("disk %d: WARNING: size of \"%s\" is not a multiple of 1024\r\n", disk_selected, filename);
    }else
        disk[disk_selected].size_bytes = 0;
}

void disk_seek_final_sector(bool exact_device_size)
{
    // we seek to either:
    //  - size of the device, ie one sector past the last usable sector (exact_device_size=true)
    //  - the last usable sector, ie sector (n-1) for an n sector device (exact_device_size=false)
    disk[disk_selected].sector_number = (disk[disk_selected].size_bytes >> disk[disk_selected].sector_size_log) - (exact_device_size ? 0 : 1);
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
            disk_transfer(false);
            break;
        case 0x21: // perform write operation
            disk_transfer(true);
            break;
        case 0x22: // perform mount operation
            disk_mount();
            break;
        case 0x23: // perform unmount operation
            disk_unmount();
            break;
        case 0x24: // seek to final sector
            disk_seek_final_sector(false);
            break;
        case 0x25: // set sector number to device sector count (useful to read out exact device size)
            disk_seek_final_sector(true);
            break;
        case 0x26: // perform sync operation
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

bool disk_file_mounted(const char *new_filename)
{
    char filename[MAX_FILENAME_LENGTH];

    for(int d=0; d<NUM_DISK_DRIVES; d++){
        if(disk[d].mounted){
            disk[d].file.getName(filename, MAX_FILENAME_LENGTH);
            // hmm is this really adequate? what about .., multiple path separators, 8.3 vs long file name etc?
            if(strcasecmp(new_filename, filename) == 0)
                return true;
        }
    }

    return false;
}

void disk_init(void) {
    for(int d=0; d<NUM_DISK_DRIVES; d++) {
        disk[d].sector_number = 0;
        disk[d].dma_address = 0;
        disk[d].sector_size_log = 9; // 2^9 = 512 bytes
        disk[d].sector_count = 1;
        disk[d].size_bytes = 0;
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

#define FORMAT_BUFFER_SIZE 4096
#define FORMAT_BYTE_VALUE 0xE5
bool disk_format(const char *filename, uint32_t bytes)
{
    SdBaseFile file;
    unsigned long timer;
    uint32_t progress, done, last_prog = 0;
    int xfer;
    char buffer[FORMAT_BUFFER_SIZE];
    bool result = true;

    if(disk_file_mounted(filename)){
        report("disk: cannot format a mounted disk\r\n");
        return false;
    }

    // could check if file exists and if so refuse to proceed?
    
    report("disk: format \"%s\" (%d bytes)  [                ]\x08\x08\x08\x08\x08\x08\x08\x08\x08\x08\x08\x08\x08\x08\x08\x08\x08", filename, bytes);
    progress = bytes / 16;

    timer = micros();
    if(!file.open(&sdcard, filename, O_WRONLY | O_CREAT | O_TRUNC)){
        report("disk: failed to open file \"%s\"\r\n", filename);
        return false;
    }

    memset(buffer, FORMAT_BYTE_VALUE, FORMAT_BUFFER_SIZE);
    done = 0;
    while(bytes > 0){
        xfer = (bytes > FORMAT_BUFFER_SIZE) ? FORMAT_BUFFER_SIZE : bytes;
        if(file.write(buffer, xfer) != xfer){
            report("disk: write() failed during format\r\n");
            bytes = 0; // abort
            result = false;
        }else{
            bytes -= xfer;
            done += xfer;
        }
        if(done / progress != last_prog){
            last_prog = done / progress;
            report("=");
        }
    }

    file.sync();
    file.close();

    timer = micros() - timer;
    report("]  %.1fMB/sec\r\n", ((float)done / (1024.0*1024.0)) / ((float)timer / 1000000.0f));

    return result;
}
