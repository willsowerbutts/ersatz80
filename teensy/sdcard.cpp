#include <Arduino.h>
#include <SD.h>

#include "debug.h"
#include "z80.h"
#include "sdcard.h"

uint8_t sdcard_secnum_0 = 0x00;
uint8_t sdcard_secnum_1 = 0x00;
uint8_t sdcard_secnum_2 = 0x00;
uint8_t sdcard_secnum_3 = 0x00;
uint8_t sdcard_dma_addr_0 = 0x00;
uint8_t sdcard_dma_addr_1 = 0xe0;
uint8_t sdcard_dma_addr_2 = 0x80; // alternate modes not yet implemented
uint8_t sdcard_sec_count = 0x01;

uint8_t sdcard_status_byte = 0x80;
uint16_t sdcard_sector_size = 128;
uint16_t sdcard_dma_addr_local;
uint32_t sdcard_secnum;
uint32_t sdcard_dma_addr_global;

uint8_t sdcard_sec_buf[512];

// status byte bit definitions
// bit 0-3 - selected disk (note - disk 15 is currently the custom filename 
//           mounted disk)
// bit 4-5 - currently selected sector size in bytes 00 = 128, 01 = 256,
//           10 = 512, 11 = reserved for future expansion (1024 bytes?)
// bit 6   - RO / RW flag - 0 = RW, 1 = RO (not yet implemented)
// bit 7   - error flag - 0 = OK, 1 = error
// initial status has error flag set since no disk is selected yet

char sdcard_filename[12];
File sdcard_image_file;

uint8_t *reg_table[8] = {
    &sdcard_secnum_0,       // sdcard_base_port
    &sdcard_secnum_1,       // sdcard_base_port + 0x01
    &sdcard_secnum_2,       // sdcard_base_port + 0x02
    &sdcard_secnum_3,       // sdcard_base_port + 0x03
    &sdcard_dma_addr_0,     // sdcard_base_port + 0x04
    &sdcard_dma_addr_1,     // sdcard_base_port + 0x05
    &sdcard_dma_addr_2,     // sdcard_base_port + 0x06
    &sdcard_sec_count       // sdcard_base_port + 0x07
};

void sdcard_init() {
  report("ersatz80: initializing SD card...\r\n");
  
  if (!SD.begin(254)) {
        report("ersatz80: SD card initialization failed!\r\n");
  }
}

void sdcard_reg_wr(uint8_t address, uint8_t value) {
    *reg_table[address-sdcard_base_port] = value;
}

uint8_t sdcard_reg_rd(uint8_t address) {
    return *reg_table[address-sdcard_base_port];
}

void sdcard_cmd_wr(uint8_t value) {
    switch(value) {
        // select disks 0-14, file names are 'disk<nn>.img'
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07:
        case 0x08:
        case 0x09:
        case 0x0a:
        case 0x0b:
        case 0x0c:
        case 0x0d:
        case 0x0e:
            sdcard_select_prenamed_disk(value);
            break;
        // select disk 15, file name is an 8.3 name located at the
        // DMA address
        case 0x0f:
            report("Filename mounted disk not implemented yet");
            break;
        // set sector size
        case 0x10:
            sdcard_sector_size = 128;
            sdcard_status_byte = sdcard_status_byte & 0b11001111;
            break;
        case 0x11:
            sdcard_sector_size = 256;
            sdcard_status_byte = (sdcard_status_byte & 0b11001111) | 0b00010000;
            break;
        case 0x12:
            sdcard_sector_size = 512;
            sdcard_status_byte = (sdcard_status_byte & 0b11001111) | 0b00100000;
            break;
        // perform disk read
        case 0x13:
            sdcard_disk_read();
            break;
        // perform disk write
        case 0x14:
            sdcard_disk_write();
            break;
        // list sdcard dir to DMA addr as null-terminated string
        case 0x15:
            report("Directory featured not implemented yet");
            break;
        case 0xFE:
        // reset error flag after a failed operation
            sdcard_status_byte = sdcard_status_byte & 0b01111111;                                 
            break;
        case 0xFF:
        // debug listing of the sdcard routine variables
            sdcard_reg_debug();
            break;
    }
}   

void sdcard_select_prenamed_disk(uint8_t disknum) {    
    // close previous file if it was open
    sdcard_image_file.close();
    // build disk name filename
    sprintf(sdcard_filename, "disk%02d.img", disknum);
    // check if file exists, if so open, else error
    sdcard_image_file = SD.open(sdcard_filename,(O_READ | O_WRITE));
    if (sdcard_image_file) {
        sdcard_status_byte = ((sdcard_status_byte & 0b01110000) | disknum);
    } else {
        report(sdcard_filename);
        report(" does not exist \r\n");
        sdcard_status_byte = sdcard_status_byte | 0b10000000;                 
    }
}

void sdcard_disk_read() {
    if ((sdcard_dma_addr_2 >> 6) == 0b00000010) {
        // create a single variable representation of the sector number
        sdcard_secnum = (sdcard_secnum_3 << 24) | (sdcard_secnum_2 << 16) | (sdcard_secnum_1 << 8) | (sdcard_secnum_0);
        // create a single variable representatio of the DMA address
        sdcard_dma_addr_local = (sdcard_dma_addr_1 << 8) | (sdcard_dma_addr_0);
        // make sure there is enough data to read
        if (sdcard_image_file.size() < ((sdcard_sector_size * sdcard_secnum) + (sdcard_sector_size * sdcard_sec_count))) {
            report("not enough data available in file\r\n");
            report("starting byte location %d\r\n",(sdcard_sector_size * sdcard_secnum));
            report("ending byte location %d\r\n",((sdcard_sector_size * sdcard_secnum) + (sdcard_sector_size * sdcard_sec_count)));
            report("file is %d bytes\r\n",sdcard_image_file.size());
            sdcard_status_byte = sdcard_status_byte | 0b10000000; 
            return;
        }
        // make sure there is enough RAM to write (sector count * size)
        if (((sdcard_dma_addr_local-1) + (sdcard_sector_size * sdcard_sec_count)) > 0xffff) {
            report("not enough RAM available above DMA location\r\n");
            sdcard_status_byte = sdcard_status_byte | 0b10000000; 
            return;
        }
        // seek to the correct sector in the file
        if (!sdcard_image_file.seek(sdcard_sector_size * sdcard_secnum)) {
            report("error seeking starting byte %d\r\n",(sdcard_sector_size * sdcard_secnum));
            sdcard_status_byte = sdcard_status_byte | 0b10000000;  
            return;
        }
        
        // start DMA mode
        begin_dma();

        // stash current state
        bool old_ram_ce = ram_ce;

        // enable the SRAM
        ram_ce = true;
        shift_register_update();
        
        // loop for # of sectors required
        int j, i;
        for (j = sdcard_sec_count; j > 0; j--) {
            // get sector from SD card
            if (!sdcard_image_file.read(sdcard_sec_buf, sdcard_sector_size)) {
                report("error reading sector to teensy ram buffer\r\n");
                sdcard_status_byte = sdcard_status_byte | 0b10000000; 
            }

            // load 1 sector to SRAM 
            for (i = 0; i < sdcard_sector_size; i++) {
                z80_memory_write(sdcard_dma_addr_local++, sdcard_sec_buf[i]);
            }
        }

        // restore machine state
        ram_ce = old_ram_ce;
        shift_register_update();
        
        //stop DMA mode
        end_dma();        
    } else if ((sdcard_dma_addr_2 >> 6) == 0b00000000) {
        report("Alternate DMA address mode not yet implemented\r\n");
    } else if ((sdcard_dma_addr_2 >> 6) == 0b00000001) {
        report("Alternate DMA address mode not yet implemented\r\n");
    }
}

void sdcard_disk_write() {
    if ((sdcard_dma_addr_2 >> 6) == 0b00000010) {
        // create a single variable representation of the sector number
        sdcard_secnum = (sdcard_secnum_3 << 24) | (sdcard_secnum_2 << 16) | (sdcard_secnum_1 << 8) | (sdcard_secnum_0);
        // create a single variable representatio of the DMA address
        sdcard_dma_addr_local = (sdcard_dma_addr_1 << 8) | (sdcard_dma_addr_0);
        // make sure there is enough room in file to write
        if (sdcard_image_file.size() < ((sdcard_sector_size * sdcard_secnum) + (sdcard_sector_size * sdcard_sec_count))) {
            report("not enough space available in file\r\n");
            report("starting byte location %d\r\n",(sdcard_sector_size * sdcard_secnum));
            report("ending byte location %d\r\n",((sdcard_sector_size * sdcard_secnum) + (sdcard_sector_size * sdcard_sec_count)));
            report("file is %d bytes\r\n",sdcard_image_file.size());
            sdcard_status_byte = sdcard_status_byte | 0b10000000; 
            return;
        }
        // make sure there is enough RAM to hold all data being written (sector count * size)
        if (((sdcard_dma_addr_local-1) + (sdcard_sector_size * sdcard_sec_count)) > 0xffff) {
            report("not enough RAM available above DMA location\r\n");
            sdcard_status_byte = sdcard_status_byte | 0b10000000; 
            return;
        }
        // seek to the correct sector in the file
        if (!sdcard_image_file.seek(sdcard_sector_size * sdcard_secnum)) {
            report("error seeking starting byte %d\r\n",(sdcard_sector_size * sdcard_secnum));
            sdcard_status_byte = sdcard_status_byte | 0b10000000;  
            return;
        }
        
        // start DMA mode
        begin_dma();

        // stash current state
        bool old_ram_ce = ram_ce;

        // enable the SRAM
        ram_ce = true;
        shift_register_update();
        
        // loop for # of sectors required
        int j, i;
        for (j = sdcard_sec_count; j > 0; j--) {
            // copy 1 sector from SRAM
            for (i = 0; i < sdcard_sector_size; i++) {
                sdcard_sec_buf[i] = z80_memory_read(sdcard_dma_addr_local++);
            }
            // write sector to SD card
            if (!sdcard_image_file.write(sdcard_sec_buf, sdcard_sector_size)) {
                report("error writing sector to sdcard\r\n");
                sdcard_status_byte = sdcard_status_byte | 0b10000000; 
            }
        }
        // flush any remaining writes
        sdcard_image_file.flush();
        
        // restore machine state
        ram_ce = old_ram_ce;
        shift_register_update();
        
        //stop DMA mode
        end_dma();        
    } else if ((sdcard_dma_addr_2 >> 6) == 0b00000000) {
        report("Alternate DMA address mode not yet implemented\r\n");
    } else if ((sdcard_dma_addr_2 >> 6) == 0b00000001) {
        report("Alternate DMA address mode not yet implemented\r\n");
    }
}

void sdcard_reg_debug() {
    report("sdcard_secnum_0 = 0x%02x\r\n",sdcard_secnum_0);
    report("sdcard_secnum_1 = 0x%02x\r\n",sdcard_secnum_1);
    report("sdcard_secnum_2 = 0x%02x\r\n",sdcard_secnum_2);
    report("sdcard_secnum_3 = 0x%02x\r\n",sdcard_secnum_3);
    report("sdcard_dma_addr_0 = 0x%02x\r\n",sdcard_dma_addr_0);
    report("sdcard_dma_addr_1 = 0x%02x\r\n",sdcard_dma_addr_1);
    report("sdcard_dma_addr_2 = 0x%02x\r\n",sdcard_dma_addr_2);
    report("sdcard_sec_count = %03d\r\n",sdcard_sec_count);
    report("sdcard_selected_disk = %02d\r\n",(sdcard_status_byte & 0b00001111));
    report("sdcard_sector_size = %02d\r\n",sdcard_sector_size);
    report("sdcard_status_byte = 0x%02x\r\n",sdcard_status_byte);
}
