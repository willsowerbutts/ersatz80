#ifndef __SDCARD_DOT_H__
#define __SDCARD_DOT_H__

#include <Arduino.h>
#include "SdFat.h"
#include "z80.h"

#define NUM_DISK_DRIVES 4 // max 16

void    sdcard_init(void); 
uint8_t disk_sector_read(uint16_t address);
void    disk_sector_write(uint16_t address, uint8_t value);
uint8_t disk_address_read(uint16_t address);
void    disk_address_write(uint16_t address, uint8_t value);
uint8_t disk_seccount_read(uint16_t address);
void    disk_seccount_write(uint16_t address, uint8_t value);
uint8_t disk_status_read(uint16_t address);
void    disk_command_write(uint16_t address, uint8_t value);
uint8_t disk_reserved_read(uint16_t address);
void    disk_reserved_write(uint16_t address, uint8_t value);

// WRS: can also try using SdFatSdio here -- it may be slower? have not benchmarked.
extern SdFatSdioEX sdcard;

typedef struct {
    SdBaseFile file;
    uint32_t   sector_number;
    uint32_t   dma_address;
    uint8_t    sector_size_log;  // stored as power of 2; 7=128, 8=256, 9=512, 10=1024.
    uint8_t    sector_count;
    bool       mounted;
    bool       error;
    bool       writable;
} disk_info_t;

extern disk_info_t disk[NUM_DISK_DRIVES];

#endif
