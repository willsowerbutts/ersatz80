#ifndef __DISK_DOT_H__
#define __DISK_DOT_H__

#include <Arduino.h>
#include "SdFat.h"
#include "z80.h"

#define NUM_DISK_DRIVES 4       // max 16
#define MAX_FILENAME_LENGTH 64  // really this is a file path ie directory plus filename

void    disk_init(void); 
void    disk_sync(void);
bool    disk_is_file_mounted(const char *filename);
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
bool    disk_format(const char *filename, uint32_t bytes);
bool    disk_rm(const char *victim);
bool    disk_cp(const char *source, const char *dest);
bool    disk_mv(const char *source, const char *dest);

// WRS: can also try using SdFatSdio here -- it may be slower? have not benchmarked.
extern SdFatSdioEX sdcard;

typedef struct {
    SdBaseFile file;
    uint32_t   sector_number;
    uint32_t   dma_address;
    uint32_t   size_bytes;
    uint8_t    sector_size_log;  // stored as power of 2; 7=128, 8=256, 9=512, 10=1024.
    uint8_t    sector_count;
    bool       mounted;
    bool       error;
    bool       writable;
} disk_info_t;

extern disk_info_t disk[NUM_DISK_DRIVES];

#endif
