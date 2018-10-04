#ifndef __SDCARD_DOT_H__
#define __SDCARD_DOT_H__

#include <Arduino.h>
#include "SdFat.h"
#include "z80.h"

void    sdcard_init(void); 
uint8_t disk_sector_read(uint16_t address);
void    disk_sector_write(uint16_t address, uint8_t value);
uint8_t disk_address_read(uint16_t address);
void    disk_address_write(uint16_t address, uint8_t value);
uint8_t disk_seccount_read(uint16_t address);
void    disk_seccount_write(uint16_t address, uint8_t value);
uint8_t disk_status_read(uint16_t address);
void    disk_command_write(uint16_t address, uint8_t value);

#endif
