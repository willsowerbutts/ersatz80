#ifndef __SDCARD_DOT_H__
#define __SDCARD_DOT_H__

#include <Arduino.h>
#include <SD.h>

const uint8_t sdcard_base_port = 0x90;

extern uint8_t sdcard_status_byte;

void sdcard_init(void); 
void sdcard_reg_wr(uint8_t address, uint8_t value);
uint8_t sdcard_reg_rd(uint8_t address);
void sdcard_cmd_wr(uint8_t value);
void sdcard_select_prenamed_disk(uint8_t disknum);
void sdcard_disk_read(void);
void sdcard_disk_write(void);
void sdcard_reg_debug(void);

#endif
