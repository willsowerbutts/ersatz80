#ifndef __ROM_DOT_H__
#define __ROM_DOT_H__

#define ROM_ADDR_MASK     0x3FFF  // 16KB
#define MONITOR_ROM_START 0xF800
#define MONITOR_ROM_SIZE  0x0800
typedef unsigned char uint8_t;
extern const uint8_t monitor_rom[];
extern const uint8_t basic_rom[];

#endif
