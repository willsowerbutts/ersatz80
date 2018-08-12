#ifndef __DEBUG_DOT_H__
#define __DEBUG_DOT_H__

#include <stdio.h>
#include <avr/pgmspace.h>

void debug_boldon(void);
void debug_boldoff(void);
void debug_init(void);
extern FILE serial_port_file;

// report() is basically printf() to the serial port

#ifdef DEBUG
#define report(msg, args...) do { debug_boldon(); printf_P( PSTR(msg), ## args); debug_boldoff(); } while(0)
void debug_dumpmem(void *ptr, uint16_t len);
#define dumpmem(ptr, len) do { debug_dumpmem(ptr, len); } while(0)
#else
#define report(msg, args...) do { } while(0)
#define dumpmem(ptr, len) do { } while(0)
#endif

#endif
