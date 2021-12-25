#ifndef __DEBUG_DOT_H__
#define __DEBUG_DOT_H__

#include <stdio.h>

void debug_mode_super(void);
void debug_mode_user(void);

#define report(msg, args...) do { debug_mode_super(); Serial.printf( msg, ## args); } while(0)
void debug_dumpmem(void *ptr, uint16_t len);
#define dumpmem(ptr, len) do { debug_dumpmem(ptr, len); } while(0)

#endif
