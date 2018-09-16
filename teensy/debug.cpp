#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "serial.h"
#include "debug.h"

#ifdef DEBUG
void debug_dumpmem(void *_ptr, uint16_t len)
{
    uint8_t *ptr = (uint8_t*)_ptr;
    report("Memory at 0x%x len 0x%x: [", ptr, len);
    for(int i=0; i<len; i++){
        if(i>0)
            report(", ");
        report("0x%02x", ptr[i]);
    }
    report("]\n");
}
#endif

uint8_t debug_boldlevel = 0;

void debug_boldon(void)
{
    if(debug_boldlevel == 0)
        Serial.write("\x1b[1m");
    debug_boldlevel++;
}

void debug_boldoff(void)
{
    if(debug_boldlevel == 0)
        Serial.write("debug_boldoff with debug_boldlevel=0");
    debug_boldlevel--;
    if(debug_boldlevel == 0)
        Serial.write("\x1b[0m");
}
