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

bool debug_flag_user = true;

void debug_mode_super(void){
    if(!debug_flag_user)
        return;
    debug_flag_user = false;
    Serial.write("\x1b[1m");
}

void debug_mode_user(void)
{
    if(debug_flag_user)
        return;
    debug_flag_user = true;
    Serial.write("\x1b[0m");
}
