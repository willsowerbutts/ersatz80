#include <Arduino.h>
#include "serial.h"

int serial_read_line(unsigned char *buffer, int buffer_length)
{
    int c, o;

    o = 0;
    while(1){
        c = Serial.read();
        if(c >= 0){
            if(c == 0x7f || c == 0x08){ // backspace and delete
                if(o > 0){
                    Serial.write("\x08 \x08"); // erase last char
                    o--;
                }
            }else if(c == 0x0d || c == 0x0a){
                buffer[o] = 0;
                return o;
            }else if(c >= 0x20){
                if(o < (buffer_length-1)){
                    buffer[o++] = c;
                    Serial.write(c);
                }
            }
        }
    }
}
