#include <avr/io.h>
#include "serial.h"

#define TARGET_BAUD 115200
#define TARGET_UBRR0 ((F_CPU / (8 * TARGET_BAUD)) - 1)

void serial_init(void)
{
    // serial init: baud rate
    UBRR0H = (TARGET_UBRR0 >> 8);       // baud rate generator high byte
    UBRR0L = (TARGET_UBRR0 & 0xFF);     // baud rate generator low byte
    UCSR0A = _BV(U2X0);                 // double USART speed
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // enable receiver and transmitter
    UCSR0C = _BV(UCSZ00) | _BV(UCSZ01); // 8N1 framing
}

int serial_read_byte(void)
{
    if(UCSR0A & _BV(RXC0))
        return UDR0;
    return -1;
}

int serial_read_line(unsigned char *buffer, int buffer_length)
{
    int c, o;

    o = 0;

    while(1){
        c = serial_read_byte();
        if(c >= 0){
            if(c == 0x7f || c == 0x08){ // backspace and delete
                if(o > 0){
                    serial_write("\x08 \x08"); // erase last char
                    o--;
                }
            }else if(c == 0x0d || c == 0x0a){
                buffer[o] = 0;
                return o;
            }else if(c >= 0x20){
                if(o < (buffer_length-1)){
                    buffer[o++] = c;
                    serial_write_byte(c);
                }
            }
        }
    }
}

void serial_write_byte(unsigned char byte)
{
    while (!(UCSR0A & _BV(UDRE0))); // wait for any previous transmission to complete
    UDR0 = byte;
}

void serial_write(char *string)
{
    char *p = string;
    while(*p)
        serial_write_byte(*p++);
}
