#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include "serial.h"
#include "debug.h"

// stdio interface functions
static int serial_getchar(FILE *stream);
static int serial_putchar(char c, FILE *stream);
FILE serial_port_file = FDEV_SETUP_STREAM(serial_putchar, serial_getchar, _FDEV_SETUP_RW);

void debug_init(void)
{
    // connect stdio functions
    stdin = stdout = &serial_port_file;
}

#ifdef DEBUG
void debug_dumpmem(void *_ptr, uint16_t len)
{
    uint8_t *ptr = _ptr;
    report("Memory at 0x%x len 0x%x: [", ptr, len);
    for(int i=0; i<len; i++){
        if(i>0)
            report(", ");
        report("0x%02x", ptr[i]);
    }
    report("]\n");
}
#endif

static int serial_getchar(FILE *stream)
{
    return serial_read_byte();
}

static int serial_putchar(char c, FILE *stream)
{
    if(c == '\n')
        serial_write_byte('\r');
    serial_write_byte(c);

    return c;
}
