#ifndef __SERIAL_DOT_H__
#define __SERIAL_DOT_H__

#include <stdbool.h>

int serial_read_line(unsigned char *buffer, int buffer_length); // read until newline (discards newline)

#endif
