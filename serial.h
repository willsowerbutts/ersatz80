#ifndef __SERIAL_DOT_H__
#define __SERIAL_DOT_H__

void serial_init(void);
void serial_write_byte(unsigned char byte);
void serial_write(char *string);
int serial_read_line(unsigned char *buffer, int buffer_length); // read until newline (discards newline)
int serial_read_byte(void);

#endif
