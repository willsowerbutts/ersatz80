#ifndef __Z80IO_DOT_H__
#define __Z80IO_DOT_H__

uint8_t iodevice_read(uint16_t address);
void iodevice_write(uint16_t address, uint8_t value); // call ONLY when in DMA mode!

uint8_t memory_read(uint16_t address);
void memory_write(uint16_t address, uint8_t value);

bool uart_rx_fifo_push(uint8_t keyin);
bool uart_interrupt_request(void);

#endif
