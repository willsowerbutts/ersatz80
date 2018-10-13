#ifndef __IRQ_DOT_H__
#define __IRQ_DOT_H__

#define INT_BIT_TIMER   0
#define INT_BIT_UART0   1

uint8_t z80_active_interrupts(void);
uint8_t z80_irq_vector(void);
void handle_z80_interrupts(void);
void z80_assert_interrupt(void);
void z80_clear_interrupt(void);

void int_requests_write(uint16_t address, uint8_t value);
uint8_t int_requests_read(uint16_t address);
void int_mask_write(uint16_t address, uint8_t value);
uint8_t int_mask_read(uint16_t address);

bool uart_interrupt_request(void);

#endif
