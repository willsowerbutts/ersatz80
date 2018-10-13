#ifndef __TIMER_DOT_H__
#define __TIMER_DOT_H__

void handle_timer(unsigned long now);
uint8_t timer_read_status(uint16_t address);
void timer_write_control(uint16_t address, uint8_t value);
bool timer_interrupt_request(void);

#endif
