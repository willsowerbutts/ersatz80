#include <Arduino.h>
#include <stdbool.h>
#include "debug.h"
#include "z80.h"
#include "irq.h"

// our interrupts are level sensitive
// Z80 can clear the IRQ by writing to the status register.
uint8_t int_requests = 0x00;
uint8_t int_mask = 0x00;

void handle_z80_interrupts(void)
{
    if(z80_active_interrupts())
        z80_assert_interrupt();
}

uint8_t z80_active_interrupts(void)
{
    int_requests = (uart_interrupt_request()  ?  (1 << INT_BIT_UART0) : 0) |
                   (timer_interrupt_request() ?  (1 << INT_BIT_TIMER) : 0);
    return (int_requests & int_mask);
}

void z80_assert_interrupt(void)
{
    if(!z80_irq){
        report("[irq ON]");
        z80_irq = true;
        shift_register_update();
    }
}

void z80_clear_interrupt(void)
{
    if(z80_irq){
        z80_irq = false;
        shift_register_update();
        report("[irq OFF]");
    }
}

void int_requests_write(uint16_t address, uint8_t value)
{
    if(!z80_active_interrupts())
        z80_clear_interrupt();
}

uint8_t int_requests_read(uint16_t address)
{
    return int_requests;
}

void int_mask_write(uint16_t address, uint8_t value)
{
    int_mask = value;
}

uint8_t int_mask_read(uint16_t address)
{
    return int_mask;
}
