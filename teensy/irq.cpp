#include <Arduino.h>
#include <stdbool.h>
#include "debug.h"
#include "z80.h"
#include "irq.h"

// our interrupts are level sensitive
// Z80 can clear the IRQ by writing to the status register.
uint8_t int_requests = 0x00;
uint8_t int_mask = 0x00;
uint8_t int_active = 0x00;
uint8_t timer_vector = 0x00; // bit 0 must be 0
uint8_t uart0_vector = 0x02; // bit 0 must be 0

void handle_z80_interrupts(void)
{
    int_active = z80_active_interrupts();
    if(int_active)
        z80_assert_interrupt();
}

uint8_t z80_active_interrupts(void)
{
    int_requests = (uart_interrupt_request()  ?  (1 << INT_BIT_UART0) : 0) |
                   (timer_interrupt_request() ?  (1 << INT_BIT_TIMER) : 0);
    return (int_requests & int_mask);
}

uint8_t z80_irq_vector(void)
{
    // note that bit 0 of whatever we return should be 0
    if(int_active & (1 << INT_BIT_TIMER))
        return timer_vector;
    if(int_active & (1 << INT_BIT_UART0))
        return timer_vector;
    // if we get here the likely reason is that the Z80 code has
    // handled the requesting device already, but has not cleared
    // the IRQ line by writing to the status register before
    // re-enabling interrupts.
    report("irq_vector: no active irq?\r\n");
    return 0x00;
}

void z80_assert_interrupt(void)
{
    if(!z80_irq){
        // report("[irq ON]");
        z80_irq = true;
        shift_register_update();
    }
}

void z80_clear_interrupt(void)
{
    if(z80_irq){
        z80_irq = false;
        shift_register_update();
        // report("[irq OFF]");
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
