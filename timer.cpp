#include <Arduino.h>
#include <stdbool.h>
#include "debug.h"
#include "z80.h"
#include "timer.h"
#include "irq.h"

bool timer_tick_pending = false;
unsigned long timer_interval = 1000;
unsigned long next_timer_tick = 0;

bool timer_interrupt_request(void)
{
    return timer_tick_pending;
}

void handle_timer(unsigned long now)
{
    if(now >= next_timer_tick)
        timer_tick_pending = true;
}

uint8_t timer_read_status(uint16_t address)
{
    return timer_tick_pending ? 1 : 0;
}

void timer_write_control(uint16_t address, uint8_t value)
{
    unsigned long now;

    if(value){ // only non-zero writes will set the frequency
        timer_interval = 1000 / value;
        report("timer: interval %dms\r\n", timer_interval);
    }

    // writes always clear any pending timer interrupt
    if(timer_tick_pending)
        timer_tick_pending = false;

    // writes always calculate when the next tick is due
    now = millis();
    next_timer_tick = now - (now % timer_interval) + timer_interval;
}
