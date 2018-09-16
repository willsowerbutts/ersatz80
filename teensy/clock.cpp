#include <Arduino.h>
#include "debug.h"
#include "z80.h"

typedef enum { CLK_FAST, CLK_SLOW, CLK_STOP } clk_mode_t;
clk_mode_t clk_mode;
float clk_slow_freq;

void z80_clk_init(void)
{
    pinMode(CLK_STROBE, OUTPUT);
    pinMode(CLK_FAST_ENABLE, OUTPUT);
    digitalWrite(CLK_STROBE, 0);
    digitalWrite(CLK_FAST_ENABLE, 0);
    clk_mode = CLK_STOP;
}

void z80_clk_fast_start(void)
{
    digitalWrite(CLK_STROBE, 1);
    digitalWrite(CLK_FAST_ENABLE, 1);
}

void z80_clk_fast_stop(void)
{
    digitalWrite(CLK_FAST_ENABLE, 0);
    digitalWrite(CLK_STROBE, 0);
}

float z80_clk_slow_start(float frequency)
{
    // TODO - ensure this starts the clock cleanly
    if(frequency > (F_BUS/4)) // limit to speeds we can reliably generate
        frequency = F_BUS/4;
    analogWriteFrequency(CLK_STROBE, frequency);
    analogWrite(CLK_STROBE, 128);
    // note CLK_STROBE == CORE_FTM3_CH2_PIN
    // compute the actual frequency configured:
    float freq;
    switch(FTM3_SC & FTM_SC_CLKS_MASK){ // clock source
        case FTM_SC_CLKS(0):
            freq = 0;
            break;
        case FTM_SC_CLKS(1):
            freq = F_BUS;
            break;
        case FTM_SC_CLKS(2):
            freq = 31250;
            break;
        default:
            report("(unknown FTM CS?)");
    }
    freq = freq / (1 << (FTM3_SC & FTM_SC_PS_MASK)); // prescaler
    freq = freq / (1 + FTM3_MOD);                    // modulo
    return freq;
}

void z80_clk_slow_stop(void)
{
    // TODO - ensure this stops the clock cleanly
    pinMode(CLK_STROBE, OUTPUT);
    digitalWrite(CLK_STROBE, 0);
}

void z80_clk_switch_stop(void)
{
    switch(clk_mode){
        case CLK_STOP:
            break;
        case CLK_SLOW:
            z80_clk_slow_stop();
            break;
        case CLK_FAST:
            z80_clk_fast_stop();
            break;
    }
    clk_mode = CLK_STOP;
}

void z80_clk_switch_fast(void)
{
    switch(clk_mode){
        case CLK_FAST:
            break;
        case CLK_SLOW:
            z80_clk_slow_stop();
            // fall through
        case CLK_STOP:
            z80_clk_fast_start();
            break;
    }
    clk_mode = CLK_FAST;
}

float z80_clk_switch_slow(float frequency)
{
    switch(clk_mode){
        case CLK_SLOW:
            if(clk_slow_freq == frequency)
                return;
            break;
        case CLK_FAST:
            z80_clk_fast_stop();
            break;
        case CLK_STOP:
            break;
    }
    clk_mode = CLK_SLOW;
    clk_slow_freq = frequency;
    return z80_clk_slow_start(frequency);
}

void z80_set_clk(bool level)
{
    if(level){
        digitalWrite(CLK_STROBE, 0); // CLK high
        if(z80_bus_trace)
            z80_bus_report_state();
    }else
        digitalWrite(CLK_STROBE, 1); // CLK low
}

void z80_clock_pulse(void)
{
    z80_set_clk(false);
    z80_set_clk(true);
}
