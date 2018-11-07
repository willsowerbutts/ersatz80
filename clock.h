#ifndef __CLOCK_DOT_H__
#define __CLOCK_DOT_H__

#include <Arduino.h>

#define CLK_FAST_FREQUENCY (20000000.0f)
#define CLK_SLOW_MAX_FREQUENCY (15000000.0f)

void z80_clk_init(void);
void z80_clk_set_independent(float frequency);
void z80_clk_set_supervised(float frequency);
float z80_clk_get_frequency(void);
void z80_clk_set_supervised(bool supervised);
void z80_clk_pause(bool at_instruction_start=false);
void z80_clk_resume(void);
void z80_clock_pulse(void);
bool z80_clk_is_independent(void); // do we need to manually jiggle the clock line?
bool z80_clk_is_stopped(void);
bool z80_clk_is_supervised(void);
void z80_set_clk(bool level);
const char *z80_clk_get_name(void);

#endif
