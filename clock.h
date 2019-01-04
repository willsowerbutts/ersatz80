#ifndef __CLOCK_DOT_H__
#define __CLOCK_DOT_H__

#include <Arduino.h>

#define CLK_FAST_FREQUENCY (20000000.0f)     // frequency of external crystal
#define CLK_FTM_MAX_FREQUENCY (F_BUS/4)     // restricted by FTM hardware

void z80_clock_pulse(void);
void z80_set_clk(bool level);

void clk_setup(void);
void clk_set_supervised(bool supervised);
bool clk_is_supervised(void); // generally should use z80_supervised_mode() instead
void clk_set_supervised_frequency(float hz);
void clk_set_unsupervised_frequency(float hz);
float clk_get_supervised_frequency(void);
float clk_get_unsupervised_frequency(void);

#endif
