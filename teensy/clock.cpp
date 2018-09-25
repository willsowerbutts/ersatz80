#include <Arduino.h>
#include <assert.h>
#include "debug.h"
#include "z80.h"

clk_mode_t clk_mode, paused_clk_mode;
float clk_slow_requested; // requested frequency for slow clock
float clk_slow_freq;      // actual frequency of slow clock

bool z80_clk_running(void)
{
    return (clk_mode != CLK_STOP);
}

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
    assert(clk_mode == CLK_STOP);
    *portOutputRegister(CLK_STROBE) = 1;
    *portOutputRegister(CLK_FAST_ENABLE) = 1;
    clk_mode = CLK_FAST;
}

void z80_clk_fast_stop(void)
{
    assert(clk_mode == CLK_FAST);
    *portOutputRegister(CLK_FAST_ENABLE) = 0;
    *portOutputRegister(CLK_STROBE) = 0;
    clk_mode = CLK_STOP;
}

/* The slow clock is generated using the FTM in Edge-Aligned PWM (EPWM) mode.
 * This requires: FTMEN=0, QUADEN=0, DECAPEN=0, COMBINE=0, CPWMS=0, MSxB=1.
 * MOD must be at most 0xFFFE so that we can get to 100% duty cycle.
 * MOD must be at least 0x0001 (0x0002?) so that we can have 50% duty cycle.
 * V register writes will be buffered until the next timer overflow event.
 * note that our CLK_STROBE == CORE_FTM3_CH2_PIN
 */
float z80_clk_slow_start(float frequency)
{
    uint32_t clks, prescale, mod, basefreq;

    assert(clk_mode == CLK_STOP);
 
    if(frequency > (F_BUS/2)) // limit ourselves to the speeds we can actually generate!
        frequency = F_BUS/2;

    // for very low frequencies we use a much slower clock source
    if(frequency < ((F_BUS>>7)/65536.0f)){
        clks = 2;
        basefreq = 31250;
    }else{
        clks = 1;
        basefreq = F_BUS;
    }

    // determine best prescaler setting
    for(prescale=0; prescale<7; prescale++)
        if(frequency >= ((basefreq >> prescale) / 65536.0f))
            break;

    // compute best mod setting
    mod = ((basefreq >> prescale) / frequency) - 0.5f;
    mod |= 1; // must be odd to ensure we can reach an even 50% duty cycle
    if(mod > 0xFFFE) // must never exceed 0xFFFE
        mod = 0xFFFE;

    // stop the FTM first
    FTM3_SC = FTM_SC_CLKS(0) | FTM_SC_PS(0); // select no clock

    // configure the FTM
    FTM3_C2SC = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM3_MOD = mod;         // this dictates the frequency
    FTM3_C2V = 1 + (mod >> 1);    // 50% duty cycle please

    // reconfigure the output pin to connect it to the FTM
    CORE_PIN7_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;

    // select a running clock
    FTM3_SC = FTM_SC_CLKS(clks) | FTM_SC_PS(prescale); // select system clock, no prescaler

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
    clk_mode = CLK_SLOW;
    return freq;
}

void z80_clk_slow_stop(void)
{
    assert(clk_mode == CLK_SLOW);

    // set 0% duty in V reg
    FTM3_C2V = 0;

    // clear the TOF flag by reading SC and then writing 0 to the TOF bit
    FTM3_SC = FTM3_SC & (~FTM_SC_TOF);

    // wait for timer to overflow (TOF flag is set) so we know the V reg has been updated
    while(!(FTM3_SC & FTM_SC_TOF));

    // pre-set the GPIO to the desired output level
    *portOutputRegister(CLK_STROBE) = 0;

    // switch the mux from FTM back to GPIO
    CORE_PIN7_CONFIG = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    clk_mode = CLK_STOP;
}

void z80_clk_pause(void)
{
    paused_clk_mode = clk_mode;
    z80_clk_switch_stop();
}

void z80_clk_resume(void)
{
    switch(paused_clk_mode){
        case CLK_STOP:
            break;
        case CLK_SLOW:
            z80_clk_slow_start(clk_slow_requested);
            break;
        case CLK_FAST:
            z80_clk_fast_start();
            break;
    }
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
}

float z80_clk_switch_slow(float frequency)
{
    switch(clk_mode){
        case CLK_SLOW:
            if(clk_slow_requested == frequency)
                return clk_slow_freq;
            else
                z80_clk_slow_stop();
            break;
        case CLK_FAST:
            z80_clk_fast_stop();
            break;
        case CLK_STOP:
            break;
    }
    clk_slow_requested = frequency;
    clk_slow_freq = z80_clk_slow_start(frequency);
    return clk_slow_freq;
}

void z80_set_clk(bool level)
{
    assert(clk_mode == CLK_STOP);
    if(level){
        *portOutputRegister(CLK_STROBE) = 0; // Z80 CLK line goes high
        if(z80_bus_trace)
            z80_bus_report_state();
    }else
        *portOutputRegister(CLK_STROBE) = 1; // Z80 CLK line goes low
}

void z80_clock_pulse(void)
{
    z80_set_clk(false);
    z80_set_clk(true);
}
