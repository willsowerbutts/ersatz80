#include <Arduino.h>
#include <assert.h>
#include "debug.h"
#include "clock.h"
#include "z80.h"

#ifdef ERSATZ80_PCB_REV1
// On rev1 PCBs FTM3 channel 2 drives CLK_STROBE
#define FTM_CLK_CSC FTM3_C2SC
#define FTM_CLK_CNT FTM3_CNT
#define FTM_CLK_MOD FTM3_MOD
#define FTM_CLK_CV  FTM3_C2V
#define FTM_CLK_SC  FTM3_SC
#define FTM_PORT_PCR_MUX 4
#else
// On rev2 PCBs FTM2 channel 0 drives CLK_STROBE
#define FTM_CLK_CSC FTM2_C0SC
#define FTM_CLK_CNT FTM2_CNT
#define FTM_CLK_MOD FTM2_MOD
#define FTM_CLK_CV  FTM2_C0V
#define FTM_CLK_SC  FTM2_SC
#define FTM_PORT_PCR_MUX 3
#endif

typedef enum {
    CLK_UNSUPERVISED_FAST,
    CLK_UNSUPERVISED_FTM,
    CLK_SUPERVISED,
} clk_mode_t;

float clk_ftm_frequency;
float clk_freq_supervised;
float clk_freq_unsupervised;
clk_mode_t clk_mode;

static void clk_ftm_wait_event(void)
{
    FTM_CLK_CSC = FTM_CLK_CSC & (~FTM_CSC_CHF); // clear the CHF bit
    while(!(FTM_CLK_CSC & FTM_CSC_CHF));        // wait for CHG bit to be set
}

static void clk_ftm_wait_overflow(void)
{
    FTM_CLK_SC = FTM_CLK_SC & (~FTM_SC_TOF);    // clear the TOF flag by reading SC and then writing 0 to the TOF bit
    while(!(FTM_CLK_SC & FTM_SC_TOF));          // wait for timer to overflow (TOF flag is set)
}

void z80_set_clk(bool level)
{
    if(level){
        *portOutputRegister(CLK_STROBE) = 0; // Z80 CLK line goes high
        if(clk_is_supervised()){
            clk_ftm_wait_overflow();
            z80_bus_trace_state();
        }
    }else{
        *portOutputRegister(CLK_STROBE) = 1; // Z80 CLK line goes low
        if(clk_is_supervised()){
            clk_ftm_wait_event();
        }
    }
}

void z80_clock_pulse(void)
{
    z80_set_clk(false);
    z80_set_clk(true);
}

static void clk_fast_start(void)
{
    *portOutputRegister(CLK_STROBE) = 1;
    *portOutputRegister(CLK_FAST_ENABLE) = 1;
}

static void clk_fast_stop(void)
{
    *portOutputRegister(CLK_FAST_ENABLE) = 0;
    *portOutputRegister(CLK_STROBE) = 0;
}

/* The variable clock is generated using the FTM in Edge-Aligned PWM (EPWM) mode.
 * This requires: FTMEN=0, QUADEN=0, DECAPEN=0, COMBINE=0, CPWMS=0, MSxB=1.
 * MOD must be at most 0xFFFE so that we can get to 100% duty cycle.
 * MOD must be odd so that we have a 50% duty cycle.
 * V register writes will be hardware buffered until the next timer overflow event.
 */
static float clk_ftm_prepare(float frequency, bool apply)
{
    uint32_t clks, prescale, mod, basefreq;

    if(frequency >= CLK_FTM_MAX_FREQUENCY) // limit ourselves to the speeds we can actually generate!
        frequency = CLK_FTM_MAX_FREQUENCY;

    if(frequency < 0.1) // set a reasonable lower limit
        frequency = 0.1;

    // for very low frequencies we need to use a slower clock source
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

    if(apply){
        // configure the FTM hardware
        FTM_CLK_SC  = FTM_SC_CLKS(0) | FTM_SC_PS(0); // select no clock
        FTM_CLK_CSC = FTM_CSC_MSB | FTM_CSC_ELSB;
        FTM_CLK_CNT = 0;
        FTM_CLK_MOD = mod;
        FTM_CLK_CV  = 1 + (mod >> 1); // always 50% duty cycle
        FTM_CLK_SC  = FTM_SC_CLKS(clks) | FTM_SC_PS(prescale); // select system clock, no prescaler
    }

    // compute the actual frequency being generated:
    return (float)(basefreq >> prescale) / (float)(1 + mod);
}

static float clk_ftm_get_closest_frequency(float frequency)
{
    return clk_ftm_prepare(frequency, false);
}

static float clk_ftm_set_frequency(float frequency)
{
    return clk_ftm_prepare(frequency, true);
}

static void clk_ftm_start(float frequency)
{
    *portConfigRegister(CLK_STROBE) = PORT_PCR_MUX(FTM_PORT_PCR_MUX) | PORT_PCR_DSE | PORT_PCR_SRE; // connect output pin to the FTM
    clk_ftm_set_frequency(frequency);    // start the FTM
}

static void clk_ftm_stop(void)
{
    FTM_CLK_CV = 0;                             // set 0% duty in V reg
    clk_ftm_wait_overflow();                    // wait for timer to overflow so we know the V reg has been updated
    *portOutputRegister(CLK_STROBE) = 0;        // pre-set the GPIO to the desired output level
    *portConfigRegister(CLK_STROBE) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1); // switch the mux from FTM back to GPIO
}

void clk_setup(void)
{
    pinMode(CLK_FAST_ENABLE, OUTPUT);
    pinMode(CLK_STROBE, OUTPUT);
    *portOutputRegister(CLK_FAST_ENABLE) = 0;
    *portOutputRegister(CLK_STROBE) = 0;
    z80_set_clk(true);
    clk_freq_supervised = CLK_FTM_MAX_FREQUENCY;
    clk_freq_unsupervised = CLK_FAST_FREQUENCY;
    clk_mode = CLK_UNSUPERVISED_FAST;
    clk_set_supervised(true);
}

bool clk_is_supervised(void)
{
    return (clk_mode == CLK_SUPERVISED);
}

void clk_set_supervised(bool supervised)
{
    if(clk_is_supervised() == supervised)
        return;

    switch(clk_mode){
        case CLK_UNSUPERVISED_FAST:
            clk_fast_stop();
            break;
        case CLK_UNSUPERVISED_FTM:
            clk_ftm_stop();
            break;
        case CLK_SUPERVISED:
            break;
    }
    if(supervised){
        clk_ftm_set_frequency(clk_freq_supervised);
        clk_mode = CLK_SUPERVISED;
    }else{
        if(clk_freq_unsupervised > CLK_FTM_MAX_FREQUENCY){
            clk_fast_start();
            clk_mode = CLK_UNSUPERVISED_FAST;
        }else{
            clk_ftm_start(clk_freq_unsupervised);
            clk_mode = CLK_UNSUPERVISED_FTM;
        }
    }
}

void clk_set_supervised_frequency(float hz)
{
    if(hz < 0.1)
        hz = 0.11;
    if(hz > CLK_FTM_MAX_FREQUENCY)
        hz = CLK_FTM_MAX_FREQUENCY;

    clk_freq_supervised = clk_ftm_get_closest_frequency(hz);

    if(clk_is_supervised())
        clk_set_supervised(true);
}

void clk_set_unsupervised_frequency(float hz)
{
    if(hz < 0.1)
        hz = 0.11;

    if(hz > CLK_FTM_MAX_FREQUENCY)
        clk_freq_unsupervised = CLK_FAST_FREQUENCY;
    else
        clk_freq_unsupervised = clk_ftm_get_closest_frequency(hz);

    if(!clk_is_supervised())
        clk_set_supervised(false);
}

float clk_get_supervised_frequency(void)
{
    return clk_freq_supervised;
}

float clk_get_unsupervised_frequency(void)
{
    return clk_freq_unsupervised;
}
