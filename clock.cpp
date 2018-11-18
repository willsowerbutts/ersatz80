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

void z80_slow_clock_set_frequency(float frequency);

typedef enum { CLK_FAST, CLK_SLOW, CLK_SUPERVISED, CLK_STOPPED } clk_mode_t;
clk_mode_t clk_mode, paused_clk_mode;
float clk_slow_requested; // requested frequency for slow clock
float clk_slow_freq;      // actual frequency of slow clock

void z80_clk_fast_start(void)
{
    assert(clk_mode == CLK_STOPPED);
    *portOutputRegister(CLK_STROBE) = 1;
    *portOutputRegister(CLK_FAST_ENABLE) = 1;
    clk_mode = CLK_FAST;
}

void z80_clk_fast_stop(void)
{
    assert(clk_mode == CLK_FAST);
    *portOutputRegister(CLK_FAST_ENABLE) = 0;
    *portOutputRegister(CLK_STROBE) = 0;
    clk_mode = CLK_STOPPED;
}

/* The slow clock is generated using the FTM in Edge-Aligned PWM (EPWM) mode.
 * This requires: FTMEN=0, QUADEN=0, DECAPEN=0, COMBINE=0, CPWMS=0, MSxB=1.
 * MOD must be at most 0xFFFE so that we can get to 100% duty cycle.
 * MOD must be at least 0x0001 (0x0002?) so that we can have 50% duty cycle.
 * V register writes will be buffered until the next timer overflow event.
 * note that our CLK_STROBE == CORE_FTM3_CH2_PIN
 */
void z80_clk_slow_start(float frequency)
{
    assert(clk_mode == CLK_STOPPED || clk_mode == CLK_SUPERVISED);
    // reconfigure the output pin to connect it to the FTM
    *portConfigRegister(CLK_STROBE) = PORT_PCR_MUX(FTM_PORT_PCR_MUX) | PORT_PCR_DSE | PORT_PCR_SRE;
    z80_slow_clock_set_frequency(frequency);
    // select a running clock
    clk_mode = CLK_SLOW;
}

void z80_clk_slow_wait_event(void)
{
    FTM_CLK_CSC = FTM_CLK_CSC & (~FTM_CSC_CHF); // clear the CHF bit
    while(!(FTM_CLK_CSC & FTM_CSC_CHF));        // wait for CHG bit to be set
}

void z80_clk_slow_wait_overflow(void)
{
    FTM_CLK_SC = FTM_CLK_SC & (~FTM_SC_TOF);    // clear the TOF flag by reading SC and then writing 0 to the TOF bit
    while(!(FTM_CLK_SC & FTM_SC_TOF));          // wait for timer to overflow (TOF flag is set)
}

void z80_clk_slow_stop(void)
{
    assert(clk_mode == CLK_SLOW);

    // set 0% duty in V reg
    FTM_CLK_CV = 0;

    // wait for timer to overflow so we know the V reg has been updated
    z80_clk_slow_wait_overflow();

    // pre-set the GPIO to the desired output level
    *portOutputRegister(CLK_STROBE) = 0;

    // switch the mux from FTM back to GPIO
    *portConfigRegister(CLK_STROBE) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    clk_mode = CLK_STOPPED;
}

void z80_slow_clock_set_frequency(float frequency)
{
    uint32_t clks, prescale, mod, basefreq;

    assert(clk_mode == CLK_STOPPED || clk_mode == CLK_SUPERVISED);

    if(frequency >= CLK_SLOW_MAX_FREQUENCY) // limit ourselves to the speeds we can actually generate!
        frequency = CLK_SLOW_MAX_FREQUENCY;

    if(frequency == 0.0f)
        return;

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

    // configure the FTM hardware
    FTM_CLK_SC  = FTM_SC_CLKS(0) | FTM_SC_PS(0); // select no clock
    FTM_CLK_CSC = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM_CLK_CNT = 0;
    FTM_CLK_MOD = mod;
    FTM_CLK_CV  = 1 + (mod >> 1); // always 50% duty cycle
    FTM_CLK_SC  = FTM_SC_CLKS(clks) | FTM_SC_PS(prescale); // select system clock, no prescaler

    // compute the frequency actually configured:
    clk_slow_freq = (float)(basefreq >> prescale) / (float)(1 + mod);
    clk_slow_requested = frequency;
}

void z80_clk_init(void)
{
    pinMode(CLK_STROBE, OUTPUT);
    pinMode(CLK_FAST_ENABLE, OUTPUT);
    *portOutputRegister(CLK_STROBE) = 0;
    *portOutputRegister(CLK_FAST_ENABLE) = 0;
    clk_mode = CLK_STOPPED;
    clk_slow_freq = clk_slow_requested = 0.0f;
    z80_slow_clock_set_frequency(1000000); // 1MHz
}

const char *z80_clk_get_name(void)
{
    switch(clk_mode){
        case CLK_FAST:       return "external crystal";
        case CLK_SLOW:       return "internal timer";
        case CLK_SUPERVISED: return "software supervised";
        case CLK_STOPPED:    return "stopped";
        default: return "UNKNOWN";
    }
}

float z80_clk_get_frequency(void)
{
    switch(clk_mode){
        case CLK_FAST:
            return CLK_FAST_FREQUENCY;
        case CLK_SLOW:
        case CLK_SUPERVISED:
            return clk_slow_freq;
        case CLK_STOPPED:
            return 0.0f;
    }
    assert(false);
}

bool z80_clk_is_independent(void)
{
    switch(clk_mode){
        case CLK_FAST:
        case CLK_SLOW:
            return true;
        case CLK_SUPERVISED:
        case CLK_STOPPED:
            return false;
    }
    assert(false);
}

bool z80_clk_is_stopped(void)
{
    return (clk_mode == CLK_STOPPED);
}

bool z80_clk_is_supervised(void)
{
    return (clk_mode == CLK_SUPERVISED);
}

void z80_clk_stop(void)
{
    switch(clk_mode){
        case CLK_SUPERVISED:
            clk_mode = CLK_STOPPED;
            break;
        case CLK_STOPPED:
            break;
        case CLK_FAST:
            z80_clk_fast_stop();
            break;
        case CLK_SLOW:
            z80_clk_slow_stop();
            break;
    }
    assert(clk_mode == CLK_STOPPED);
}

void z80_clk_set_independent(float frequency)
{
    // is this a nop?
    if(frequency <= 0 && clk_mode == CLK_STOPPED)
        return;
    if(frequency == clk_slow_requested && clk_mode == CLK_SLOW)
        return;
    if(frequency > CLK_SLOW_MAX_FREQUENCY && clk_mode == CLK_FAST)
        return;

    z80_clk_stop();

    if(frequency <= 0)
        return;
    else if(frequency > CLK_SLOW_MAX_FREQUENCY)
        z80_clk_fast_start();
    else
        z80_clk_slow_start(frequency);
}

void z80_clk_set_supervised(float frequency)
{
    if(frequency == clk_slow_requested && clk_mode == CLK_SUPERVISED)
        return;

    clk_mode_t old_mode = clk_mode;
    z80_clk_stop();
    z80_slow_clock_set_frequency(frequency);
    if(old_mode != CLK_STOPPED)
        clk_mode = CLK_SUPERVISED;
}

void z80_clk_pause(bool at_instruction_start)
{
    paused_clk_mode = clk_mode;
    z80_clk_stop();

    // stop the CPU only at the start of an instruction fetch.
    // NOTE/TODO: this may stop the CPU partway through a multi
    // byte instruction, eg in 'ED 51' we may stop after the ED
    // is fetched, but before the 51.
    if(at_instruction_start){
        while(!z80_m1_asserted() || z80_mreq_asserted()){
            if(!z80_clk_is_independent())
                z80_clock_pulse();
            handle_z80_bus();
        }
    }
}

void z80_clk_resume(void)
{
    switch(paused_clk_mode){
        case CLK_SUPERVISED:
            clk_mode = CLK_SUPERVISED;
        case CLK_STOPPED:
            break;
        case CLK_SLOW:
            z80_clk_slow_start(clk_slow_requested);
            break;
        case CLK_FAST:
            z80_clk_fast_start();
            break;
    }
}

void z80_set_clk(bool level)
{
    assert(clk_mode == CLK_STOPPED || clk_mode == CLK_SUPERVISED);
    if(level){
        *portOutputRegister(CLK_STROBE) = 0; // Z80 CLK line goes high
        if(clk_mode == CLK_SUPERVISED)
            z80_clk_slow_wait_overflow();
        if(z80_bus_trace != TR_OFF)
            z80_bus_trace_state();
    }else{
        *portOutputRegister(CLK_STROBE) = 1; // Z80 CLK line goes low
        if(clk_mode == CLK_SUPERVISED)
            z80_clk_slow_wait_event();
    }
}

void z80_clock_pulse(void)
{
    z80_set_clk(false);
    z80_set_clk(true);
}
