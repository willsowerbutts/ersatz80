#include <Arduino.h>
#include "z80.h"
#include "interface.h"
#include "debug.h"

static uint16_t user_leds = 0x000; // 12-bit value to show on user-controlled LEDs
static bool z80_reset = true;      // Z80 /RESET pin (z80_reset=true means /RESET is driven low ie asserted)
static bool z80_irq = false;       // Z80 /IRQ pin
static bool z80_nmi = false;       // Z80 /NMI pin
static bool ram_ce = false;        // RAM /CE pin

static void shift_register_update(void)
{
    unsigned int bit = 0x8000;
    unsigned int output =
        (user_leds << 4)    |
        (ram_ce    ? 0 : 8) |
        (z80_irq   ? 0 : 4) |
        (z80_nmi   ? 0 : 2) |
        (z80_reset ? 0 : 1);
#ifdef KINETISK
    do{
        *portOutputRegister(SHIFT_REGISTER_DATA) = (output & bit) ? 1 : 0;
        *portOutputRegister(SHIFT_REGISTER_CLK) = 1;
        bit >>= 1;
        *portOutputRegister(SHIFT_REGISTER_CLK) = 0;
    }while(bit);
    *portOutputRegister(SHIFT_REGISTER_LATCH) = 1;
    *portOutputRegister(SHIFT_REGISTER_LATCH) = 0;
#else
    do{
        digitalWrite(SHIFT_REGISTER_DATA, (output & bit) ? 1 : 0);
        digitalWrite(SHIFT_REGISTER_CLK, 1);
        bit >>= 1;
        digitalWrite(SHIFT_REGISTER_CLK, 0);
    }while(bit);
    digitalWrite(SHIFT_REGISTER_LATCH, 1);
    digitalWrite(SHIFT_REGISTER_LATCH, 0);
#endif
}

static void shift_register_setup(void)
{
    // shift register setup
    pinMode(SHIFT_REGISTER_CLK, OUTPUT);
    pinMode(SHIFT_REGISTER_LATCH, OUTPUT);
    pinMode(SHIFT_REGISTER_DATA, OUTPUT);
    digitalWrite(SHIFT_REGISTER_CLK, 0);
    digitalWrite(SHIFT_REGISTER_LATCH, 0);
    digitalWrite(SHIFT_REGISTER_DATA, 0);
    shift_register_update();
}

bool z80_get_ram_ce(void)
{
    return ram_ce;
}

bool z80_get_irq(void)
{
    return z80_irq;
}

bool z80_get_nmi(void)
{
    return z80_nmi;
}

bool z80_get_reset(void)
{
    return z80_reset;
}

void z80_set_ram_ce(bool active)
{
    if(ram_ce == active)
        return;
    ram_ce = active;
    shift_register_update();
}

void z80_set_irq(bool active)
{
    if(z80_irq == active)
        return;
    z80_irq = active;
    shift_register_update();
}

void z80_set_nmi(bool active)
{
    if(z80_nmi == active)
        return;
    z80_nmi = active;
    shift_register_update();
}

void z80_set_reset(bool active)
{
    if(z80_reset == active)
        return;
    z80_reset = active;
    shift_register_update();
}

void z80_set_user_leds(uint16_t leds)
{
    leds &= 0xFFF;

    if(user_leds == leds)
        return;

    user_leds = leds;
    shift_register_update();
}

uint16_t z80_get_user_leds(void)
{
    return user_leds;
}

bool check_pcb_revision(void)
{
    // perform a set of simple tests to try and catch when the PCB revision is misconfigured
    bool okay = true;

    shift_register_setup();

    pinMode(WAIT_RESET, OUTPUT);
    pinMode(Z80_BUSRQ, OUTPUT);
    pinMode(CLK_FAST_ENABLE, OUTPUT);
    pinMode(CLK_STROBE, OUTPUT);
    pinMode(Z80_WAIT, INPUT);
    digitalWrite(CLK_FAST_ENABLE, 0);

    z80_set_reset(true);
    for(int i=0; i<20; i++){
        digitalWrite(CLK_STROBE, 1);
        delayMicroseconds(50);
        digitalWrite(CLK_STROBE, 0);
        delayMicroseconds(50);
    }
    z80_set_reset(false);

    digitalWrite(Z80_BUSRQ, 0); // request DMA
    // can't rely on Z80_BUSACK being connected so just send a bunch of clocks out
    for(int i=0; i<20; i++){
        digitalWrite(CLK_STROBE, 1);
        delayMicroseconds(50);
        digitalWrite(CLK_STROBE, 0);
        delayMicroseconds(50);
    }
    pinMode(Z80_IORQ, OUTPUT);
    digitalWrite(Z80_IORQ, 1);
    digitalWrite(WAIT_RESET, 0); // release /WAIT
    delayMicroseconds(50);
    digitalWrite(WAIT_RESET, 1); 
    delayMicroseconds(50);
    if(!digitalRead(Z80_WAIT)) // should be 1
        okay = false;
    digitalWrite(Z80_IORQ, 0);  // strobe /IORQ, should assert /WAIT
    delayMicroseconds(50);
    digitalWrite(Z80_IORQ, 1); 
    delayMicroseconds(50);
    if(digitalRead(Z80_WAIT)) // should be 0
        okay = false;
    digitalWrite(WAIT_RESET, 0); // release /WAIT
    delayMicroseconds(50);
    digitalWrite(WAIT_RESET, 1); 
    delayMicroseconds(50);
    if(!digitalRead(Z80_WAIT)) // should be 1
        okay = false;
    digitalWrite(Z80_BUSRQ, 1); // release DMA
    return okay;
}


// NOTE it is assumed that if writing=false then those pins are already inputs, they are not forced to inputs
void z80_bus_master(bool writing)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    if(writing){
        GPIOA_PDDR |= ((1<<5) | (1<<12) | (1<<14) | (1<<15) | (1<<16) | (1<<17));
        GPIOB_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) |
                (1<<16) | (1<<17) | (1<<18) | (1<<19));
        GPIOC_PDDR |= ((1<<1) | (1<<2) | (1<<8) | (1<<9) | (1<<10) | (1<<11));
        GPIOE_PDDR |= ((1<<24) | (1<<25) | (1<<26));
    }else{
        GPIOA_PDDR |= ((1<<5) | (1<<12) | (1<<14) | (1<<15) | (1<<16));
        GPIOB_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) |
                (1<<16) | (1<<17) | (1<<18) | (1<<19));
        GPIOC_PDDR |= ((1<<1) | (1<<2));
        GPIOE_PDDR |= ((1<<26));
    }
    GPIOD_PDDR |= ((1<<0) | (1<<5) | (1<<6));
#else
    GPIOB_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) | (1<<16) | (1<<17));
    GPIOC_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7) |
                   (1<<8) | (1<<9) | (1<<10) | (1<<11));
    if(writing)
        GPIOD_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7));
#endif
#else
    pinMode(Z80_A0,    OUTPUT);
    pinMode(Z80_A1,    OUTPUT);
    pinMode(Z80_A2,    OUTPUT);
    pinMode(Z80_A3,    OUTPUT);
    pinMode(Z80_A4,    OUTPUT);
    pinMode(Z80_A5,    OUTPUT);
    pinMode(Z80_A6,    OUTPUT);
    pinMode(Z80_A7,    OUTPUT);
    pinMode(Z80_A8,    OUTPUT);
    pinMode(Z80_A9,    OUTPUT);
    pinMode(Z80_A10,   OUTPUT);
    pinMode(Z80_A11,   OUTPUT);
    pinMode(Z80_A12,   OUTPUT);
    pinMode(Z80_A13,   OUTPUT);
    pinMode(Z80_A14,   OUTPUT);
    pinMode(Z80_A15,   OUTPUT);
    pinMode(Z80_IORQ,  OUTPUT);
    pinMode(Z80_MREQ,  OUTPUT);
    pinMode(Z80_RD,    OUTPUT);
    pinMode(Z80_WR,    OUTPUT);
    if(writing){
        pinMode(Z80_D0,    OUTPUT);
        pinMode(Z80_D1,    OUTPUT);
        pinMode(Z80_D2,    OUTPUT);
        pinMode(Z80_D3,    OUTPUT);
        pinMode(Z80_D4,    OUTPUT);
        pinMode(Z80_D5,    OUTPUT);
        pinMode(Z80_D6,    OUTPUT);
        pinMode(Z80_D7,    OUTPUT);
    }
#endif
}

void z80_bus_slave(void)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    GPIOA_PDDR &=~((1<<5) | (1<<12) | (1<<14) | (1<<15) | (1<<16) | (1<<17));
    GPIOB_PDDR &=~((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) |
                   (1<<16) | (1<<17) | (1<<18) | (1<<19));
    GPIOC_PDDR &=~((1<<1) | (1<<2) | (1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOD_PDDR &=~((1<<0) | (1<<5) | (1<<6));
    GPIOE_PDDR &=~((1<<24) | (1<<25) | (1<<26));
#else
    GPIOB_PDDR &=~((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) | (1<<16) | (1<<17));
    GPIOC_PDDR &=~((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7) |
                   (1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOD_PDDR &=~((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7));
#endif
#else
    pinMode(Z80_A0,    INPUT);
    pinMode(Z80_A1,    INPUT);
    pinMode(Z80_A2,    INPUT);
    pinMode(Z80_A3,    INPUT);
    pinMode(Z80_A4,    INPUT);
    pinMode(Z80_A5,    INPUT);
    pinMode(Z80_A6,    INPUT);
    pinMode(Z80_A7,    INPUT);
    pinMode(Z80_A8,    INPUT);
    pinMode(Z80_A9,    INPUT);
    pinMode(Z80_A10,   INPUT);
    pinMode(Z80_A11,   INPUT);
    pinMode(Z80_A12,   INPUT);
    pinMode(Z80_A13,   INPUT);
    pinMode(Z80_A14,   INPUT);
    pinMode(Z80_A15,   INPUT);
    pinMode(Z80_D0,    INPUT);
    pinMode(Z80_D1,    INPUT);
    pinMode(Z80_D2,    INPUT);
    pinMode(Z80_D3,    INPUT);
    pinMode(Z80_D4,    INPUT);
    pinMode(Z80_D5,    INPUT);
    pinMode(Z80_D6,    INPUT);
    pinMode(Z80_D7,    INPUT);
    pinMode(Z80_IORQ,  INPUT_PULLUP);
    pinMode(Z80_MREQ,  INPUT_PULLUP);
    pinMode(Z80_RD,    INPUT_PULLUP);
    pinMode(Z80_WR,    INPUT_PULLUP);
#endif
}

void z80_show_pin_states(void)
{
    report("Z80_A0     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A0    ), *portOutputRegister(Z80_A0    ), *portInputRegister(Z80_A0    ), *portModeRegister(Z80_A0    ) ? "OUTPUT":"input");
    report("Z80_A1     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A1    ), *portOutputRegister(Z80_A1    ), *portInputRegister(Z80_A1    ), *portModeRegister(Z80_A1    ) ? "OUTPUT":"input");
    report("Z80_A2     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A2    ), *portOutputRegister(Z80_A2    ), *portInputRegister(Z80_A2    ), *portModeRegister(Z80_A2    ) ? "OUTPUT":"input");
    report("Z80_A3     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A3    ), *portOutputRegister(Z80_A3    ), *portInputRegister(Z80_A3    ), *portModeRegister(Z80_A3    ) ? "OUTPUT":"input");
    report("Z80_A4     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A4    ), *portOutputRegister(Z80_A4    ), *portInputRegister(Z80_A4    ), *portModeRegister(Z80_A4    ) ? "OUTPUT":"input");
    report("Z80_A5     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A5    ), *portOutputRegister(Z80_A5    ), *portInputRegister(Z80_A5    ), *portModeRegister(Z80_A5    ) ? "OUTPUT":"input");
    report("Z80_A6     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A6    ), *portOutputRegister(Z80_A6    ), *portInputRegister(Z80_A6    ), *portModeRegister(Z80_A6    ) ? "OUTPUT":"input");
    report("Z80_A7     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A7    ), *portOutputRegister(Z80_A7    ), *portInputRegister(Z80_A7    ), *portModeRegister(Z80_A7    ) ? "OUTPUT":"input");
    report("Z80_A8     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A8    ), *portOutputRegister(Z80_A8    ), *portInputRegister(Z80_A8    ), *portModeRegister(Z80_A8    ) ? "OUTPUT":"input");
    report("Z80_A9     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A9    ), *portOutputRegister(Z80_A9    ), *portInputRegister(Z80_A9    ), *portModeRegister(Z80_A9    ) ? "OUTPUT":"input");
    report("Z80_A10    PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A10   ), *portOutputRegister(Z80_A10   ), *portInputRegister(Z80_A10   ), *portModeRegister(Z80_A10   ) ? "OUTPUT":"input");
    report("Z80_A11    PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A11   ), *portOutputRegister(Z80_A11   ), *portInputRegister(Z80_A11   ), *portModeRegister(Z80_A11   ) ? "OUTPUT":"input");
    report("Z80_A12    PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A12   ), *portOutputRegister(Z80_A12   ), *portInputRegister(Z80_A12   ), *portModeRegister(Z80_A12   ) ? "OUTPUT":"input");
    report("Z80_A13    PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A13   ), *portOutputRegister(Z80_A13   ), *portInputRegister(Z80_A13   ), *portModeRegister(Z80_A13   ) ? "OUTPUT":"input");
    report("Z80_A14    PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A14   ), *portOutputRegister(Z80_A14   ), *portInputRegister(Z80_A14   ), *portModeRegister(Z80_A14   ) ? "OUTPUT":"input");
    report("Z80_A15    PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_A15   ), *portOutputRegister(Z80_A15   ), *portInputRegister(Z80_A15   ), *portModeRegister(Z80_A15   ) ? "OUTPUT":"input");
    report("Z80_D0     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_D0    ), *portOutputRegister(Z80_D0    ), *portInputRegister(Z80_D0    ), *portModeRegister(Z80_D0    ) ? "OUTPUT":"input");
    report("Z80_D1     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_D1    ), *portOutputRegister(Z80_D1    ), *portInputRegister(Z80_D1    ), *portModeRegister(Z80_D1    ) ? "OUTPUT":"input");
    report("Z80_D2     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_D2    ), *portOutputRegister(Z80_D2    ), *portInputRegister(Z80_D2    ), *portModeRegister(Z80_D2    ) ? "OUTPUT":"input");
    report("Z80_D3     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_D3    ), *portOutputRegister(Z80_D3    ), *portInputRegister(Z80_D3    ), *portModeRegister(Z80_D3    ) ? "OUTPUT":"input");
    report("Z80_D4     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_D4    ), *portOutputRegister(Z80_D4    ), *portInputRegister(Z80_D4    ), *portModeRegister(Z80_D4    ) ? "OUTPUT":"input");
    report("Z80_D5     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_D5    ), *portOutputRegister(Z80_D5    ), *portInputRegister(Z80_D5    ), *portModeRegister(Z80_D5    ) ? "OUTPUT":"input");
    report("Z80_D6     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_D6    ), *portOutputRegister(Z80_D6    ), *portInputRegister(Z80_D6    ), *portModeRegister(Z80_D6    ) ? "OUTPUT":"input");
    report("Z80_D7     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_D7    ), *portOutputRegister(Z80_D7    ), *portInputRegister(Z80_D7    ), *portModeRegister(Z80_D7    ) ? "OUTPUT":"input");
    report("Z80_M1     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_M1    ), *portOutputRegister(Z80_M1    ), *portInputRegister(Z80_M1    ), *portModeRegister(Z80_M1    ) ? "OUTPUT":"input");
    report("Z80_RD     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_RD    ), *portOutputRegister(Z80_RD    ), *portInputRegister(Z80_RD    ), *portModeRegister(Z80_RD    ) ? "OUTPUT":"input");
    report("Z80_WR     PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_WR    ), *portOutputRegister(Z80_WR    ), *portInputRegister(Z80_WR    ), *portModeRegister(Z80_WR    ) ? "OUTPUT":"input");
    report("Z80_IORQ   PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_IORQ  ), *portOutputRegister(Z80_IORQ  ), *portInputRegister(Z80_IORQ  ), *portModeRegister(Z80_IORQ  ) ? "OUTPUT":"input");
    report("Z80_MREQ   PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_MREQ  ), *portOutputRegister(Z80_MREQ  ), *portInputRegister(Z80_MREQ  ), *portModeRegister(Z80_MREQ  ) ? "OUTPUT":"input");
    report("Z80_HALT   PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_HALT  ), *portOutputRegister(Z80_HALT  ), *portInputRegister(Z80_HALT  ), *portModeRegister(Z80_HALT  ) ? "OUTPUT":"input");
    report("Z80_WAIT   PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_WAIT  ), *portOutputRegister(Z80_WAIT  ), *portInputRegister(Z80_WAIT  ), *portModeRegister(Z80_WAIT  ) ? "OUTPUT":"input");
    report("Z80_BUSACK PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_BUSACK), *portOutputRegister(Z80_BUSACK), *portInputRegister(Z80_BUSACK), *portModeRegister(Z80_BUSACK) ? "OUTPUT":"input");
    report("Z80_BUSRQ  PCR=%08x Out=%d In=%d %s\r\n", *portConfigRegister(Z80_BUSRQ ), *portOutputRegister(Z80_BUSRQ ), *portInputRegister(Z80_BUSRQ ), *portModeRegister(Z80_BUSRQ ) ? "OUTPUT":"input");
}

void z80_setup(void)
{
    shift_register_setup();

    z80_set_mreq(false);
    z80_set_iorq(false);
    z80_set_rd(false);
    z80_set_wr(false);
#ifdef KINETISK
    // note SRE = slew rate limiting
    // we set the config ONCE ONLY, suitable for both input and output. we need to
    // avoid using pinMode() on these registers as it will overwrite our config!
    *portConfigRegister(Z80_A0  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A1  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A2  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A3  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A4  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A5  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A6  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A7  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A8  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A9  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A10 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A11 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A12 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A13 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A14 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_A15 ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D0  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D1  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D2  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D3  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D4  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D5  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D6  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_D7  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    // enable pull-ups on our control signals so they do not float when transitioning to/from DMA states
    *portConfigRegister(Z80_RD  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;
    *portConfigRegister(Z80_WR  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;
    *portConfigRegister(Z80_IORQ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;
    *portConfigRegister(Z80_MREQ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;
    // drive all Z80 bus pins to 1
#ifdef ERSATZ80_PCB_REV1
    GPIOA_PSOR = ((1<<5) | (1<<12) | (1<<14) | (1<<15) | (1<<16) | (1<<17));
    GPIOB_PSOR = ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) |
                  (1<<16) | (1<<17) | (1<<18) | (1<<19));
    GPIOC_PSOR = ((1<<1) | (1<<2) | (1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOD_PSOR = ((1<<0) | (1<<5) | (1<<6));
    GPIOE_PSOR = ((1<<24) | (1<<25) | (1<<26));
#else
    GPIOB_PSOR = ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) | (1<<16) | (1<<17));
    GPIOC_PSOR = ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7) |
                   (1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOD_PSOR = ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7));
#endif
#endif
    z80_bus_slave();
    pinMode(Z80_M1, INPUT);
    pinMode(Z80_WAIT, INPUT);
    pinMode(Z80_HALT, INPUT);
    pinMode(Z80_BUSACK, INPUT);
    pinMode(Z80_BUSRQ, OUTPUT);

    pinMode(MMU_EW, OUTPUT);
    pinMode(WAIT_RESET, OUTPUT);

    // put us into MODE_SUPERVISED
    z80_set_release_wait(true);
    z80_set_busrq(false);
    z80_set_mmu_ew(false);
    z80_set_ram_ce(true);
    z80_set_release_wait(false);
    z80_mode = MODE_SUPERVISED;

    z80_do_reset();
}

uint16_t z80_read_bus_address(void)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    unsigned int gpio_a = GPIOA_PDIR;
    unsigned int gpio_b = GPIOB_PDIR;
    unsigned int gpio_c = GPIOC_PDIR;
    unsigned int gpio_d = GPIOD_PDIR;
    unsigned int gpio_e = GPIOE_PDIR;

    return ((gpio_b & (1<< 0 )) >> 0  <<  0) |
           ((gpio_b & (1<< 1 )) >> 1  <<  1) |
           ((gpio_b & (1<< 3 )) >> 3  <<  2) |
           ((gpio_b & (1<< 2 )) >> 2  <<  3) |
           ((gpio_d & (1<< 5 )) >> 5  <<  4) |
           ((gpio_d & (1<< 6 )) >> 6  <<  5) |
           ((gpio_c & (1<< 1 )) >> 1  <<  6) |
           ((gpio_c & (1<< 2 )) >> 2  <<  7) |
           ((gpio_e & (1<< 26)) >> 26 <<  8) |
           ((gpio_a & (1<< 5 )) >> 5  <<  9) |
           ((gpio_a & (1<< 14)) >> 14 << 10) |
           ((gpio_a & (1<< 15)) >> 15 << 11) |
           ((gpio_a & (1<< 16)) >> 16 << 12) |
           ((gpio_b & (1<< 18)) >> 18 << 13) |
           ((gpio_b & (1<< 19)) >> 19 << 14) |
           ((gpio_b & (1<< 10)) >> 10 << 15);
#else
    return (GPIOC_PDIR & 0xFFF) | ((GPIOB_PDIR & 0xF) << 12);
#endif
#else
    return  digitalRead(Z80_A0)         |
           (digitalRead(Z80_A1)  <<  1) |
           (digitalRead(Z80_A2)  <<  2) |
           (digitalRead(Z80_A3)  <<  3) |
           (digitalRead(Z80_A4)  <<  4) |
           (digitalRead(Z80_A5)  <<  5) |
           (digitalRead(Z80_A6)  <<  6) |
           (digitalRead(Z80_A7)  <<  7) |
           (digitalRead(Z80_A8)  <<  8) |
           (digitalRead(Z80_A9)  <<  9) |
           (digitalRead(Z80_A10) << 10) |
           (digitalRead(Z80_A11) << 11) |
           (digitalRead(Z80_A12) << 12) |
           (digitalRead(Z80_A13) << 13) |
           (digitalRead(Z80_A14) << 14) |
           (digitalRead(Z80_A15) << 15);
#endif
}

uint8_t z80_read_bus_address_low8(void)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    unsigned int gpio_b = GPIOB_PDIR;
    unsigned int gpio_c = GPIOC_PDIR;
    unsigned int gpio_d = GPIOD_PDIR;

    return ((gpio_b & (1<< 0 )) >>  0  <<  0) |
           ((gpio_b & (1<< 1 )) >>  1  <<  1) |
           ((gpio_b & (1<< 3 )) >>  3  <<  2) |
           ((gpio_b & (1<< 2 )) >>  2  <<  3) |
           ((gpio_d & (1<< 5 )) >>  5  <<  4) |
           ((gpio_d & (1<< 6 )) >>  6  <<  5) |
           ((gpio_c & (1<< 1 )) >>  1  <<  6) |
           ((gpio_c & (1<< 2 )) >>  2  <<  7);
#else
    return (GPIOC_PDIR & 0xFF);
#endif
#else
    return  digitalRead(Z80_A0)       |
           (digitalRead(Z80_A1) << 1) |
           (digitalRead(Z80_A2) << 2) |
           (digitalRead(Z80_A3) << 3) |
           (digitalRead(Z80_A4) << 4) |
           (digitalRead(Z80_A5) << 5) |
           (digitalRead(Z80_A6) << 6) |
           (digitalRead(Z80_A7) << 7);
#endif
}

uint8_t z80_read_bus_data(void)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    unsigned int gpio_a = GPIOA_PDIR;
    unsigned int gpio_b = GPIOB_PDIR;
    unsigned int gpio_c = GPIOC_PDIR;
    unsigned int gpio_e = GPIOE_PDIR;

    return ((gpio_b & (1<< 11 )) >> 11 << 0) |
           ((gpio_e & (1<< 24 )) >> 24 << 1) |
           ((gpio_e & (1<< 25 )) >> 25 << 2) |
           ((gpio_c & (1<< 8  )) >> 8  << 3) |
           ((gpio_c & (1<< 9  )) >> 9  << 4) |
           ((gpio_c & (1<< 10 )) >> 10 << 5) |
           ((gpio_c & (1<< 11 )) >> 11 << 6) |
           ((gpio_a & (1<< 17 )) >> 17 << 7);
#else
    return (GPIOD_PDIR & 0xFF);
#endif
#else
    return  digitalRead(Z80_D0)       | // digitalRead() returns only 0 or 1
           (digitalRead(Z80_D1) << 1) |
           (digitalRead(Z80_D2) << 2) |
           (digitalRead(Z80_D3) << 3) |
           (digitalRead(Z80_D4) << 4) |
           (digitalRead(Z80_D5) << 5) |
           (digitalRead(Z80_D6) << 6) |
           (digitalRead(Z80_D7) << 7);
#endif
}

void z80_bus_data_outputs(void)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    GPIOA_PDDR |= ((1<<17));
    GPIOB_PDDR |= ((1<<11));
    GPIOC_PDDR |= ((1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOE_PDDR |= ((1<<24) | (1<<25));
#else
    GPIOD_PDDR |= 0xFF;
#endif
#else
    pinMode(Z80_D0, OUTPUT);
    pinMode(Z80_D1, OUTPUT);
    pinMode(Z80_D2, OUTPUT);
    pinMode(Z80_D3, OUTPUT);
    pinMode(Z80_D4, OUTPUT);
    pinMode(Z80_D5, OUTPUT);
    pinMode(Z80_D6, OUTPUT);
    pinMode(Z80_D7, OUTPUT);
#endif
}

void z80_bus_set_data(uint8_t data)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    *portOutputRegister(Z80_D0) = data; // bitband memory accesses only consider the lowest bit
    *portOutputRegister(Z80_D1) = data >> 1;
    *portOutputRegister(Z80_D2) = data >> 2;
    *portOutputRegister(Z80_D3) = data >> 3;
    *portOutputRegister(Z80_D4) = data >> 4;
    *portOutputRegister(Z80_D5) = data >> 5;
    *portOutputRegister(Z80_D6) = data >> 6;
    *portOutputRegister(Z80_D7) = data >> 7;
#else
    GPIOD_PDOR = (GPIOD_PDOR & ~0xFF) | data;
#endif
#else
    digitalWrite(Z80_D0, data & (1<<0) ? 1 : 0);
    digitalWrite(Z80_D1, data & (1<<1) ? 1 : 0);
    digitalWrite(Z80_D2, data & (1<<2) ? 1 : 0);
    digitalWrite(Z80_D3, data & (1<<3) ? 1 : 0);
    digitalWrite(Z80_D4, data & (1<<4) ? 1 : 0);
    digitalWrite(Z80_D5, data & (1<<5) ? 1 : 0);
    digitalWrite(Z80_D6, data & (1<<6) ? 1 : 0);
    digitalWrite(Z80_D7, data & (1<<7) ? 1 : 0);
#endif
}

void z80_bus_data_inputs(void)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    GPIOA_PDDR &= ~((1<<17));
    GPIOB_PDDR &= ~((1<<11));
    GPIOC_PDDR &= ~((1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOE_PDDR &= ~((1<<24) | (1<<25));
#else
    GPIOD_PDDR &= ~0xFF;
#endif
#else
    pinMode(Z80_D0,   INPUT);
    pinMode(Z80_D1,   INPUT);
    pinMode(Z80_D2,   INPUT);
    pinMode(Z80_D3,   INPUT);
    pinMode(Z80_D4,   INPUT);
    pinMode(Z80_D5,   INPUT);
    pinMode(Z80_D6,   INPUT);
    pinMode(Z80_D7,   INPUT);
#endif
}

void z80_bus_set_address(uint16_t address)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    *portOutputRegister(Z80_A0 ) = address;
    *portOutputRegister(Z80_A1 ) = address >>  1;
    *portOutputRegister(Z80_A2 ) = address >>  2;
    *portOutputRegister(Z80_A3 ) = address >>  3;
    *portOutputRegister(Z80_A4 ) = address >>  4;
    *portOutputRegister(Z80_A5 ) = address >>  5;
    *portOutputRegister(Z80_A6 ) = address >>  6;
    *portOutputRegister(Z80_A7 ) = address >>  7;
    *portOutputRegister(Z80_A8 ) = address >>  8;
    *portOutputRegister(Z80_A9 ) = address >>  9;
    *portOutputRegister(Z80_A10) = address >> 10;
    *portOutputRegister(Z80_A11) = address >> 11;
    *portOutputRegister(Z80_A12) = address >> 12;
    *portOutputRegister(Z80_A13) = address >> 13;
    *portOutputRegister(Z80_A14) = address >> 14;
    *portOutputRegister(Z80_A15) = address >> 15;
#else
    GPIOC_PDOR = (GPIOC_PDOR & ~0xFFF) | (address & 0xFFF);
    GPIOB_PDOR = (GPIOB_PDOR & ~0xF)   | (address >> 12);
#endif
#else
    digitalWrite(Z80_A0,  address & (1<<0)  ? 1 : 0);
    digitalWrite(Z80_A1,  address & (1<<1)  ? 1 : 0);
    digitalWrite(Z80_A2,  address & (1<<2)  ? 1 : 0);
    digitalWrite(Z80_A3,  address & (1<<3)  ? 1 : 0);
    digitalWrite(Z80_A4,  address & (1<<4)  ? 1 : 0);
    digitalWrite(Z80_A5,  address & (1<<5)  ? 1 : 0);
    digitalWrite(Z80_A6,  address & (1<<6)  ? 1 : 0);
    digitalWrite(Z80_A7,  address & (1<<7)  ? 1 : 0);
    digitalWrite(Z80_A8,  address & (1<<8)  ? 1 : 0);
    digitalWrite(Z80_A9,  address & (1<<9)  ? 1 : 0);
    digitalWrite(Z80_A10, address & (1<<10) ? 1 : 0);
    digitalWrite(Z80_A11, address & (1<<11) ? 1 : 0);
    digitalWrite(Z80_A12, address & (1<<12) ? 1 : 0);
    digitalWrite(Z80_A13, address & (1<<13) ? 1 : 0);
    digitalWrite(Z80_A14, address & (1<<14) ? 1 : 0);
    digitalWrite(Z80_A15, address & (1<<15) ? 1 : 0);
#endif
}

void z80_bus_set_address_data(uint16_t address, uint8_t data)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    *portOutputRegister(Z80_A0 ) = address;
    *portOutputRegister(Z80_A1 ) = address >>  1;
    *portOutputRegister(Z80_A2 ) = address >>  2;
    *portOutputRegister(Z80_A3 ) = address >>  3;
    *portOutputRegister(Z80_A4 ) = address >>  4;
    *portOutputRegister(Z80_A5 ) = address >>  5;
    *portOutputRegister(Z80_A6 ) = address >>  6;
    *portOutputRegister(Z80_A7 ) = address >>  7;
    *portOutputRegister(Z80_A8 ) = address >>  8;
    *portOutputRegister(Z80_A9 ) = address >>  9;
    *portOutputRegister(Z80_A10) = address >> 10;
    *portOutputRegister(Z80_A11) = address >> 11;
    *portOutputRegister(Z80_A12) = address >> 12;
    *portOutputRegister(Z80_A13) = address >> 13;
    *portOutputRegister(Z80_A14) = address >> 14;
    *portOutputRegister(Z80_A15) = address >> 15;
    *portOutputRegister(Z80_D0 ) = data;
    *portOutputRegister(Z80_D1 ) = data >> 1;
    *portOutputRegister(Z80_D2 ) = data >> 2;
    *portOutputRegister(Z80_D3 ) = data >> 3;
    *portOutputRegister(Z80_D4 ) = data >> 4;
    *portOutputRegister(Z80_D5 ) = data >> 5;
    *portOutputRegister(Z80_D6 ) = data >> 6;
    *portOutputRegister(Z80_D7 ) = data >> 7;
#else
    GPIOC_PDOR = (GPIOC_PDOR & ~0xFFF) | (address & 0xFFF);
    GPIOB_PDOR = (GPIOB_PDOR & ~0xF)   | (address >> 12);
    GPIOD_PDOR = (GPIOD_PDOR & ~0xFF)  | data;
#endif
#else
    digitalWrite(Z80_A0,  address & (1<<0)  ? 1 : 0);
    digitalWrite(Z80_A1,  address & (1<<1)  ? 1 : 0);
    digitalWrite(Z80_A2,  address & (1<<2)  ? 1 : 0);
    digitalWrite(Z80_A3,  address & (1<<3)  ? 1 : 0);
    digitalWrite(Z80_A4,  address & (1<<4)  ? 1 : 0);
    digitalWrite(Z80_A5,  address & (1<<5)  ? 1 : 0);
    digitalWrite(Z80_A6,  address & (1<<6)  ? 1 : 0);
    digitalWrite(Z80_A7,  address & (1<<7)  ? 1 : 0);
    digitalWrite(Z80_A8,  address & (1<<8)  ? 1 : 0);
    digitalWrite(Z80_A9,  address & (1<<9)  ? 1 : 0);
    digitalWrite(Z80_A10, address & (1<<10) ? 1 : 0);
    digitalWrite(Z80_A11, address & (1<<11) ? 1 : 0);
    digitalWrite(Z80_A12, address & (1<<12) ? 1 : 0);
    digitalWrite(Z80_A13, address & (1<<13) ? 1 : 0);
    digitalWrite(Z80_A14, address & (1<<14) ? 1 : 0);
    digitalWrite(Z80_A15, address & (1<<15) ? 1 : 0);

    digitalWrite(Z80_D0, data & (1<<0) ? 1 : 0);
    digitalWrite(Z80_D1, data & (1<<1) ? 1 : 0);
    digitalWrite(Z80_D2, data & (1<<2) ? 1 : 0);
    digitalWrite(Z80_D3, data & (1<<3) ? 1 : 0);
    digitalWrite(Z80_D4, data & (1<<4) ? 1 : 0);
    digitalWrite(Z80_D5, data & (1<<5) ? 1 : 0);
    digitalWrite(Z80_D6, data & (1<<6) ? 1 : 0);
    digitalWrite(Z80_D7, data & (1<<7) ? 1 : 0);
#endif
}
