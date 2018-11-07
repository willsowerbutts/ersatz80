#include <Arduino.h>
#include "rom.h"
#include "debug.h"
#include "clock.h"
#include "z80.h"
#include "irq.h"
#include "disk.h"
#include "disasm.h"

uint16_t user_led = 0x000; // value to show on user-controlled LEDs
bool z80_reset = true;       // Z80 /RESET pin (z80_reset=true means /RESET is driven low ie asserted)
bool z80_irq = false;        // Z80 /IRQ pin
bool z80_nmi = false;        // Z80 /NMI pin
bool ram_ce = false;         // RAM /CE pin
int z80_bus_trace = 0;
uint8_t mmu[4];
uint8_t mmu_foreign[4];
uint8_t ram_pages = 0;

void shift_register_update(void)
{
    unsigned int bit = 0x8000;
    unsigned int output =
        (user_led << 4) |
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

void z80_bus_master(void)
{
#ifdef KINETISK
    // if(*portOutputRegister(Z80_MREQ) == 0)
    //     report("z80_bus_master: MREQ=0!\r\n");
    // if(*portOutputRegister(Z80_IORQ) == 0)
    //     report("z80_bus_master: IORQ=0!\r\n");
    // if(*portOutputRegister(Z80_WR) == 0)
    //     report("z80_bus_master: WR=0!\r\n");
    // if(*portOutputRegister(Z80_RD) == 0)
    //     report("z80_bus_master: RD=0!\r\n");
#ifdef ERSATZ80_PCB_REV1
    GPIOA_PDDR |= ((1<<5) | (1<<12) | (1<<14) | (1<<15) | (1<<16) | (1<<17));
    GPIOB_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) |
                   (1<<16) | (1<<17) | (1<<18) | (1<<19));
    GPIOC_PDDR |= ((1<<1) | (1<<2) | (1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOD_PDDR |= ((1<<0) | (1<<5) | (1<<6));
    GPIOE_PDDR |= ((1<<24) | (1<<25) | (1<<26));
#else
    GPIOB_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) | (1<<16) | (1<<17));
    GPIOC_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7) |
                   (1<<8) | (1<<9) | (1<<10) | (1<<11));
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
    pinMode(Z80_D0,    OUTPUT);
    pinMode(Z80_D1,    OUTPUT);
    pinMode(Z80_D2,    OUTPUT);
    pinMode(Z80_D3,    OUTPUT);
    pinMode(Z80_D4,    OUTPUT);
    pinMode(Z80_D5,    OUTPUT);
    pinMode(Z80_D6,    OUTPUT);
    pinMode(Z80_D7,    OUTPUT);
    pinMode(Z80_IORQ,  OUTPUT);
    pinMode(Z80_MREQ,  OUTPUT);
    pinMode(Z80_RD,    OUTPUT);
    pinMode(Z80_WR,    OUTPUT);
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
    pinMode(Z80_IORQ,  INPUT);
    pinMode(Z80_MREQ,  INPUT);
    pinMode(Z80_RD,    INPUT);
    pinMode(Z80_WR,    INPUT);
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
    z80_clk_init();

    // shift register setup
    pinMode(SHIFT_REGISTER_CLK, OUTPUT);
    pinMode(SHIFT_REGISTER_LATCH, OUTPUT);
    pinMode(SHIFT_REGISTER_DATA, OUTPUT);

    digitalWrite(SHIFT_REGISTER_CLK, 0);
    digitalWrite(SHIFT_REGISTER_LATCH, 0);
    digitalWrite(SHIFT_REGISTER_DATA, 0);
    shift_register_update();

    digitalWrite(Z80_IORQ, 1);
    digitalWrite(Z80_MREQ, 1);
    digitalWrite(Z80_RD, 1);
    digitalWrite(Z80_WR, 1);
    z80_bus_slave();
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
    *portConfigRegister(Z80_RD  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_WR  ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_IORQ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
    *portConfigRegister(Z80_MREQ) = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
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
    pinMode(Z80_M1, INPUT);
    pinMode(Z80_WAIT, INPUT);
    pinMode(Z80_HALT, INPUT);
    pinMode(Z80_BUSACK, INPUT);
    pinMode(Z80_BUSRQ, OUTPUT);

    // system
    pinMode(MMU_EW, OUTPUT);
    pinMode(WAIT_RESET, OUTPUT);

    digitalWrite(Z80_BUSRQ, 1);
    digitalWrite(MMU_EW, 1);
    digitalWrite(WAIT_RESET, 1);

    z80_set_clk(true);
    z80_set_reset(true);
}

uint16_t z80_bus_address(void)
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

uint8_t z80_bus_address_low8(void)
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

uint8_t z80_bus_data(void)
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

void z80_set_reset(bool active)
{
    z80_reset = active;
    shift_register_update();
}

void z80_setup_drive_data(uint8_t data)
{
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
    GPIOA_PDDR |= ((1<<17));
    GPIOB_PDDR |= ((1<<11));
    GPIOC_PDDR |= ((1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOE_PDDR |= ((1<<24) | (1<<25));
    *portOutputRegister(Z80_D0) = data; // bitband memory accesses only consider the lowest bit
    *portOutputRegister(Z80_D1) = data >> 1;
    *portOutputRegister(Z80_D2) = data >> 2;
    *portOutputRegister(Z80_D3) = data >> 3;
    *portOutputRegister(Z80_D4) = data >> 4;
    *portOutputRegister(Z80_D5) = data >> 5;
    *portOutputRegister(Z80_D6) = data >> 6;
    *portOutputRegister(Z80_D7) = data >> 7;
#else
    GPIOD_PDDR |= 0xFF;
    GPIOD_PDOR = (GPIOD_PDOR & ~0xFF) | data;
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

void z80_shutdown_drive_data(void)
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

void z80_clock_pulse_drive_data(uint8_t data)
{
    z80_setup_drive_data(data);

    do{
        z80_clock_pulse();
    }while( z80_rd_asserted() );

    z80_shutdown_drive_data();
}

void z80_clock_pulse_while_writing(void)
{
    do{
        z80_clock_pulse();
    }while( z80_wr_asserted() );
}

void z80_do_reset(void)
{
    z80_clk_pause(false);
    z80_set_reset(true);
    z80_set_release_wait(true);
    for(int i=0; i<10; i++){ // Z80 requires at least 3 clocks to fully reset
        z80_set_clk(false);
        z80_set_clk(true);
    }
    z80_set_release_wait(false);
    z80_set_reset(false);
    z80_clk_resume();
}

void z80_setup_address(uint16_t address)
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

void z80_setup_address_data(uint16_t address, uint8_t data)
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

void z80_mmu_write(uint8_t bank, uint8_t data)
{
    /* assert(DMA_MODE / WR_ADDR | WR_DATA / ...)  ? to detect being called in wrong mode */
    z80_setup_address_data(bank << 14, data);
#ifdef KINETISK
    *portOutputRegister(MMU_EW) = 0;
    *portOutputRegister(MMU_EW) = 1;
#else
    digitalWrite(MMU_EW, 0);
    digitalWrite(MMU_EW, 1);
#endif
}

void z80_wipe_page(void)
{
    z80_setup_address_data(0, 0);
    *portOutputRegister(Z80_MREQ) = 0;
    *portOutputRegister(Z80_WR) = 0;
    // there must be a more efficient way to do this, since
    // relatively few of the pins change on most cycles:
    for(int i=0; i<16384; i++){
#ifdef ERSATZ80_PCB_REV1
        *portOutputRegister(Z80_A0 ) = i;
        *portOutputRegister(Z80_A1 ) = i >>  1;
        *portOutputRegister(Z80_A2 ) = i >>  2;
        *portOutputRegister(Z80_A3 ) = i >>  3;
        *portOutputRegister(Z80_A4 ) = i >>  4;
        *portOutputRegister(Z80_A5 ) = i >>  5;
        *portOutputRegister(Z80_A6 ) = i >>  6;
        *portOutputRegister(Z80_A7 ) = i >>  7;
        if(!(i&0xff)){ // skip updating this half
            *portOutputRegister(Z80_A8 ) = i >>  8;
            *portOutputRegister(Z80_A9 ) = i >>  9;
            *portOutputRegister(Z80_A10) = i >> 10;
            *portOutputRegister(Z80_A11) = i >> 11;
            *portOutputRegister(Z80_A12) = i >> 12;
            *portOutputRegister(Z80_A13) = i >> 13;
            *portOutputRegister(Z80_A14) = i >> 14;
            *portOutputRegister(Z80_A15) = i >> 15;
        }
#else
        GPIOC_PDOR = (GPIOC_PDOR & ~0xFFF) | (i & 0xFFF);
        GPIOB_PDOR = (GPIOB_PDOR & ~0xF)   | (i >> 12);
#endif
    }
    *portOutputRegister(Z80_WR) = 1;
    *portOutputRegister(Z80_MREQ) = 1;
}

void z80_memory_write(uint16_t address, uint8_t data)
{
    /* assert(DMA_MODE / WR_ADDR | WR_DATA / ...)  ? to detect being called in wrong mode */
    z80_setup_address_data(address, data);
#ifdef KINETISK
    *portOutputRegister(Z80_MREQ) = 0;
    *portOutputRegister(Z80_WR) = 0;
    delayMicroseconds(1);
    *portOutputRegister(Z80_WR) = 1;
    *portOutputRegister(Z80_MREQ) = 1;
#else
    digitalWrite(Z80_MREQ, 0);
    digitalWrite(Z80_WR, 0);
    delayMicroseconds(1);
    digitalWrite(Z80_WR, 1);
    digitalWrite(Z80_MREQ, 1);
#endif
}

void z80_memory_write_block(uint16_t address, const uint8_t *dataptr, uint16_t count)
{
    uint8_t data;
    if(count == 0)
        return;
    z80_setup_address_data(address, *dataptr);
#ifdef KINETISK
    *portOutputRegister(Z80_MREQ) = 0;
    *portOutputRegister(Z80_WR) = 0;
#else
    digitalWrite(Z80_MREQ, 0);
    digitalWrite(Z80_WR, 0);
#endif
    while(count){
        data = *(dataptr++);
        count--;
#ifdef KINETISK
        *portOutputRegister(Z80_WR) = 1;
#ifdef ERSATZ80_PCB_REV1
        *portOutputRegister(Z80_D0) = data;
        *portOutputRegister(Z80_D1) = data >> 1;
        *portOutputRegister(Z80_D2) = data >> 2;
        *portOutputRegister(Z80_D3) = data >> 3;
        *portOutputRegister(Z80_D4) = data >> 4;
        *portOutputRegister(Z80_D5) = data >> 5;
        *portOutputRegister(Z80_D6) = data >> 6;
        *portOutputRegister(Z80_D7) = data >> 7;
        *portOutputRegister(Z80_A0) = address;
        *portOutputRegister(Z80_A1) = address >> 1;
        *portOutputRegister(Z80_A2) = address >> 2;
        *portOutputRegister(Z80_A3) = address >> 3;
        *portOutputRegister(Z80_A4) = address >> 4;
        *portOutputRegister(Z80_A5) = address >> 5;
        *portOutputRegister(Z80_A6) = address >> 6;
        *portOutputRegister(Z80_A7) = address >> 7;
        if(!(address&0xff)){ // skip updating this half
            *portOutputRegister(Z80_A8)  = address >>  8;
            *portOutputRegister(Z80_A9)  = address >>  9;
            *portOutputRegister(Z80_A10) = address >> 10;
            *portOutputRegister(Z80_A11) = address >> 11;
            *portOutputRegister(Z80_A12) = address >> 12;
            *portOutputRegister(Z80_A13) = address >> 13;
            *portOutputRegister(Z80_A14) = address >> 14;
            *portOutputRegister(Z80_A15) = address >> 15;
        }
#else
        GPIOC_PDOR = (GPIOC_PDOR & ~0xFFF) | (address & 0xFFF);
        GPIOB_PDOR = (GPIOB_PDOR & ~0xF)   | (address >> 12);
        GPIOD_PDOR = (GPIOD_PDOR & ~0xFF)  | data;
#endif
        *portOutputRegister(Z80_WR) = 0;
#else
        digitalWrite(Z80_WR, 1);
        digitalWrite(Z80_D0, data & (1<<0) ? 1 : 0);
        digitalWrite(Z80_D1, data & (1<<1) ? 1 : 0);
        digitalWrite(Z80_D2, data & (1<<2) ? 1 : 0);
        digitalWrite(Z80_D3, data & (1<<3) ? 1 : 0);
        digitalWrite(Z80_D4, data & (1<<4) ? 1 : 0);
        digitalWrite(Z80_D5, data & (1<<5) ? 1 : 0);
        digitalWrite(Z80_D6, data & (1<<6) ? 1 : 0);
        digitalWrite(Z80_D7, data & (1<<7) ? 1 : 0);
        digitalWrite(Z80_A0, address & (1<<0) ? 1 : 0);
        digitalWrite(Z80_A1, address & (1<<1) ? 1 : 0);
        digitalWrite(Z80_A2, address & (1<<2) ? 1 : 0);
        digitalWrite(Z80_A3, address & (1<<3) ? 1 : 0);
        digitalWrite(Z80_A4, address & (1<<4) ? 1 : 0);
        digitalWrite(Z80_A5, address & (1<<5) ? 1 : 0);
        digitalWrite(Z80_A6, address & (1<<6) ? 1 : 0);
        digitalWrite(Z80_A7, address & (1<<7) ? 1 : 0);
        if(!(address&0xff)){ // skip updating this half
            digitalWrite(Z80_A8,  address & (1<<8)  ? 1 : 0);
            digitalWrite(Z80_A9,  address & (1<<9)  ? 1 : 0);
            digitalWrite(Z80_A10, address & (1<<10) ? 1 : 0);
            digitalWrite(Z80_A11, address & (1<<11) ? 1 : 0);
            digitalWrite(Z80_A12, address & (1<<12) ? 1 : 0);
            digitalWrite(Z80_A13, address & (1<<13) ? 1 : 0);
            digitalWrite(Z80_A14, address & (1<<14) ? 1 : 0);
            digitalWrite(Z80_A15, address & (1<<15) ? 1 : 0);
        }
        digitalWrite(Z80_WR, 0);
#endif
        address++;
    }
#ifdef KINETISK
    *portOutputRegister(Z80_WR) = 1;
    *portOutputRegister(Z80_MREQ) = 1;
#else
    digitalWrite(Z80_WR, 1);
    digitalWrite(Z80_MREQ, 1);
#endif
}

void z80_memory_read_block(uint16_t address, uint8_t *dataptr, uint16_t count)
{
    if(count == 0)
        return;
    z80_setup_address(address);
    z80_shutdown_drive_data();
#ifdef KINETISK
    *portOutputRegister(Z80_MREQ) = 0;
    *portOutputRegister(Z80_RD) = 0;
#else
    digitalWrite(Z80_MREQ, 0);
    digitalWrite(Z80_RD, 0);
#endif
    while(count){
        *(dataptr++) = z80_bus_data();
        address++;
#ifdef KINETISK
#ifdef ERSATZ80_PCB_REV1
        *portOutputRegister(Z80_A0) = address;
        *portOutputRegister(Z80_A1) = address >> 1;
        *portOutputRegister(Z80_A2) = address >> 2;
        *portOutputRegister(Z80_A3) = address >> 3;
        *portOutputRegister(Z80_A4) = address >> 4;
        *portOutputRegister(Z80_A5) = address >> 5;
        *portOutputRegister(Z80_A6) = address >> 6;
        *portOutputRegister(Z80_A7) = address >> 7;
        if(!(address&0xff)){ // skip updating this half
            *portOutputRegister(Z80_A8)  = address >>  8;
            *portOutputRegister(Z80_A9)  = address >>  9;
            *portOutputRegister(Z80_A10) = address >> 10;
            *portOutputRegister(Z80_A11) = address >> 11;
            *portOutputRegister(Z80_A12) = address >> 12;
            *portOutputRegister(Z80_A13) = address >> 13;
            *portOutputRegister(Z80_A14) = address >> 14;
            *portOutputRegister(Z80_A15) = address >> 15;
        }
#else
        GPIOC_PDOR = (GPIOC_PDOR & ~0xFFF) | (address & 0xFFF);
        GPIOB_PDOR = (GPIOB_PDOR & ~0xF)   | (address >> 12);
#endif
#else
        digitalWrite(Z80_A0, address & (1<<0) ? 1 : 0);
        digitalWrite(Z80_A1, address & (1<<1) ? 1 : 0);
        digitalWrite(Z80_A2, address & (1<<2) ? 1 : 0);
        digitalWrite(Z80_A3, address & (1<<3) ? 1 : 0);
        digitalWrite(Z80_A4, address & (1<<4) ? 1 : 0);
        digitalWrite(Z80_A5, address & (1<<5) ? 1 : 0);
        digitalWrite(Z80_A6, address & (1<<6) ? 1 : 0);
        digitalWrite(Z80_A7, address & (1<<7) ? 1 : 0);
        if(!(address&0xff)){ // skip updating this half
            digitalWrite(Z80_A8,  address & (1<<8)  ? 1 : 0);
            digitalWrite(Z80_A9,  address & (1<<9)  ? 1 : 0);
            digitalWrite(Z80_A10, address & (1<<10) ? 1 : 0);
            digitalWrite(Z80_A11, address & (1<<11) ? 1 : 0);
            digitalWrite(Z80_A12, address & (1<<12) ? 1 : 0);
            digitalWrite(Z80_A13, address & (1<<13) ? 1 : 0);
            digitalWrite(Z80_A14, address & (1<<14) ? 1 : 0);
            digitalWrite(Z80_A15, address & (1<<15) ? 1 : 0);
        }
#endif
        count--;
    }
#ifdef KINETISK
    *portOutputRegister(Z80_RD) = 1;
    *portOutputRegister(Z80_MREQ) = 1;
#else
    digitalWrite(Z80_RD, 1);
    digitalWrite(Z80_MREQ, 1);
#endif
    z80_setup_drive_data(0);
}

uint8_t z80_memory_read(uint16_t address)
{
    uint8_t byte;
    z80_setup_address(address);
    z80_shutdown_drive_data();
#ifdef KINETISK
    *portOutputRegister(Z80_MREQ) = 0;
    *portOutputRegister(Z80_RD) = 0;
    delayMicroseconds(1);
    byte = z80_bus_data();
    *portOutputRegister(Z80_RD) = 1;
    *portOutputRegister(Z80_MREQ) = 1;
#else
    digitalWrite(Z80_MREQ, 0);
    digitalWrite(Z80_RD, 0);
    byte = z80_bus_data();
    digitalWrite(Z80_RD, 1);
    digitalWrite(Z80_MREQ, 1);
#endif
    z80_setup_drive_data(0);
    return byte;
}

// Z80 CPU conducts a write to memory
uint8_t z80_enchanted_cpu_read(uint16_t *address=NULL)
{
    uint8_t data;

    // wait for write cycle to begin
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();

    // capture the cycle
    if(address)
        *address = z80_bus_address();
    data = z80_bus_data();

    // wait for write cycle to complete
    do{
        z80_clock_pulse();
    }while( z80_wr_asserted() );

    // return value written
    return data;
}

// Z80 CPU conducts a read to memory
uint16_t z80_enchanted_cpu_write(uint8_t data)
{
    uint16_t address;

    // wait for read cycle to begin
    while(!(z80_mreq_asserted() && z80_rd_asserted()))
        z80_clock_pulse();

    // capture address
    address = z80_bus_address();

    // put the byte on the data bus
    z80_setup_drive_data(data);

    // wait for the read cycle to complete
    do{
        z80_clock_pulse();
    }while( z80_rd_asserted() );

    // release the data bus
    z80_shutdown_drive_data();

    // return address read
    return address;
}

uint16_t z80_enchanted_cpu_read16(uint16_t *address = NULL)
{
    return (z80_enchanted_cpu_read(address) << 8) | z80_enchanted_cpu_read();
}

uint16_t z80_enchanted_cpu_write16(uint16_t value)
{
    uint16_t r;
    r = z80_enchanted_cpu_write(value);
    z80_enchanted_cpu_write(value >> 8);
    return r;
}


int  enchanted_z80_bus_trace_stash;
bool enchanted_ram_ce_stash;

// This turns off the Z80 clock and RAM, we will provide
// both of these. Used when we want to force the Z80 to
// instructions we synthesise, while we capture the results.
void z80_enchant_cpu(void)
{
    z80_clk_pause(true);

    // turn off bus tracing before we blow its tiny little mind
    enchanted_z80_bus_trace_stash = z80_bus_trace;
    z80_bus_trace = 0;

    // disable the RAM so we can control the data bus
    enchanted_ram_ce_stash = ram_ce;
    ram_ce = false;
    shift_register_update();
}

// Turn back on the Z80 clock and RAM
void z80_disenchant_cpu(void)
{
    ram_ce = enchanted_ram_ce_stash;               // enable SRAM (if it was enabled before)
    shift_register_update();
    z80_bus_trace = enchanted_z80_bus_trace_stash; // and tracing
    z80_clk_resume();                              // fire up clk
}

void z80_set_pc(uint16_t address)
{
    z80_enchant_cpu();

    z80_enchanted_cpu_write(0xC3);                  // JP xxxx
    z80_enchanted_cpu_write(address & 0xFF);        //  ...
    z80_enchanted_cpu_write(address >> 8);          //  ...

    z80_disenchant_cpu();
}

uint16_t z80_set_register_swap_registers(z80_register_t reg)
{
    switch(reg){
        case Z80_REG_AF_ALT:
            return z80_enchanted_cpu_write(0x08);   // EX AF, AF'
        case Z80_REG_BC_ALT:
        case Z80_REG_DE_ALT:
        case Z80_REG_HL_ALT:
            return z80_enchanted_cpu_write(0xD9);   // EXX
        default:
            return 0;
    }
}

void z80_set_register(z80_register_t reg, uint16_t value)
{
    uint16_t orig_pc, orig_sp, orig_af;

    // Sequence of instructions for loading each register:
    //
    //     PC:                                                        JP xx xx
    //     SP:                                          LD SP, xx xx; JP xx xx
    //     AF:              POP AF <xx xx>;             LD SP, xx xx; JP xx xx
    //     BC:              LD BC, xx xx;                             JP xx xx
    //     DE:              LD DE, xx xx;                             JP xx xx
    //     HL:              LD HL, xx xx;                             JP xx xx
    //     IX:              LD IX, xx xx;                             JP xx xx
    //     IY:              LD IY, xx xx;                             JP xx xx
    // AF_ALT:  EX AF, AF'; POP AF <xx xx>; EX AF, AF'; LD SP, xx xx; JP xx xx
    // BC_ALT:  EXX;        LD BC, xx xx;   EXX;                      JP xx xx
    // DE_ALT:  EXX;        LD DE, xx xx;   EXX;                      JP xx xx
    // HL_ALT:  EXX;        LD HL, xx xx;   EXX;                      JP xx xx
    //      I:  PUSH AF <xx xx>; LD A, xx; LD I, a; POP AF <xx xx>; LD SP, xx xx; JP xx xx
    z80_enchant_cpu();

    switch(reg){
        case Z80_REG_I:
            orig_pc = z80_enchanted_cpu_write(0xF5);  // PUSH AF
            orig_af = z80_enchanted_cpu_read16(&orig_sp);
            z80_enchanted_cpu_write(0x3E);  // LD A, xx
            z80_enchanted_cpu_write(value); // new value for I
            z80_enchanted_cpu_write(0xED);  // LD I, A
            z80_enchanted_cpu_write(0x47);  // (2-byte instruction)
            z80_enchanted_cpu_write(0xF1);  // POP AF
            z80_enchanted_cpu_write16(orig_af);
            break;
        case Z80_REG_PC:
            orig_pc = value;
            break;
        case Z80_REG_SP:
            orig_pc = z80_enchanted_cpu_write(0x31);     // LD SP
            break;
        case Z80_REG_AF:
            orig_pc = z80_enchanted_cpu_write(0xF1);     // POP AF -- can't do LD AF
            break;
        case Z80_REG_BC:
            orig_pc = z80_enchanted_cpu_write(0x01);     // LD BC
            break;
        case Z80_REG_DE:
            orig_pc = z80_enchanted_cpu_write(0x11);     // LD DE
            break;
        case Z80_REG_HL:
            orig_pc = z80_enchanted_cpu_write(0x21);     // LD HL
            break;
        case Z80_REG_AF_ALT:
            orig_pc = z80_set_register_swap_registers(reg);
            z80_enchanted_cpu_write(0xF1);               // POP AF -- can't do LD AF
            break;
        case Z80_REG_BC_ALT:
            orig_pc = z80_set_register_swap_registers(reg);
            z80_enchanted_cpu_write(0x01);               // LD BC
            break;
        case Z80_REG_DE_ALT:
            orig_pc = z80_set_register_swap_registers(reg);
            z80_enchanted_cpu_write(0x11);               // LD DE
            break;
        case Z80_REG_HL_ALT:
            orig_pc = z80_set_register_swap_registers(reg);
            z80_enchanted_cpu_write(0x21);               // LD HL
            break;
        case Z80_REG_IX:
            orig_pc = z80_enchanted_cpu_write(0xDD);     // ...
            z80_enchanted_cpu_write(0xE1);               // POP IX
            break;
        case Z80_REG_IY:
            orig_pc = z80_enchanted_cpu_write(0xFD);     // ...
            z80_enchanted_cpu_write(0xE1);               // POP IY
            break;
    }

    // where required, feed in the data value (and capture SP for POP instructions)
    switch(reg){
        case Z80_REG_I:
        case Z80_REG_PC:
            break;
        default:
            orig_sp = z80_enchanted_cpu_write16(value);
            break;
    }

    // where required, restore SP
    switch(reg){
        case Z80_REG_AF:
        case Z80_REG_AF_ALT:
        case Z80_REG_IX:
        case Z80_REG_IY:
        case Z80_REG_I:
            z80_enchanted_cpu_write(0x31);      // LD SP, xxxx
            z80_enchanted_cpu_write16(orig_sp + 1);
            break;
        default:
            break;
    }

    // switch register set back (NOP if not required)
    z80_set_register_swap_registers(reg);

    // pick up in real memory back where we started
    z80_enchanted_cpu_write(0xC3);              // JP, xxxx
    z80_enchanted_cpu_write16(orig_pc);

    z80_disenchant_cpu();
}

void z80_show_regs(void)
{
    uint16_t pc, sp, af, bc, de, hl, ix, iy, af_, bc_, de_, hl_;
    uint8_t i;

    // this code does not deal with the situation where CPU is HALTed.
    // solution might be: wake CPU with an int/nmi, capture PC when it writes it to the
    //                    stack. capture regs as usual. then IRET and JP to PC-1, then
    //                    feed it a HALT when it fetches PC-1. it will HALT with PC correct.

    // take over the CPU
    z80_enchant_cpu();

    // we feed it an F5 (PUSH AF) instruction
    pc = z80_enchanted_cpu_write(0xF5); // PUSH AF; capture PC

    af = z80_enchanted_cpu_read16(&sp); // capture both SP and AF
    sp += 1;                            // adjust SP to previous value

    z80_enchanted_cpu_write(0xC5);      // PUSH BC
    bc = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xD5);      // PUSH DE
    de = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xE5);      // PUSH HL
    hl = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0x08);      // EX AF, AF'
    z80_enchanted_cpu_write(0xF5);      // PUSH AF
    af_ = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0x08);      // EX AF, AF' again (swap back)
    z80_enchanted_cpu_write(0xD9);      // EXX
    z80_enchanted_cpu_write(0xC5);      // PUSH BC
    bc_ = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xD5);      // PUSH DE
    de_ = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xE5);      // PUSH HL
    hl_ = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xD9);      // EXX again (swap back)
    z80_enchanted_cpu_write(0xDD);      // IX prefix
    z80_enchanted_cpu_write(0xE5);      // PUSH IX
    ix = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xFD);      // IY prefix
    z80_enchanted_cpu_write(0xE5);      // PUSH IY
    iy = z80_enchanted_cpu_read16();
    z80_enchanted_cpu_write(0xED);      // ED prefix
    z80_enchanted_cpu_write(0x57);      // LD A,I - note this affects the flags register

    z80_enchanted_cpu_write(0xF5);      // PUSH AF
    i = z80_enchanted_cpu_read16()>>8;  // I was copied into A (top bits)

    // finally we need to put AF, SP and PC back as they were before our tinkering
    z80_enchanted_cpu_write(0xF1);      // POP af
    z80_enchanted_cpu_write16(af);
    z80_enchanted_cpu_write(0x31);      // LD SP, xxxx
    z80_enchanted_cpu_write16(sp);
    z80_enchanted_cpu_write(0xC3);      // JP xxxx
    z80_enchanted_cpu_write16(pc);

    // release the CPU
    z80_disenchant_cpu();

    report("PC=%04x SP=%04x\r\nAF=%04x AF'=%04x\r\n" \
           "BC=%04x BC'=%04x\r\nDE=%04x DE'=%04x\r\n" \
           "HL=%04x HL'=%04x\r\nIX=%04x IY=%04x I=%02x\r\n",
           pc, sp, af, af_,
           bc, bc_, de, de_,
           hl, hl_, ix, iy, i);
}

inline void z80_complete_read(uint8_t data)
{
    z80_setup_drive_data(data);
    z80_set_busrq(true);
    z80_set_release_wait(true);
    while(!z80_busack_asserted())
        if(!z80_clk_is_independent())
            z80_clock_pulse();
    z80_shutdown_drive_data();
    z80_set_release_wait(false);
    // return with DMA capable -- caller must do z80_set_busrq(false);
}

inline void z80_complete_write(void)
{
    z80_set_busrq(true);
    z80_set_release_wait(true);
    while(!z80_busack_asserted())
        if(!z80_clk_is_independent())
            z80_clock_pulse();
    z80_set_release_wait(false);
    // return with DMA capable -- caller must do z80_set_busrq(false);
}

void handle_z80_bus(void)
{
    if(z80_wait_asserted()){
        if(z80_iorq_asserted()){
            if(z80_rd_asserted()){       // I/O read
                z80_complete_read(iodevice_read(z80_bus_address()));
                z80_set_busrq(false);
            }else if(z80_wr_asserted()){ // I/O write
                z80_complete_write();
                iodevice_write(z80_bus_address_low8(), z80_bus_data());
                z80_set_busrq(false);
            }else if(z80_m1_asserted()){ // Interrupt acknowledge
                z80_complete_read(z80_irq_vector());
                z80_set_busrq(false);
            }else
                report("(iorq weird?)");
        } else if(z80_mreq_asserted()){
            if(z80_rd_asserted()){       // Memory read
                z80_complete_read(memory_read(z80_bus_address()));
                z80_set_busrq(false);
            }else if(z80_wr_asserted()){ // Memory write
                z80_complete_write();
                memory_write(z80_bus_address_low8(), z80_bus_data());
                z80_set_busrq(false);
            }else
                report("(mreq weird?)");
        } else
            report("(wait weird?)");
    }
}

void z80_mmu_switch_context_local(void)
{
    for(int b=0; b<4; b++)
        z80_mmu_write(b, mmu[b]);
}

void z80_mmu_switch_context_foreign(void)
{
    for(int b=0; b<4; b++)
        z80_mmu_write(b, mmu_foreign[b]);
}

void z80_set_mmu(int bank, uint8_t page) // call only in DMA mode
{
    if(bank < 0 || bank > 3){
        report("z80_set_mmu: bad bank %d!\r\n", bank);
        return;
    }
    if(mmu[bank] == page)
        return;
    mmu[bank] = page;
    z80_mmu_write(bank, page);
}

void begin_dma(void)
{
    z80_set_busrq(true);
    while(!z80_busack_asserted()){  // wait for BUSACK
        if(!z80_clk_is_independent())
            z80_clock_pulse();
        handle_z80_bus();
        z80_set_busrq(true); // handle_z80_bus may perform a DMA cycle so we need to assert again
    }
    z80_bus_master();
}

void end_dma(void)
{
    z80_bus_slave();
    z80_set_busrq(false);
}

void mmu_setup(void)
{
    begin_dma();

    for(int i=0; i<4; i++){
        mmu[i] = 0xaa; // force an update
        z80_set_mmu(i, i);
    }

    end_dma();
}

void sram_setup(void)
{
    int i;

    begin_dma();

    // stash current state
    bool old_ram_ce = ram_ce;
    uint8_t old_mmu = mmu[0];

    // enable the SRAM
    ram_ce = true;
    shift_register_update();

    // wipe RAM
    report("ersatz80: wipe RAM: page ___");
    for(i=0; i<256; i++){
        report("\x08\x08\x08%03d", i);
        z80_set_mmu(0, i);
        // is it really there?
        z80_memory_write(0, 0xaa);
        z80_memory_write(1, 0x55);
        z80_memory_write(2, 0x00);
        z80_memory_write(3, i);
        z80_memory_write(16383, i);
        z80_memory_write(16382, 0x00);
        z80_memory_write(16381, 0x55);
        z80_memory_write(16380, 0xaa);
        if(z80_memory_read(0) != 0xaa ||
           z80_memory_read(1) != 0x55 ||
           z80_memory_read(2) != 0x00 ||
           z80_memory_read(3) != i ||
           z80_memory_read(16380) != 0xaa ||
           z80_memory_read(16381) != 0x55 ||
           z80_memory_read(16382) != 0x00 ||
           z80_memory_read(16383) != i)
            break;
        // yup, it seems to be there!
        z80_wipe_page();
    }
    ram_pages = i;
    report("\x08\x08\x08\x08\x08\x08\x08\x08%d pages (%dKB)\r\n", ram_pages, 16*ram_pages);

    // return machine to previous state
    ram_ce = old_ram_ce;
    shift_register_update();
    z80_set_mmu(0, old_mmu);
    end_dma();
}

void load_program_to_sram(const uint8_t *program, uint16_t address, uint16_t length, uint16_t start_address)
{
    begin_dma();

    // stash current state
    bool old_ram_ce = ram_ce;

    // enable the SRAM
    ram_ce = true;
    shift_register_update();

    if(start_address != 0x0000){
        z80_memory_write(0, 0xc3); // JP instruction
        z80_memory_write(1, start_address & 0xFF);
        z80_memory_write(2, start_address >> 8);
    }

    z80_memory_write_block(address, program, length);

    // restore machine state
    ram_ce = old_ram_ce;
    shift_register_update();
    end_dma();
}

#define LOAD_BUFFER_SIZE 512
int load_file_to_sram(char *filename, uint16_t address)
{
    SdBaseFile file;
    uint8_t buffer[LOAD_BUFFER_SIZE];
    bool old_ram_ce;
    int r, total;

    // Open file
    if(!file.open(&sdcard, filename, O_RDONLY))
        return -1;

    // prepare the machine
    begin_dma();
    old_ram_ce = ram_ce;
    ram_ce = true;
    shift_register_update();

    // Load the file block by block
    total = 0;
    while(true){
        r = file.read(buffer, LOAD_BUFFER_SIZE);
        if(r <= 0)
            break;
        z80_memory_write_block(address, buffer, r);
        address += r;
        total += r;
    }

    // restore machine state
    ram_ce = old_ram_ce;
    shift_register_update();
    end_dma();
    file.close();

    return total;
}

enum bus_cycle_t { MEM_READ, MEM_WRITE, IO_READ, IO_WRITE, NO_CYCLE };

typedef struct {
    bus_cycle_t cycle;
    uint16_t address;
    uint8_t data;
} bus_trace_t;

#define MAX_BUS_STATES 8 // presumably this can be lower??
bus_trace_t bus_trace[MAX_BUS_STATES];
bool bus_trace_wait_cycle_end = false;
int instruction_clock_cycles = 0;
int bus_trace_count = 0;
int bus_trace_di = 0;

uint8_t read_bus_trace_bytes(void)
{
    return bus_trace[bus_trace_di++].data;
}

void z80_instruction_ended(void)
{
    char output[20];
    uint16_t addr;
    bus_cycle_t cycle;

    bus_trace_di = 0;
    z80ctrl_disasm(read_bus_trace_bytes, output);
    report("%-13s %2d  ", output, instruction_clock_cycles);

    addr = ~bus_trace[0].address;
    for(int i=0; i<bus_trace_count; i++){
        if(bus_trace[i].address != addr || cycle != bus_trace[i].cycle){
            addr = bus_trace[i].address;
            cycle = bus_trace[i].cycle;
            switch(bus_trace[i].cycle){
                case MEM_READ:  report("%04x: ", bus_trace[i].address); break;
                case MEM_WRITE: report("%04x<-", bus_trace[i].address); break;
                case IO_READ:   report("io%04x: ", bus_trace[i].address); break;
                case IO_WRITE:  report("io%04x<-", bus_trace[i].address); break;
                case NO_CYCLE:  break; // should never happen
            }
        }
        report("%02x%s", bus_trace[i].data, i == (bus_trace_count-1) ? "" : (i == (bus_trace_di-1) ? " / " : " "));
        addr++;
    }
    report("\r\n");
    instruction_clock_cycles = 0;
}

void z80_bus_trace_state(void)
{
    bus_cycle_t type = NO_CYCLE;

    if(z80_bus_trace >= 1){
        instruction_clock_cycles++;
        if(bus_trace_wait_cycle_end && !(z80_mreq_asserted() || z80_iorq_asserted()))
            bus_trace_wait_cycle_end = false;
        else if(!bus_trace_wait_cycle_end && !z80_wait_asserted()){
            if(z80_mreq_asserted()){
                if(z80_rd_asserted()){
                    type = MEM_READ;
                }else if(z80_wr_asserted()){
                    type = MEM_WRITE;
                }
            }else if(z80_iorq_asserted()){
                if(z80_rd_asserted()){
                    type = IO_READ;
                }else if(z80_wr_asserted()){
                    type = IO_WRITE;
                }
            }
            if(type != NO_CYCLE){
                if(z80_m1_asserted() && !(bus_trace_count == 1 && (bus_trace[0].data == 0xcb || bus_trace[0].data == 0xdd || bus_trace[0].data == 0xed || bus_trace[0].data == 0xfd))){
                    z80_instruction_ended();
                    bus_trace_count = 0;
                }
                bus_trace_wait_cycle_end = true;
                bus_trace[bus_trace_count].cycle = type;
                bus_trace[bus_trace_count].data = z80_bus_data();
                bus_trace[bus_trace_count].address = z80_bus_address();
                if(bus_trace_count < (MAX_BUS_STATES-1))
                    bus_trace_count++;
            }
        }
    }

    if(z80_bus_trace >= 2)
        report("\r\n|%04x|%02x|%s|%s|%s|",
                z80_bus_address(), z80_bus_data(),
                z80_mreq_asserted() ? "MREQ" : (z80_iorq_asserted() ? "IORQ" : "    "),
                z80_rd_asserted() ? "RD" : (z80_wr_asserted() ? "WR" : "  "),
                z80_m1_asserted() ? "M1" : "  ");
}
