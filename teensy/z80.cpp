#include <Arduino.h>
#include "rom.h"
#include "debug.h"
#include "z80.h"

uint16_t user_led = 0x000; // value to show on user-controlled LEDs
bool z80_reset = true;       // Z80 /RESET pin (z80_reset=true means /RESET is driven low ie asserted)
bool z80_irq = false;        // Z80 /IRQ pin
bool z80_nmi = false;        // Z80 /NMI pin
bool ram_ce = false;         // RAM /CE pin
bool z80_bus_trace = false;
uint8_t mmu[4];
uint8_t ram_pages = 0;

void shift_register_update(void)
{
    int output = (user_led & 0xFFF) |       // the LEDs are all active high
                 (z80_reset ? 0 : 0x8000) | // these signals are all active low
                 (z80_nmi   ? 0 : 0x4000) |
                 (z80_irq   ? 0 : 0x2000) |
                 (ram_ce    ? 0 : 0x1000);

#ifdef KINETISK
    // this can be made slightly faster by using the GPIO?_PSOR/PCOR 
    // registers ... but then it's too fast for the 74AHCT595s!
    for(int i=0; i<16; i++){
        *portOutputRegister(SHIFT_REGISTER_DATA) = output;
        *portOutputRegister(SHIFT_REGISTER_CLK) = 1;
        output >>= 1;
        *portOutputRegister(SHIFT_REGISTER_CLK) = 0;
    }
    *portOutputRegister(SHIFT_REGISTER_LATCH) = 1;
    *portOutputRegister(SHIFT_REGISTER_LATCH) = 0;
#else
    for(int i=0; i<16; i++){
        digitalWrite(SHIFT_REGISTER_DATA, output & 1);
        digitalWrite(SHIFT_REGISTER_CLK, 1);
        output >>= 1;
        digitalWrite(SHIFT_REGISTER_CLK, 0);
    }
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
    GPIOA_PDDR |= ((1<<5) | (1<<12) | (1<<14) | (1<<15) | (1<<16) | (1<<17));
    GPIOB_PDDR |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) |
                   (1<<16) | (1<<17) | (1<<18) | (1<<19));
    GPIOC_PDDR |= ((1<<1) | (1<<2) | (1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOD_PDDR |= ((1<<0) | (1<<5) | (1<<6));
    GPIOE_PDDR |= ((1<<24) | (1<<25) | (1<<26));
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
    GPIOA_PDDR &=~((1<<5) | (1<<12) | (1<<14) | (1<<15) | (1<<16) | (1<<17));
    GPIOB_PDDR &=~((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) |
                   (1<<16) | (1<<17) | (1<<18) | (1<<19));
    GPIOC_PDDR &=~((1<<1) | (1<<2) | (1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOD_PDDR &=~((1<<0) | (1<<5) | (1<<6));
    GPIOE_PDDR &=~((1<<24) | (1<<25) | (1<<26));
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
    GPIOA_PSOR = ((1<<5) | (1<<12) | (1<<14) | (1<<15) | (1<<16) | (1<<17));
    GPIOB_PSOR = ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<10) | (1<<11) |
                  (1<<16) | (1<<17) | (1<<18) | (1<<19));
    GPIOC_PSOR = ((1<<1) | (1<<2) | (1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOD_PSOR = ((1<<0) | (1<<5) | (1<<6));
    GPIOE_PSOR = ((1<<24) | (1<<25) | (1<<26));
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

void z80_bus_report_state(void)
{
    report("\r\n|%04x|%02x|%s|%s|%s|",
            z80_bus_address(), z80_bus_data(), 
            z80_mreq_asserted() ? "MREQ" : (z80_iorq_asserted() ? "IORQ" : "    "),
            z80_rd_asserted() ? "RD" : (z80_wr_asserted() ? "WR" : "  "),
            z80_m1_asserted() ? "M1" : "  ");
}

uint16_t z80_bus_address(void)
{
#ifdef KINETISK
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
    GPIOA_PDDR &= ~((1<<17));
    GPIOB_PDDR &= ~((1<<11));
    GPIOC_PDDR &= ~((1<<8) | (1<<9) | (1<<10) | (1<<11));
    GPIOE_PDDR &= ~((1<<24) | (1<<25));
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
    // report("Z80 CPU Reset\r\n");
    z80_set_reset(true);
    z80_set_release_wait(true);
    for(int i=0; i<10; i++){ // Z80 requires at least 3 clocks to fully reset
        z80_set_clk(false);
        z80_set_clk(true);
    }
    z80_set_release_wait(false);
    z80_set_reset(false);
}

void z80_setup_address(uint16_t address)
{
#ifdef KINETISK
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

void z80_mmu_write(uint16_t address, uint8_t data)
{
    /* assert(DMA_MODE / WR_ADDR | WR_DATA / ...)  ? to detect being called in wrong mode */
    z80_setup_address_data(address, data);
#ifdef KINETISK
    *portOutputRegister(MMU_EW) = 0;
    *portOutputRegister(MMU_EW) = 1;
#else
    digitalWrite(MMU_EW, 0);
    delayMicroseconds(1);
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

uint8_t z80_memory_read(uint16_t address)
{
    uint8_t byte;
    z80_setup_address(address);
    z80_shutdown_drive_data();
#ifdef KINETISK
    *portOutputRegister(Z80_MREQ) = 0;
    *portOutputRegister(Z80_RD) = 0;
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

void z80_set_mmu(int bank, uint8_t page) // call only in DMA mode
{
    if(bank < 0 || bank > 3){
        report("z80_set_mmu: bad bank %d!\r\n", bank);
        return;
    }
    if(mmu[bank] == page)
        return;
    mmu[bank] = page;
    z80_mmu_write(bank << 14, page);
}

void begin_dma(void)
{
    // this is incomplete. only really useful when Z80 is held in reset.
    z80_set_busrq(true);
    while(!z80_busack_asserted()){  // wait for BUSACK
        // TODO -- check /WAIT etc in here
        if(!z80_clk_running())
            z80_clock_pulse();
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
    report("Wipe RAM: page ___");
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
    
    for(int i=0; i<length; i++)
        z80_memory_write(address++, program[i]);

    // restore machine state
    ram_ce = old_ram_ce;
    shift_register_update();
    end_dma();
}
