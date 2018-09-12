# ersatz80

## Overview

ersatz80 is a Z80 controlled by an AVR. In this prototype the AVR provides all
resources to the Z80 including RAM, ROM, peripherals, and the clock. This is a
prototype for a future PCB that will include SD, SRAM, and allow the Z80 to run
at full speed.

## Hardware

Photos: https://photos.app.goo.gl/xWK76i5JX4hLooR16

You'll need an Arduino Mega2560 board (about $10 on eBay) and a CMOS Z80 (eg
Z84C0020). Note that NMOS Z80s will not work as they are not fully static.

Connect a bit of prototyping board to the 36-pin "DIGITAL" connector on the
Mega2560 board. Then wire up a 40-pin socket for the Z80. Start by placing a
0.1uF ceramic capacitor across pins 11 (+5V) and 29 (GND) to decouple the power
supply. Then connect a wire to each pin on the socket as follows:

| Signal        | Z80 pin       | AVR Mega2560 |
| ------------- | ------------- | ------------ |
| A11           | 1             | PL3 (D46)    |
| A12           | 2             | PL4 (D45)    |
| A13           | 3             | PL5 (D44)    |
| A14           | 4             | PL6 (D43)    |
| A15           | 5             | PL7 (D42)    |
| CLK           | 6             | PG0 (D41)    |
| D4            | 7             | PC4 (D33)    |
| D3            | 8             | PC3 (D34)    |
| D5            | 9             | PC5 (D32)    |
| D6            | 10            | PC6 (D31)    |
| +5V           | 11            | +5V          |
| D2            | 12            | PC2 (D35)    |
| D7            | 13            | PC7 (D30)    |
| D0            | 14            | PC0 (D37)    |
| D1            | 15            | PC1 (D36)    |
| /INT          | 16            | +5V          |
| /NMI          | 17            | +5V          |
| /HALT         | 18            | N/C          |
| /MREQ         | 19            | PB3 (D50)    |
| /IORQ         | 20            | PB1 (D52)    |
| /RD           | 21            | PG1 (D40)    |
| /WR           | 22            | PD7 (D38)    |
| /BUSACK       | 23            | N/C          |
| /WAIT         | 24            | +5V          |
| /BUSREQ       | 25            | +5V          |
| /RESET        | 26            | PG2 (D39)    |
| /M1           | 27            | PB2 (D51)    |
| /RFSH         | 28            | N/C          |
| GND           | 29            | GND          |
| A0            | 30            | PA0 (D22)    |
| A1            | 31            | PA1 (D23)    |
| A2            | 32            | PA2 (D24)    |
| A3            | 33            | PA3 (D25)    |
| A4            | 34            | PA4 (D26)    |
| A5            | 35            | PA5 (D27)    |
| A6            | 36            | PA6 (D28)    |
| A7            | 37            | PA7 (D29)    |
| A8            | 38            | PL0 (D49)    |
| A9            | 39            | PL1 (D48)    |
| A10           | 40            | PL2 (D47)    |

Note N/C = no connection.

I also have a 10K resistor between /RESET and GND just to ensure the Z80 stays
reset until the AVR starts up. This is very likely not required, but
shouldn't hurt (let me know if you test without this).

## Building

To build this you will need make, gcc-avr, avr-libc, avrdude, z80asm and
python. You'll also need a terminal program, I recommend picocom. The Arduino
IDE is not required (or desired!).

I'm assuming you're using Linux. On a Debian or Ubuntu system you should be
able to install all the required software with:

`$ sudo apt-get install make gcc-avr avr-libc avrdude z80asm python picocom`

If you get this to build on Windows, well done you, please let me know how!

To build the software, just run "make program". This will assemble the monitor
ROM, compile the AVR firmware, upload it all to the AVR flash using the serial
bootloader, then run picocom so you can talk to the running system. The Makefile
assumes that the AVR is on /dev/ttyUSB0, just adjust the `PROG_DEV` variable if
you're using another port.

## Running

It's all a bit early for end-user documentation... 

Briefly: 

The AVR provides the Z80 with 4KB of RAM which is mapped into the low 32KB
eight times (so it appears at 0x0000, 0x1000, 0x2000 etc). 

The AVR also provides the Z80 with up to 32KB of ROM which is mapped from
0x8000 upwards. On startup the AVR places a "JP 0x8000" instruction at the
reset vector (0x0000) in Z80 RAM.

The AVR UART is accessible to the Z80 via I/O ports 0 and 1, have a look at
"incharwait" and "outchar" in monitor.asm for the nitty gritty.  No other I/O
devices are provided ... yet. 

Sending a special keypress (currently Ctrl-G) from the PC will cause the AVR to
switch to supervisor mode; the Z80 is halted and the AVR listens for commands
from the PC. The supported supervisor commands are:
 * `trace bus` (switch on tracing of Z80 bus states)
 * `no trace bus` (switch it off again)
 * `regs` (extract and display CPU register contents)
 * `quit` (exit supervisor mode)

All other characters sent from the PC are sent to the Z80 UART.

Any output from the supervisor should be sent to your terminal with the
"bright" attribute set. Output from the Z80 is sent with the "bright" attribute
unset. There's a baked-in assumption that your terminal is VT100 compatible.
