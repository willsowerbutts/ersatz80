# ersatz80

ersatz80 is a Z80 microcomputer system using an ARM microcontroller as a
peripheral and supervisor/debugger. The system has memory banking hardware 
allowing the Z80 to access 1MB of SRAM. The Z80 clock speed is variable and 
can be up to 20MHz. The ARM provides several peripherals to the Z80 including 
a timer, interrupt controller, UART and disk controller with DMA support. The 
disk controller uses disk images in a FAT filesystem stored on a microSD 
card.

## Brief instructions

Assemble and populate the PCB. Program the Teensy with the firmware. Insert a 
FAT formatted microSD card (optional). Connect the Teensy's USB port to a 
computer and connect to the USB ACM serial device with your terminal 
software. The serial input is normally sent to the Z80 UART device however 
pressing the escape keycode (by default Ctrl+G) will wake up the supervisor 
which will read and execute a command. The "help" command should give some 
clues as to what commands are available.

## PCB

Please contact me by email (will@sowerbutts.com) if you would like me to post 
you a bare PCB or if you would like Gerber files to fabricate your own PCBs.  
The PCB measures just under 100x100mm and so qualifies for several of the 
cheaper "prototype" fabrication services.

Bill of materials for PCB:
 - Teensy 3.5 (do not substitute 3.6)
 - 16 x 0.1uF MLCC capacitors
 - Z84C0020 (Z80 CPU, DIP, CMOS, do not attempt to use NMOS parts)
 - 2 x AS6C4008
 - 2 x 74AHCT00
 - 2 x 74AHCT595
 - 2 x 74HC670 (not available in AHCT?)
 - 74AHCT139
 - 74AHCT04
 - 74AHCT02
 - 74AHCT74
 - 20MHz full- or half-can oscillator
 - 5mm green LED (D1)
 - 2 x 10 LED bar graphs (alternatively 1 x 20)
 - 3 x 1K 8 bussed resistor network (9 pins)
 - Sockets for all ICs
 - 2 x 24-way 1-row 2.54mm receptacle (eg Multicomp 2212S-24SG-85, Farnell 1593472)
 - 2 x 24-way 1-row 2.54mm pin header (eg Multicomp MC34743, Farnell 1593425)
 - Optional: 2-way 2.54mm pin header for reset (J3)
 - Optional: 2-way 2.54mm pin header for 3V coin cell (J4)
 - Optional: 2-way 2.54mm pin header to connect USB power to 5V input (J5), recommended to fit a wire jumper here instead.
 - Optional: 2-pin power socket to supply external 5V power (J2), recommended to omit this and USB power instead.

If you want to use the expansion connector you will additionally need headers
and sockets for the extra I/O pins on the underside of the Teensy 3.5.  Solder
the three surface mount headers onto the Teensy and check their alignment
carefully before soldering on the two sets of 24-way pins along the outer
edges. The expansion connector is entirely optional and the board will function
if you fit only the two 24-way through-hole connectors to the Teensy.

 - 10-way 2-row 2.54mm receptacle (eg Multicomp 2214S-10SG-85, Farnell 1593490)
 - 8-way 2-row 2.54mm receptacle (eg Multicomp 2214S-08SG-85, Farnell 1593489 or Wurth 61300821821, Farnell 2827896)
 - 6-way 2-row 2.54mm receptacle (eg Multicomp 2214S-06SG-85, Farnell 1593488)
 - 10-way 2-row 2.54mm SMT pin header (eg Multicomp 2213SM-10G-TB, Farnell 2847232)
 - 8-way 2-row 2.54mm SMT pin header (eg Multicomp 2213SM-08G-TB, Farnell 2847231)
 - 6-way 2-row 2.54mm SMT pin header (eg Multicomp 2213SM-06G-TB, Farnell 2847230)
 - 5-way 1-row 2.54mm receptacle
 - 5-way 1-row 2.54mm pin header (through hole)

Power: I recommend omitting the 5V power socket at J2 and fitting a wire jumper
at J5. This will power the board from the USB connector on the Teensy 3.5. In
principle other power supply arrangements are possible but these have not been
tested, and you will want to connect to the USB port in any event so you can
access the console!

The revision 1 PCB is a bit packed and you may wish to omit the two capacitors
between U7/U11 and U8/U15 to ensure a clean fit. This issue does not affect
the revision 2 PCB.

## PCB Revisions

For revision 1 PCBs you must uncomment "#define ERSATZ80_PCB_REV1" in z80.h

For revision 2 PCBs you must comment out "#define ERSATZ80_PCB_REV1" in z80.h

## Overclocking

It is possible to fit faster oscillators at U10 and overclock the Z80 CPU.
Using a Zilog Z84C0020PEG CMOS Z80 CPU it has been determined that oscillators
up to 22MHz (10% overclock) give reliable operation, while 24MHz was too fast.

## Supervisor Commands

To send a command to the supervisor press the escape keycode (default Ctrl+G)
and then type your command followed by Enter. To abort entering a command just
press the escape keycode again, or enter the commands "quit" or "exit".

### Supervisor Command: `help`

This just prints a list of the commands the supervisor knows. Useful if you
forget the name of a command.

### Supervisor Command: `reset`

Reset the Z80 CPU. The Z80 **/RESET** line is asserted for 10 clock cycles.
After this the Z80 will start executing code from address 0. Note that only the
Z80 CPU is reset, the Teensy virtual peripherals are not reset.

### Supervisor Command: `regs`

Prints the contents of the Z80 CPU registers.

### Supervisor Commands: `run`

The `run` command forces the Z80 execution to jump to the given address. The
address is always specified in hex, the `0x` prefix is optional.

 * `run 0` -- jump to address 0
 * `run 150` -- jump to address 0x0150
 * `run 0x8000` -- jump to address 0x8000

### Supervisor Command: `clk`

The `clk` command reports or changes the frequency of the clock fed to the Z80 CPU.

The CPU can be clocked from either a crystal oscillator (fitted at U10 on the
PCB, typically 20MHz), or from a clock signal generated by the Teensy.

The Teensy generates a clock signal by dividing a 30MHz clock by an integer
amount, so the possible frequencies are 15MHz, 10MHz, 7.5MHz, 6Mhz, 5MHz,
4.29MHz, 3.75MHz, 3.33MHz, 3MHz, 2.73MHz, 2.5MHz, 2.31MHz, 2.14MHz, 2MHz, etc.
Very slow frequencies (below 1Hz) are supported.

 * `clk` -- report current clock source and frequency
 * `clk stop` -- stop the Z80 clock entirely
 * `clk fast` -- use the fast crystal oscillator (20MHz)
 * `clk <frequency>` -- use the specified frequency
 * `clk 5M` -- use a 5MHz clock
 * `clk 100K` -- use a 100KHz clock
 * `clk 500` -- use a 500Hz clock

TODO: loadrom loadfile trace ls mv rm cp disk mount unmount format sync in out exec
TODO: document virtual peripherals: MMU, timer, interrupt controller, disk controller
