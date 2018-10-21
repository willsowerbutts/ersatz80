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
between U7/U11 and U8/U15 to ensure a clean fit.
