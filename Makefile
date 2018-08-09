CC=avr-gcc
OBJ2HEX=avr-objcopy
AVRDUDE=avrdude

CPU_TYPE=atmega2560
CPU_FREQ=16000000
PROG_DEV=/dev/ttyUSB0
PROG_BAUD=115200

CCFLAGS=-DDEBUG
CCFLAGS+=-Wall -Werror -W -Wno-unused-parameter -Wno-sign-compare -Wno-char-subscripts -g -Os -std=gnu99 -fdata-sections -ffunction-sections -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -mcall-prologues -fshort-enums -fno-strict-aliasing

FIRMWARE_OBJS=main.o serial.o debug.o version.o

all:	firmware.bin firmware.hex

firmware.elf:	$(FIRMWARE_OBJS)
	$(CC) -DF_CPU=$(CPU_FREQ)UL -mmcu=$(CPU_TYPE) -Wl,--gc-sections,--relax $(FIRMWARE_OBJS) -lm -o $@ 
	./memory-usage $@ $(CPU_TYPE)
	rm -f version.c

version.c:
	./makeversion

%.o:	%.S
	$(CC) -DF_CPU=$(CPU_FREQ)UL -mmcu=$(CPU_TYPE) $(CCFLAGS) -c $< -lm -o $@

%.o:	%.c
	$(CC) -DF_CPU=$(CPU_FREQ)UL -mmcu=$(CPU_TYPE) $(CCFLAGS) -c $< -lm -o $@

%.hex:	%.elf
	$(OBJ2HEX) -O ihex -R .eeprom $< $@

%.bin:	%.elf
	avr-objcopy -O binary $< $@

clean:
	rm -f *.hex *.o *.elf *.bin version.c

program:	firmware.hex
	$(AVRDUDE) -p $(CPU_TYPE) -c arduino -P $(PROG_DEV) -b $(PROG_BAUD) -V -U firmware.hex
	picocom -b 115200 $(PROG_DEV)
