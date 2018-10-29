BOARD_TAG    = teensy35
F_CPU        = 120000000
ARDUINO_DIR  = $(HOME)/arduino-1.8.7
ARDUINO_LIB_PATH = $(ARDUINO_DIR)/hardware/teensy/avr/libraries
ARDUINO_LIBS = SPI SdFat
# Compile with -O1 and link time optimisations
OPTIMIZATION_LEVEL = 1 -flto -fno-fat-lto-objects

# https://github.com/sudar/Arduino-Makefile -- on Debian/Ubuntu install the "arduino-mk" package
include /usr/share/arduino/Teensy.mk

rom.cpp:	monitor.asm
		z80asm --list=monitor.lst -o monitor.bin monitor.asm
		./makerom rom.cpp monitor.bin


all:	rom.cpp $(TARGET_HEX)
		$(ARDUINO_DIR)/teensy_loader_cli --mcu=mk64fx512 -v -s ./build-teensy35/ersatz80.hex
		sleep 1
		picocom -b 9600 -f n /dev/ttyACM0
