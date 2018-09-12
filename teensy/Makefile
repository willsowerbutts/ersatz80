BOARD_TAG    = teensy35
ARDUINO_DIR  = $(HOME)/arduino-1.8.5
ARDUINO_LIBS =

# https://github.com/sudar/Arduino-Makefile -- on Debian/Ubuntu install the "arduino-mk" package
include /usr/share/arduino/Teensy.mk

all:	$(TARGET_HEX)
		~/arduino-1.8.5/teensy_loader_cli --mcu=mk64fx512 -v -s ./build-teensy35/teensy.hex
		sleep 1
		picocom -b 9600 -f n /dev/ttyACM0
