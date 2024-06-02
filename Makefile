ARDUINO_DIR  = $(HOME)/arduino-2.3

all:	build/teensy.avr.teensy35/ersatz80.ino.hex
	$(ARDUINO_DIR)/teensy_loader_cli --mcu=mk64fx512 -v -s build/teensy.avr.teensy35/ersatz80.ino.hex
	sleep 1.2
	picocom -b 9600 -f n /dev/ttyACM0

build/teensy.avr.teensy35/ersatz80.ino.hex:	rom.cpp
	# WRS: you can use "arduino-cli board details --fqbn ..." to check the available options
	# we override "recipe.hooks.postbuild.1.pattern" to prevent the gui loader from running
	$(ARDUINO_DIR)/arduino-cli --fqbn teensy:avr:teensy35:usb=serial2,speed=120,opt=o2lto compile --export-binaries ersatz80.ino --build-property "recipe.hooks.postbuild.1.pattern="

rom.cpp:	monitor.asm
	z80asm --list=monitor.lst -o monitor.bin monitor.asm
	python3 makerom.py rom.cpp monitor.bin

clean:
	rm -f rom.cpp -r build/

