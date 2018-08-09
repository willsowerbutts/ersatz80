#include <stdbool.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <util/crc16.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include "debug.h"
#include "serial.h"
#include "version.h"

void reboot(void)
{
    wdt_enable(WDTO_15MS);
    cli();
    while(1); // wait for watchdog to reset us
}

int main(void)
{
    // initialise serial
    serial_init();
    debug_init();

    // say hello over serial
    report("hello world\n");

    while(1){
        report("more stuff ...\n");
    }

    return 0;
}

