#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "z80io.h"
#include "clock.h"
#include "serial.h"
#include "debug.h"
#include "super.h"
#include "irq.h"
#include "mmu.h"
#include "timer.h"
#include "disk.h"
#include "rom.h"

static bool usb_acm_supervisor_input_mode = false;
void handle_usb_acm_input(void)
{
    int key;

    while((key = Serial.read()) >= 0){
        if(!uart0_on_console){
            // USB ACM is used for supervisor mode only
            if(!supervisor_menu_key_in(key))
                supervisor_menu_enter();
        }else{
            // device is shared between supervisor input and UART0
            if(usb_acm_supervisor_input_mode){
                if(!supervisor_menu_key_in(key)){
                    usb_acm_supervisor_input_mode = false;
                }
            }else{
                if(key == SUPERVISOR_ESCAPE_KEYCODE){
                    usb_acm_supervisor_input_mode = true;
                    supervisor_menu_enter();
                }else if(!uart_rx_fifo_push(key)){
                    Serial.write(0x07); // sound bell on overflow
                    return; // ... and come back to this task later
                }
            }
        }
    }
}

void __assert_func(const char *__file, int __lineno, const char *__func, const char *__sexp) {
    // put the Z80 data and address bus into a mode where we're not driving any signals
    z80_bus_slave();

    // transmit diagnostic informations through serial link. 
    report("assert() failed in %s:%d function %s: %s\r\n", __file, __lineno, __func, __sexp);
    Serial.flush();
    // end program execution.
    while(1);
}

void uart_setup(int baud)
{
    // UART0 can optionally be redirected to UART on the expansion
    // connector. Hardware flow control is used. This is used only
    // when uart0_on_console=false.
    // wiring instructions:
    //  connect GND on ersatz80 to GND on peer
    //  RX input   - pin 47 - connect to TX output pin on peer
    //  TX output  - pin 48 - connect to RX input pin on peer
    //  CTS input  - pin 56 - connect to RTS output pin on peer
    //  RTS output - pin 57 - connect to CTS input pin on peer
    Serial6.begin(baud, SERIAL_8N1);
    Serial6.attachCts(56);
    Serial6.attachRts(57);
}

void setup() 
{
    Serial.begin(115200); // console on USB ACM device (baud rate is irrelevant)

    if(!check_pcb_revision()){
        while(true){
            while(!Serial.dtr()); // wait for terminal software to connect to the USB ACM device
#ifdef ERSATZ80_PCB_REV1
            report("ERSATZ80_PCB_REV1 is #defined but this appears to be a different PCB. Edit interface.h and recompile.\r\n");
#else
            report("ERSATZ80_PCB_REV1 is not #defined but this appears to be a different PCB. Edit interface.h and recompile.\r\n");
#endif
            delay(1000);
        }
    }

    clk_setup();
    z80_setup();
    mmu_setup();
    uart_setup(115200);   // setup Serial6 (optional UART on expansion port)

    while(!Serial.dtr()); // wait for terminal software to connect to the USB ACM device
    
    report("                     _       ___   ___  \r\n  ___ _ __ ___  __ _| |_ ___( _ ) / _ \\ \r\n"
           " / _ \\ '__/ __|/ _` | __|_  / _ \\| | | |\r\n|  __/ |  \\__ \\ (_| | |_ / / (_) | |_| |\r\n"
           " \\___|_|  |___/\\__,_|\\__/___\\___/ \\___/ \r\nersatz80: init (%.1fMHz ARM, %.1fMHz bus)\r\n", 
           F_CPU/1000000.0, F_BUS/1000000.0);
    disk_setup();
    sram_setup();

    // TODO: boot to be directed by "boot.cmd" from SD card, with the below executed only
    // if that file does not exist. and this code should be updated to work using monitor
    // commands.
    report("ersatz80: load ROM\r\n");
    load_program_to_sram(monitor_rom, MONITOR_ROM_START, MONITOR_ROM_SIZE, MONITOR_ROM_START);
    // TODO is another reset necessary here?
    report("ersatz80: reset Z80\r\n");
    z80_do_reset();

    report("ersatz80: Supervisor keycode is Ctrl+%c.\r\n", 'A' - 1 + SUPERVISOR_ESCAPE_KEYCODE);

    // off we go!
    z80_set_mode(Z80_UNSUPERVISED);
}

#define DISK_SYNC_INTERVAL      3000 // milliseconds
void loop()
{
    unsigned long now, disk_sync_due = 0;

    while(true){
        now = millis();
        handle_timer(now);                      // poke the timer
        handle_z80_interrupts();                // handle Z80 interrupt line
        if(z80_supervised_mode())               // clock the CPU, if we're in supervised clock mode
            z80_clock_pulse();
        z80_end_dma_mode();
        handle_z80_bus();                       // handle peripheral I/O and memory requests from the Z80
        handle_usb_acm_input();                 // handle input over USB ACM serial
        if(now >= disk_sync_due){               // periodically flush written data to the SD card
            disk_sync();                        // sync all the disks
            disk_sync_due += DISK_SYNC_INTERVAL;
            if(now >= disk_sync_due)
                disk_sync_due = now + DISK_SYNC_INTERVAL;
        }
    }
}
