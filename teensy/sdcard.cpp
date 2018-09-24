#include <Arduino.h>
#include <SD.h>
#include <SD_t3.h>

#include "debug.h"
#include "sdcard.h"

void sdcard_init() {
  report("ersatz80: initializing SD card...\r\n");
  
  if (!SD.begin(254)) {
        report("ersatz80: SD card initialization failed!\r\n");
  }
}

