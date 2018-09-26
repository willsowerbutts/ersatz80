#include <Arduino.h>
#include "debug.h"
#include "sdcard.h"

SdFatSdioEX sd; // can also try using SdFatSdio here -- it may be slower?

void sdcard_init() {
    SdioCard *card;
    report("ersatz80: initializing SD card: ");

    if (!sd.begin()) {
        report("failed!\r\n");
        return;
    }

    card = sd.card();
    switch(card->type()){
        case 0:
            report("SDv1");
            break;
        case 1:
            report("SDv2");
            break;
        case 3:
            report("SDHC");
            break;
        default:
            report("(unknown card type)");
    }
    report(" %d blocks (%.1fGB) FAT%d\r\n", card->cardSize(), (float)card->cardSize() / 2097152.0, sd.fatType());
}

