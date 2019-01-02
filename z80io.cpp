#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "z80.h"
#include "z80io.h"
#include "clock.h"
#include "serial.h"
#include "debug.h"
#include "super.h"
#include "rom.h"
#include "irq.h"
#include "mmu.h"
#include "timer.h"
#include "disk.h"

static uint8_t mmu_foreign_read(uint16_t address);
static uint8_t mmu_read(uint16_t address);
static uint8_t uart_read_data(uint16_t address);
static uint8_t uart_read_status(uint16_t address);
static uint8_t user_leds_read(uint16_t address);

static void mmu_foreign_write(uint16_t address, uint8_t value);
static void mmu_write(uint16_t address, uint8_t value);
static void timer_write(uint16_t address, uint8_t value);
static void uart_write_data(uint16_t address, uint8_t value);
static void user_leds_write(uint16_t address, uint8_t value);

typedef struct {
    uint8_t (*read_function)(uint16_t address);
    void (*write_function)(uint16_t address, uint8_t value);
} ioregister_functions_t;

// table of read/write function pointers for each Z80 IO address
static const ioregister_functions_t io_register_handler[256] = {
    { uart_read_status,     NULL },                 // 0x00 - UART 0 status
    { uart_read_data,       uart_write_data },      // 0x01 - UART 0 data
    { NULL,                 NULL },                 // 0x02   (reserved for UART 1)
    { NULL,                 NULL },                 // 0x03   ...
    { NULL,                 NULL },                 // 0x04   (reserved for UART 2)
    { NULL,                 NULL },                 // 0x05   ...
    { NULL,                 NULL },                 // 0x06   (reserved for UART 3)
    { NULL,                 NULL },                 // 0x07   ...
    { NULL,                 NULL },                 // 0x08   (reserved for UART 4)
    { NULL,                 NULL },                 // 0x09   ...
    { NULL,                 NULL },                 // 0x0a   (reserved for UART 5)
    { NULL,                 NULL },                 // 0x0b   ...
    { NULL,                 NULL },                 // 0x0c   (reserved for UART 6)
    { NULL,                 NULL },                 // 0x0d   ...
    { NULL,                 NULL },                 // 0x0e   (reserved for UART 7)
    { NULL,                 NULL },                 // 0x0f   ...
    { user_leds_read,       user_leds_write },      // 0x10 - user LEDs (low 8 bits)
    { user_leds_read,       user_leds_write },      // 0x11 - user LEDs (top 4 bits)
    { NULL,                 timer_write },          // 0x12 - very simple duration timer
    { NULL,                 NULL },                 // 0x13
    { NULL,                 NULL },                 // 0x14
    { NULL,                 NULL },                 // 0x15
    { NULL,                 NULL },                 // 0x16
    { NULL,                 NULL },                 // 0x17
    { int_requests_read,    int_requests_write },   // 0x18 - interrupt controller (status / clear pending)
    { int_mask_read,        int_mask_write },       // 0x19 - interrupt controller (mask)
    { timer_read_status,    timer_write_control },  // 0x1a - timer status and control register
    { NULL,                 NULL },                 // 0x1b
    { NULL,                 NULL },                 // 0x1c
    { NULL,                 NULL },                 // 0x1d
    { NULL,                 NULL },                 // 0x1e
    { NULL,                 NULL },                 // 0x1f
    { NULL,                 NULL },                 // 0x20   (reserved for disk interface extensions)
    { disk_sector_read,     disk_sector_write   },  // 0x21 - disk interface (sector bits 16--23)
    { disk_sector_read,     disk_sector_write   },  // 0x22 - disk interface (sector bits 8--15)
    { disk_sector_read,     disk_sector_write   },  // 0x23 - disk interface (sector bits 0--7)
    { disk_address_read,    disk_address_write  },  // 0x24 - disk interface (DMA address bits 16--21 + 2 flags)
    { disk_address_read,    disk_address_write  },  // 0x25 - disk interface (DMA address bits 8--15)
    { disk_address_read,    disk_address_write  },  // 0x26 - disk interface (DMA address bits 0--7)
    { disk_seccount_read,   disk_seccount_write },  // 0x27 - disk interface (sector count)
    { disk_status_read,     disk_command_write  },  // 0x28 - disk interface (command/status)
    { NULL,                 NULL },                 // 0x29
    { NULL,                 NULL },                 // 0x2a
    { NULL,                 NULL },                 // 0x2b
    { NULL,                 NULL },                 // 0x2c
    { NULL,                 NULL },                 // 0x2d
    { NULL,                 NULL },                 // 0x2e
    { NULL,                 NULL },                 // 0x2f
    { NULL,                 NULL },                 // 0x30
    { NULL,                 NULL },                 // 0x31
    { NULL,                 NULL },                 // 0x32
    { NULL,                 NULL },                 // 0x33
    { NULL,                 NULL },                 // 0x34
    { NULL,                 NULL },                 // 0x35
    { NULL,                 NULL },                 // 0x36
    { NULL,                 NULL },                 // 0x37
    { NULL,                 NULL },                 // 0x38
    { NULL,                 NULL },                 // 0x39
    { NULL,                 NULL },                 // 0x3a
    { NULL,                 NULL },                 // 0x3b
    { NULL,                 NULL },                 // 0x3c
    { NULL,                 NULL },                 // 0x3d
    { NULL,                 NULL },                 // 0x3e
    { NULL,                 NULL },                 // 0x3f
    { NULL,                 NULL },                 // 0x40
    { NULL,                 NULL },                 // 0x41
    { NULL,                 NULL },                 // 0x42
    { NULL,                 NULL },                 // 0x43
    { NULL,                 NULL },                 // 0x44
    { NULL,                 NULL },                 // 0x45
    { NULL,                 NULL },                 // 0x46
    { NULL,                 NULL },                 // 0x47
    { NULL,                 NULL },                 // 0x48
    { NULL,                 NULL },                 // 0x49
    { NULL,                 NULL },                 // 0x4a
    { NULL,                 NULL },                 // 0x4b
    { NULL,                 NULL },                 // 0x4c
    { NULL,                 NULL },                 // 0x4d
    { NULL,                 NULL },                 // 0x4e
    { NULL,                 NULL },                 // 0x4f
    { NULL,                 NULL },                 // 0x50
    { NULL,                 NULL },                 // 0x51
    { NULL,                 NULL },                 // 0x52
    { NULL,                 NULL },                 // 0x53
    { NULL,                 NULL },                 // 0x54
    { NULL,                 NULL },                 // 0x55
    { NULL,                 NULL },                 // 0x56
    { NULL,                 NULL },                 // 0x57
    { NULL,                 NULL },                 // 0x58
    { NULL,                 NULL },                 // 0x59
    { NULL,                 NULL },                 // 0x5a
    { NULL,                 NULL },                 // 0x5b
    { NULL,                 NULL },                 // 0x5c
    { NULL,                 NULL },                 // 0x5d
    { NULL,                 NULL },                 // 0x5e
    { NULL,                 NULL },                 // 0x5f
    { NULL,                 NULL },                 // 0x60
    { NULL,                 NULL },                 // 0x61
    { NULL,                 NULL },                 // 0x62
    { NULL,                 NULL },                 // 0x63
    { NULL,                 NULL },                 // 0x64
    { NULL,                 NULL },                 // 0x65
    { NULL,                 NULL },                 // 0x66
    { NULL,                 NULL },                 // 0x67
    { NULL,                 NULL },                 // 0x68
    { NULL,                 NULL },                 // 0x69
    { NULL,                 NULL },                 // 0x6a
    { NULL,                 NULL },                 // 0x6b
    { NULL,                 NULL },                 // 0x6c
    { NULL,                 NULL },                 // 0x6d
    { NULL,                 NULL },                 // 0x6e
    { NULL,                 NULL },                 // 0x6f
    { NULL,                 NULL },                 // 0x70
    { NULL,                 NULL },                 // 0x71
    { NULL,                 NULL },                 // 0x72
    { NULL,                 NULL },                 // 0x73
    { NULL,                 NULL },                 // 0x74
    { NULL,                 NULL },                 // 0x75
    { NULL,                 NULL },                 // 0x76
    { NULL,                 NULL },                 // 0x77
    { mmu_read,             mmu_write },            // 0x78 - MMU bank 0 select (Zeta2 compatible)
    { mmu_read,             mmu_write },            // 0x79 - MMU bank 1 select (Zeta2 compatible)
    { mmu_read,             mmu_write },            // 0x7a - MMU bank 2 select (Zeta2 compatible)
    { mmu_read,             mmu_write },            // 0x7b - MMU bank 3 select (Zeta2 compatible)
    { mmu_foreign_read,     mmu_foreign_write },    // 0x7c - MMU foreign context bank 0 select
    { mmu_foreign_read,     mmu_foreign_write },    // 0x7d - MMU foreign context bank 1 select
    { mmu_foreign_read,     mmu_foreign_write },    // 0x7e - MMU foreign context bank 2 select
    { mmu_foreign_read,     mmu_foreign_write },    // 0x7f - MMU foreign context bank 3 select
    // remaining unspecified entries come out as {NULL,NULL}.
};

uint8_t iodevice_read(uint16_t address)
{
    if(io_register_handler[address & 0xFF].read_function)
        return io_register_handler[address & 0xFF].read_function(address);
    report("[IOR %04x]", address);
    return 0xAA;
}

void iodevice_write(uint16_t address, uint8_t value) // call ONLY when in DMA mode!
{
    if(io_register_handler[address & 0xFF].write_function)
        io_register_handler[address & 0xFF].write_function(address, value);
    else{
        report("[IOW %04x %02x]", address, value);
    }
}

uint8_t memory_read(uint16_t address)
{
    return basic_rom[address & ROM_ADDR_MASK];
}

void memory_write(uint16_t address, uint8_t value)
{
    // not implemented -- we only present ROM
}

#define UART_RX_FIFO_BUFFER_SIZE 128
uint8_t uart_rx_fifo_waiting = 0;
uint8_t uart_rx_fifo_start = 0;
uint8_t uart_rx_fifo_buffer[UART_RX_FIFO_BUFFER_SIZE];
bool uart0_on_console = true;

bool uart_rx_fifo_push(uint8_t keyin)
{
    if(uart_rx_fifo_waiting >= UART_RX_FIFO_BUFFER_SIZE)
        return false;

    // if we start using interrupts, consider atomicity
    uart_rx_fifo_buffer[(uart_rx_fifo_start + uart_rx_fifo_waiting) % UART_RX_FIFO_BUFFER_SIZE] = keyin;
    uart_rx_fifo_waiting++;
    return true;
}

uint8_t uart_rx_fifo_pop(void)
{
    uint8_t r;
    if(!uart_rx_fifo_waiting)
        return 0xff;

    // if we start using interrupts, consider atomicity
    r = uart_rx_fifo_buffer[uart_rx_fifo_start];
    uart_rx_fifo_waiting--;
    uart_rx_fifo_start = (uart_rx_fifo_start+1) % UART_RX_FIFO_BUFFER_SIZE;
    return r;
}

static uint8_t uart_read_status(uint16_t address)
{
    if(uart0_on_console){
        return (uart_rx_fifo_waiting ? 0x80 : 0x00) | (Serial.availableForWrite()>0 ? 0x00 : 0x40);
    }else{
        return (Serial6.available() ? 0x80 : 0x00) | (Serial6.availableForWrite()>0 ? 0x00 : 0x40);
    }
}

static uint8_t uart_read_data(uint16_t address)
{
    if(uart0_on_console)
        return uart_rx_fifo_pop();
    else
        return Serial6.read();

}

static void uart_write_data(uint16_t address, uint8_t value)
{
    if(uart0_on_console){
        debug_mode_user();
        Serial.write(value);
    }else{
        if(Serial6.availableForWrite() > 0)
            Serial6.write(value);
    }
}

bool uart_interrupt_request(void)
{
    if(uart0_on_console){
        return uart_rx_fifo_waiting;
    }else{
        return Serial6.available();
    }
}

static void user_leds_write(uint16_t address, uint8_t value)
{
    if(address & 1)
        z80_set_user_leds((z80_get_user_leds() & 0xFF) | ((value & 0x0F) << 8));
    else
        z80_set_user_leds((z80_get_user_leds() & 0xFF00) | value);
}

static uint8_t user_leds_read(uint16_t address)
{
    uint16_t l = z80_get_user_leds();

    if(address & 1)
        return (l >> 8) & 0x0F;
    else
        return l & 0xFF;
}

static uint8_t mmu_foreign_read(uint16_t address)
{
    return z80_get_mmu_foreign(address & 3);
}

static void mmu_foreign_write(uint16_t address, uint8_t value)
{
    z80_set_mmu_foreign(address & 3, value);
}

static uint8_t mmu_read(uint16_t address)
{
    return z80_get_mmu(address & 3);
}

static void mmu_write(uint16_t address, uint8_t value)
{
    z80_set_mmu(address & 3, value);
}

static void timer_write(uint16_t address, uint8_t value)
{
    static unsigned long last_timer = 0;
    unsigned long now;

    now = micros();
    if(value)
        report("timer: %lu\r\n", now-last_timer);
    last_timer = now;
}
