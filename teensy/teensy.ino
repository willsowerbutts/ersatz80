#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "z80.h"
#include "serial.h"
#include "debug.h"
#include "super.h"
#include "rom.h"
#include "sdcard.h"

#define UART_RX_FIFO_BUFFER_SIZE 128
uint8_t uart_rx_fifo_waiting = 0;
uint8_t uart_rx_fifo_start = 0;
uint8_t uart_rx_fifo_buffer[UART_RX_FIFO_BUFFER_SIZE];
bool supervisor_input_mode = false;

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

uint8_t uart_read_status(uint16_t address)
{
    return (uart_rx_fifo_waiting ? 0x80 : 0x00) | (Serial.availableForWrite()>0 ? 0x00 : 0x40);
}

uint8_t uart_read_data(uint16_t address)
{
    return uart_rx_fifo_pop();
}

void uart_write_data(uint16_t address, uint8_t value)
{
    Serial.write(value);
}

void user_leds_write(uint16_t address, uint8_t value)
{
    if(address & 1)
        user_led = (user_led & 0xFF00) | value;
    else
        user_led = (user_led & 0xFF) | ((value & 0x0F) << 8);
    shift_register_update();
}

uint8_t user_leds_read(uint16_t address)
{
    if(address & 1)
        return user_led & 0xFF;
    else
        return (user_led >> 8) & 0x0F;
}

uint8_t mmu_read(uint16_t address)
{
    return mmu[address & 3];
}

void mmu_write(uint16_t address, uint8_t value)
{
    z80_bus_master();
    z80_set_mmu(address & 3, value);
    z80_bus_slave();
}

typedef struct {
    uint8_t (*read_function)(uint16_t address);
    void (*write_function)(uint16_t address, uint8_t value);
} ioregister_functions_t;

const ioregister_functions_t io_register_handler[256] = {
    { uart_read_status,     NULL },                 // 0x00 - UART status
    { uart_read_data,       uart_write_data },      // 0x01 - UART data
    { NULL,                 NULL },                 // 0x02
    { NULL,                 NULL },                 // 0x03
    { NULL,                 NULL },                 // 0x04
    { NULL,                 NULL },                 // 0x05
    { NULL,                 NULL },                 // 0x06
    { NULL,                 NULL },                 // 0x07
    { NULL,                 NULL },                 // 0x08
    { NULL,                 NULL },                 // 0x09
    { NULL,                 NULL },                 // 0x0a
    { NULL,                 NULL },                 // 0x0b
    { NULL,                 NULL },                 // 0x0c
    { NULL,                 NULL },                 // 0x0d
    { NULL,                 NULL },                 // 0x0e
    { NULL,                 NULL },                 // 0x0f
    { NULL,                 NULL },                 // 0x10
    { user_leds_read,       user_leds_write },      // 0x11 - user LEDs (low 8 bits)
    { user_leds_read,       user_leds_write },      // 0x12 - user LEDs (top 4 bits)
    { NULL,                 NULL },                 // 0x13
    { NULL,                 NULL },                 // 0x14
    { NULL,                 NULL },                 // 0x15
    { NULL,                 NULL },                 // 0x16
    { NULL,                 NULL },                 // 0x17
    { NULL,                 NULL },                 // 0x18
    { NULL,                 NULL },                 // 0x19
    { NULL,                 NULL },                 // 0x1a
    { NULL,                 NULL },                 // 0x1b
    { NULL,                 NULL },                 // 0x1c
    { NULL,                 NULL },                 // 0x1d
    { NULL,                 NULL },                 // 0x1e
    { NULL,                 NULL },                 // 0x1f
    { NULL,                 NULL },                 // 0x20
    { NULL,                 NULL },                 // 0x21
    { NULL,                 NULL },                 // 0x22
    { NULL,                 NULL },                 // 0x23
    { NULL,                 NULL },                 // 0x24
    { NULL,                 NULL },                 // 0x25
    { NULL,                 NULL },                 // 0x26
    { NULL,                 NULL },                 // 0x27
    { NULL,                 NULL },                 // 0x28
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
    { mmu_read,             mmu_write },            // 0x78 - MMU bank select (Zeta2 compatible)
    { mmu_read,             mmu_write },            // 0x79 - MMU bank select (Zeta2 compatible)
    { mmu_read,             mmu_write },            // 0x7a - MMU bank select (Zeta2 compatible)
    { mmu_read,             mmu_write },            // 0x7b - MMU bank select (Zeta2 compatible)
    { NULL,                 NULL },                 // 0x7c
    { NULL,                 NULL },                 // 0x7d
    { NULL,                 NULL },                 // 0x7e
    { NULL,                 NULL },                 // 0x7f
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
    // nop
}

void z80_send_instruction(uint8_t opcode)
{
    while(!(z80_mreq_asserted() && z80_rd_asserted()))
        z80_clock_pulse();
    z80_clock_pulse_drive_data(opcode);
}

uint16_t z80_send_instruction_read_stack(uint8_t opcode)
{
    uint16_t r;

    z80_send_instruction(opcode);
    // now it will write to the stack
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();
    r = z80_bus_data();
    z80_clock_pulse_while_writing();
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();
    r = (r << 8) | z80_bus_data();
    return r;
}

void z80_show_regs(void)
{
    uint16_t pc, sp, af, bc, de, hl, ix, iy, af_, bc_, de_, hl_;
    uint8_t i;

    z80_clk_pause();

    // this code does not deal with the situation where CPU is HALTed.
    // solution might be: wake CPU with an int/nmi, capture PC when it writes it to the
    //                    stack. capture regs as usual. then IRET and JP to PC-1, then 
    //                    feed it a HALT when it fetches PC-1. it will HALT with PC correct.

    // if we're partway through an M1 cycle, allow it to complete first
    while(z80_m1_asserted()){
        if(!z80_clk_running())
            z80_clock_pulse();
        handle_z80_bus(); 
    }

    // wait for a new M1 cycle to start
    while(!z80_m1_asserted()){
        if(!z80_clk_running())
            z80_clock_pulse();
        handle_z80_bus(); 
    }

    // disable the RAM so we can control the data bus
    ram_ce = false;
    shift_register_update();

    // now we feed it a synthesised instruction - F5 (PUSH AF)
    while(!(z80_mreq_asserted() && z80_rd_asserted()))
        z80_clock_pulse();
    pc = z80_bus_address();                      // (this gives us the PC register)
    z80_clock_pulse_drive_data(0xF5);            // PUSH AF

    // now it will write to the stack
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();
    sp = z80_bus_address() + 1;                  // (this gives us the SP register)
    af = z80_bus_data();
    z80_clock_pulse_while_writing();
    while(!(z80_mreq_asserted() && z80_wr_asserted()))
        z80_clock_pulse();
    af = (af << 8) | z80_bus_data();

    bc = z80_send_instruction_read_stack(0xC5);  // PUSH BC
    de = z80_send_instruction_read_stack(0xD5);  // PUSH DE
    hl = z80_send_instruction_read_stack(0xE5);  // PUSH HL
    z80_send_instruction(0x08);                  // EX AF, AF'
    af_ = z80_send_instruction_read_stack(0xF5); // PUSH AF
    z80_send_instruction(0x08);                  // EX AF, AF' again (swap back)
    z80_send_instruction(0xD9);                  // EXX
    bc_ = z80_send_instruction_read_stack(0xC5); // PUSH BC
    de_ = z80_send_instruction_read_stack(0xD5); // PUSH DE
    hl_ = z80_send_instruction_read_stack(0xE5); // PUSH HL
    z80_send_instruction(0xD9);                  // EXX again (swap back)
    z80_send_instruction(0xDD);                  // IX prefix
    ix  = z80_send_instruction_read_stack(0xE5); // PUSH IX
    z80_send_instruction(0xFD);                  // IY prefix
    iy  = z80_send_instruction_read_stack(0xE5); // PUSH IY
    z80_send_instruction(0xED);                  // ED prefix
    z80_send_instruction(0x57);                  // LD A,I - note this affects the flags register
    i = z80_send_instruction_read_stack(0xF5) >> 8; // PUSH AF - I is now in A (high bits)

    // finally we need to put AF, SP and PC back as they were before our tinkering
    z80_send_instruction(0xF1);                  // POP af
    z80_send_instruction(af & 0xFF);             //  ...
    z80_send_instruction(af >> 8);               //  ...
    z80_send_instruction(0x31);                  // LD SP, xxxx
    z80_send_instruction(sp & 0xFF);             //  ...
    z80_send_instruction(sp >> 8);               //  ...
    z80_send_instruction(0xC3);                  // JP xxxx
    z80_send_instruction(pc & 0xFF);             //  ...
    z80_send_instruction(pc >> 8);               //  ...

    ram_ce = true;                               // turn back on the RAM
    shift_register_update();
    z80_clk_resume();

    report("PC=%04x SP=%04x\r\nAF=%04x AF'=%04x\r\n" \
           "BC=%04x BC'=%04x\r\nDE=%04x DE'=%04x\r\n" \
           "HL=%04x HL'=%04x\r\nIX=%04x IY=%04x I=%02x\r\n",
           pc, sp, af, af_,
           bc, bc_, de, de_,
           hl, hl_, ix, iy, i);
}

inline void z80_complete_read(uint8_t data)
{
    z80_setup_drive_data(data);
    z80_set_busrq(true);
    z80_set_release_wait(true);
    while(!z80_busack_asserted())
        if(!z80_clk_running())
            z80_clock_pulse();
    z80_shutdown_drive_data();
    z80_set_release_wait(false);
    z80_set_busrq(false);
    // return with Z80 running
}

inline void z80_complete_write(void)
{
    z80_set_busrq(true);
    z80_set_release_wait(true);
    while(!z80_busack_asserted())
        if(!z80_clk_running())
            z80_clock_pulse();
    z80_set_release_wait(false);
    // return with DMA capable -- caller must do z80_set_busrq(false);
}

void handle_z80_bus(void)
{
    if(z80_wait_asserted()){
        if(z80_iorq_asserted()){
            if(z80_rd_asserted()){
                z80_complete_read(iodevice_read(z80_bus_address()));
            }else if(z80_wr_asserted()){
                z80_complete_write(); // leaves us in DMA mode
                iodevice_write(z80_bus_address_low8(), z80_bus_data());
                z80_set_busrq(false);
            }else
                report("(iorq weird?)");
        } else if(z80_mreq_asserted()){
            if(z80_rd_asserted()){
                z80_complete_read(memory_read(z80_bus_address()));
            }else if(z80_wr_asserted()){
                z80_complete_write(); // leaves us in DMA mode
                memory_write(z80_bus_address_low8(), z80_bus_data());
                z80_set_busrq(false);
            }else
                report("(mreq weird?)");
        } else
            report("(wait weird?)");
    }
}

void handle_serial_input(void)
{
    int key;

    if((key = Serial.read()) >= 0){
        if(supervisor_input_mode){
            if(!supervisor_menu_key_in(key)){
                supervisor_input_mode = false;
                supervisor_menu_exit();
            }
        }else{
            if(key == SUPERVISOR_ESCAPE_KEYCODE){
                supervisor_input_mode = true;
                supervisor_menu_enter();
            }else if(!uart_rx_fifo_push(key)){
                Serial.write(0x07); // sound bell on overflow
                return; // ... and come back to this task later
            }
        }
    }
}

void __assert_func(const char *__file, int __lineno, const char *__func, const char *__sexp) {
    // put the Z80 bus in a safe mode where we're not driving any signals
    z80_bus_slave();

    // transmit diagnostic informations through serial link. 
    report("assert() failed in %s:%d function %s: %s\r\n", __file, __lineno, __func, __sexp);
    Serial.flush();
    // end program execution.
    while(1);
}

void setup() {
    z80_setup();
    Serial.begin(115200); // baud rate specified here is irrelevant
    while(!Serial.dtr()); // wait for a terminal to connect to the USB serial device
    report("                     _       ___   ___  \r\n  ___ _ __ ___  __ _| |_ ___( _ ) / _ \\ \r\n"
           " / _ \\ '__/ __|/ _` | __|_  / _ \\| | | |\r\n|  __/ |  \\__ \\ (_| | |_ / / (_) | |_| |\r\n"
           " \\___|_|  |___/\\__,_|\\__/___\\___/ \\___/ \r\nersatz80: init (%.1fMHz ARM, %.1fMHz bus)\r\n", 
           F_CPU/1000000.0, F_BUS/1000000.0);
    sdcard_init();
    z80_do_reset();
    mmu_setup();
    sram_setup();
    report("ersatz80: load ROM\r\n");
    load_program_to_sram(monitor_rom, MONITOR_ROM_START, MONITOR_ROM_SIZE, MONITOR_ROM_START);
    report("ersatz80: reset Z80\r\n");
    z80_do_reset();
    report("Supervisor keycode is Ctrl+%c.\r\n", 'A' - 1 + SUPERVISOR_ESCAPE_KEYCODE);
    // start up the Z80
    ram_ce = true;
    shift_register_update();
    z80_clk_switch_fast();
}

void loop() {
    while(true){
        handle_z80_bus();
        handle_serial_input();
    }
}
