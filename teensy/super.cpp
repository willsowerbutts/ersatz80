#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "z80.h"
#include "serial.h"
#include "super.h"
#include "debug.h"
#include "rom.h"

#define SBUFLEN 80
#define SMAXARG 10
int supervisor_cmd_offset = 0;
char supervisor_cmd_buffer[SBUFLEN];
#define is_cmd(x) (!strcasecmp_P(buf, PSTR(x)))

void super_reset(int argc, char *argv[]);
void super_regs(int argc, char *argv[]);
void super_clk(int argc, char *argv[]);
void z80_show_regs(void);

bool supervisor_menu_key_in(unsigned char keypress)
{
    if(keypress == 0x7f || keypress == 0x08){ // backspace and delete
        if(supervisor_cmd_offset > 0){
            Serial.write("\x08 \x08"); // erase last char
            supervisor_cmd_offset--;
        }
    }else if(keypress == 0x0d || keypress == 0x0a){
        supervisor_cmd_buffer[supervisor_cmd_offset] = 0;
        report("\r\n");
        bool r = execute_supervisor_command(supervisor_cmd_buffer);
        if(r)
            supervisor_menu_enter();
        return r;
    }else if(keypress == SUPERVISOR_ESCAPE_KEYCODE){
        report("*abort*\r\n");
        return false;
    }else if(keypress >= 0x20){
        if(supervisor_cmd_offset < (SBUFLEN-1)){
            supervisor_cmd_buffer[supervisor_cmd_offset++] = keypress;
            debug_boldon();
            Serial.write(keypress);
            debug_boldoff();
        }else{
            Serial.write(0x07); // sound the bell
        }
    }
    return true;
}

void supervisor_menu_enter(void)
{
    supervisor_cmd_offset = 0;
    report("Supervisor> ");
}

void supervisor_menu_exit(void)
{
}

typedef struct {
    const char *name;
    void (*function)(int argc, char *argv[]);
} cmd_entry_t;

const cmd_entry_t cmd_table[] = {
    { "quit",       NULL },
    { "exit",       NULL },
    { "q",          NULL },
    { "regs",       &super_regs },
    { "clk",        &super_clk },
    { "reset",      &super_reset },
    // list terminator:
    { NULL,         NULL }
};

bool execute_supervisor_command(char *cmd_buffer) // return false on exit/quit etc, true otherwise
{
    int argc = 0;
    char *p, *argv[SMAXARG];

    p = cmd_buffer;
    while(*p){
        while(isspace(*p)) // skip over leading whitespace
            p++;
        if(!*p) // end of line?
            break;
        argv[argc++] = p; // store ptr to start of command
        while(*p && !isspace(*p)) // find end of command
            p++;
        if(!*p) // end of line?
            break;
        *(p++) = 0; // overwrite whitespace with NUL
    }

    if(argc == 0)
        return true;

    if(!strcasecmp(argv[0], "quit") || !strcasecmp(argv[0], "exit"))
        return false;

    for(cmd_entry_t *cmd=cmd_table; cmd->name; cmd++){
        if(!strcasecmp(argv[0], cmd->name)){
            if(cmd->function == NULL){
                return false;
            }else{
                cmd->function(argc-1, argv+1);
                return true;
            }
        }
    }

    report("error: unknown command \"%s\"\r\n", argv[0]);

    return true;
}

void super_regs(int argc, char *argv[])
{
    // TODO -- need to pause clock while we do this
    z80_show_regs();
}

void super_clk(int argc, char *argv[])
{
    if(argc == 1 && !strcasecmp(argv[0], "stop")){
        z80_clk_switch_stop();
        report("clock: stopped\r\n");
    }else if(argc == 1 && !strcasecmp(argv[0], "fast")){
        z80_clk_switch_fast();
        report("clock: fast\r\n");
    }else if(argc <= 2 && !strcasecmp(argv[0], "slow")){
        float f;
        if(argc == 1){
            f = 1000000;
        }else{ // argc == 2
            char *endptr = NULL;
            f = strtof(argv[1], &endptr);
            if(f == 0){
                report("error: bad frequency\r\n");
                return;
            }else{
                switch(tolower(*endptr)){
                    case 0:
                        break;
                    case 'k':
                        f *= 1000;
                        break;
                    case 'm':
                        f *= 1000000;
                        break;
                    case 'g': // possibly getting a bit ambitious here!
                        f *= 1000000000;
                        break;
                    default:
                        report("error: unrecognised unit suffix?");
                        return;
                }
            }
            f = z80_clk_switch_slow(f);
            report("clock: slow ");
            if(f >= 950000.0)
                report("%.3fMHz", f / 1000000.0);
            else if(f > 950.0)
                report("%.3fkHz", f / 1000.0);
            else
                report("%.3fHz", f);
            report("\r\n");
        }
    }else{
        report("error: syntax: clk [stop|fast|slow <freq [kHz|MHz]>]\r\n");
    }
}

void super_reset(int argc, char *argv[])
{
    // TODO -- this assumes clock is running
    z80_set_reset(true);
    delay(10);
    z80_set_release_wait(true);
    z80_set_release_wait(false);
    z80_set_reset(false);
}
