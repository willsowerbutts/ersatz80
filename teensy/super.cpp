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
char supervisor_cmd_buffer[SBUFLEN];
#define is_cmd(x) (!strcasecmp_P(buf, PSTR(x)))

void super_reset(int argc, char *argv[]);
void super_regs(int argc, char *argv[]);
void super_clk(int argc, char *argv[]);
void z80_show_regs(void);

void supervisor_menu(void)
{
    debug_boldon();

    while(true){
        report("Supervisor> ");
        serial_read_line((unsigned char*)supervisor_cmd_buffer, SBUFLEN); // TODO - accumulate this asynchronously
        report("\r\n");
        if(!execute_supervisor_command(supervisor_cmd_buffer))
            break;
    }
    debug_boldoff();
}

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

    // report("parsed: argc=%d", argc);
    // for(int i=0; i<argc; i++)
    //     report(", argv[%d]=\"%s\"", i, argv[i]);

    if(argc == 0)
        return true;

    if(!strcasecmp(argv[0], "quit") || !strcasecmp(argv[0], "exit"))
        return false;

    // maybe this command decoding should be table driven?

    if(!strcasecmp(argv[0], "regs")){
        super_regs(argc-1, argv+1);
        return true;
    }

    if(!strcasecmp(argv[0], "clk")){
        super_clk(argc-1, argv+1);
        return true;
    }

    if(!strcasecmp(argv[0], "reset")){
        super_reset(argc-1, argv+1);
        return true;
    }

    report("error: unknown command \"%s\"\r\n", argv[0]);

    return true;
}

void super_regs(int argc, char *argv[])
{
    z80_show_regs();
}

void super_clk(int argc, char *argv[])
{
    if(argc == 1 && !strcasecmp(argv[0], "stop")){
        z80_clk_switch_stop();
    }else if(argc == 1 && !strcasecmp(argv[0], "fast")){
        z80_clk_switch_fast();
    }else if(argc == 1 && !strcasecmp(argv[0], "slow")){
        z80_clk_switch_slow(1000000);
    }else if(argc == 2 && !strcasecmp(argv[0], "slow")){
        char *endptr = NULL;
        float f = strtof(argv[1], &endptr);
        if(f == 0)
            report("error: bad frequency\r\n");
        else{
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
                    report("Unrecognised unit suffix?");
            }
            report("[f=%.3f]", f);
            z80_clk_switch_slow(f);
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
