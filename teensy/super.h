#ifndef __SUPER_DOT_H__
#define __SUPER_DOT_H__

#define SUPERVISOR_ESCAPE_KEYCODE 7 // Ctrl-G
void supervisor_menu(void);
bool execute_supervisor_command(char *cmd_buffer);

#endif
