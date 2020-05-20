/*
 * commands.h
 *
 */
#ifndef __commands_h_included
#define __commands_h_included

#include <stdarg.h>

os_thread_t start_commands(FILE* in, FILE* out);

void init_commands(void);
void deinit_commands(void);

bool add_command(const char* name, int (*function)(int argc, const char **argv));
bool remove_command(const char* name);
void show_help(const char* name, const char *params, const char *description);

#endif /* __commands_h_included */

