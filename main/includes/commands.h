/*
 * commands.h
 *
 */
#ifndef __commands_h_included
#define __commands_h_included

#include <stdarg.h>

#include "os_freertos.h"

#if 0
typedef struct command_params {
    int (*getchar)(void);
    int (*putchar)(int ch);
    int (*readline)(char *buf, size_t len);
    int (*printf)(const char* fmt, ...);
} command_params_t;
#endif

#if 0
os_thread_t start_commands(command_params_t* params);
#else
os_thread_t start_commands(int infd, int outfd);
#endif

#endif /* __commands_h_included */

