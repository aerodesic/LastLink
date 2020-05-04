/*
 * commands.h
 *
 */
#ifndef __commands_h_included
#define __commands_h_included

#include <stdarg.h>

#include "os_freertos.h"

os_thread_t start_commands(FILE* in, FILE* out);

#endif /* __commands_h_included */

