/*
 * Go through buffer and tokenize into argv/argc structure.
 */

#include <ctype.h>
#include <string.h>
#include "tokenize.h"

int tokenize(char *buffer, const char**args, int maxargs)
{
    int argc = 0;

    while (*buffer != '\0' && argc < maxargs - 1) {
        /* Look for first non-space */
        while (isspace(*buffer)) {
            ++buffer;
        }
        /* If a quote or apostrophe, take the whole field */
        if (*buffer == '"' || *buffer == '\'') {
            args[argc++] = buffer + 1;
            int delim = *buffer++;
            while (*buffer != delim && *buffer != '\0') {
                ++buffer;
            }
        } else if (*buffer != '\0') {
            args[argc++] = buffer++;
            while (!isspace(*buffer) && *buffer != '\0') {
                ++buffer;
            }
        }
        if (*buffer != '\0') {
            *buffer++ = '\0';
        }
    }

    /* Put remainder in this arg */
    if (strlen(buffer) != 0) {
       args[argc++] = buffer;
    }

    return argc;
}

char *strstrip(char *s)
{
    size_t size;
    char *end;

    size = strlen(s);

    if (size != 0) {
        end = s + size - 1;
        while (end >= s && isspace(*end))
            end--;
        *(end + 1) = '\0';

        while (*s && isspace(*s))
            s++;
    }

    return s;
}
