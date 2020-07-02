/*
 * UUID generator.
 */
#include <stdbool.h>
#include <stdio.h>

#include "os_specific.h"

#include "uuid.h"

static const char *uuid_gen_dash_flag(char *uuidbuf, size_t buflen, bool withdashes)
{
    unsigned int uuidint32[128/32];

    /* Generate random 128 bit buffer */
    for (size_t index = 0; index < sizeof(uuidint32) / sizeof(uuidint32[0]); ++index) {
        uuidint32[index] = os_urandom();
    }

    const char *dash = withdashes ? "-" : "";

    snprintf(uuidbuf, buflen, "%08x%s%04x%s%04x%s%04x%s%04x%08x",
        uuidint32[0],
        dash,
        uuidint32[1]>>16,
        dash,
        uuidint32[1]&0xFFFF,
        dash,
        uuidint32[2]>>16,
        dash,
        uuidint32[2]&0xFFFF,
        uuidint32[3]);

printf("***********************************************************************\n");
printf("%s: sizeof(uuid_text_t) %d sizeof(uuid_text_trimmed_r) %d buflen %d  '%s'\n", __func__, sizeof(uuid_text_t), sizeof(uuid_text_trimmed_t), buflen, uuidbuf);
printf("***********************************************************************\n");

    return uuidbuf;
}

const char *uuid_gen(uuid_text_t buf)
{
    return uuid_gen_dash_flag(buf, sizeof(uuid_text_t), true);
}

const char *uuid_gen_trimmed(uuid_text_trimmed_t buf)
{
    return uuid_gen_dash_flag(buf, sizeof(uuid_text_trimmed_t), false);
}

