/*
 * UUID generator.
 */

#ifndef __uuid_h_included
#define __uuid_h_included

#define UUID_BUF_LEN            (8+1+4+1+4+1+4+1+12+1) /* XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX */
#define UUID_BUF_TRIMMED_LEN    (8+4+4+4+12+1)         /* XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX */

typedef char     uuid_text_t[UUID_BUF_LEN];
typedef char     uuid_text_trimmed_t[UUID_BUF_TRIMMED_LEN];

const char *uuid_gen(uuid_text_t uuid_buf);
const char *uuid_gen_trimmed(uuid_text_trimmed_t uuid_buf);

#endif /* __uuid_h_included */

