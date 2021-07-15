/*
 * commands.h
 *
 */
#ifndef __commands_h_included
#define __commands_h_included

#include <stdarg.h>

os_thread_t start_commands(FILE* in, FILE* out);

#ifdef CONFIG_LASTLINK_COMMAND_INTERFACE
typedef struct {
    /* The context transaction_id number */
    int transaction_id;

    /*
     * True if spawned as separate process.
     * If so, input_queue and thread_id are defined.
     * argv has been copied, so must be released when release_context is called.
     */
    bool spawned;

    /* An input queue for data from command processor */
    os_queue_t input_queue;

    /* Thread id of spawned task if non-NULL */
    os_thread_t thread_id;

    /* The argc/argv from invocation */
    int argc;
    const char **argv;

    /* If asked to terminate by command processor */
    bool terminate;

    /* Extra data for server */
    void *param;

    /* Any results code from command */
    int results;
} command_context_t;
#else
/* For the few standalone cases of prints and such */
typedef void* command_context_t;
#endif

void init_commands(void);
void deinit_commands(void);

typedef enum {
    COMMAND_ONCE=0,
    COMMAND_SPAWN,
} command_mode_t;

bool add_command(const char* name, void (*function)(command_context_t* context), command_mode_t mode);
bool remove_command(const char* name);
void show_help(command_context_t* context, const char *params, const char *description);

// command_context_t* command_read(char *buffer, size_t len);
void command_reply(command_context_t* context, const char* type, const char* fmt, ...);
char* command_read_more_with_timeout(command_context_t* context, int timeout);
char* command_read_more(command_context_t* context);

void context_release(command_context_t* context);

/* Return true if a termination request is pending */
bool context_check_termination_request(command_context_t* context);

bool hit_test(int testch);

#define COMMAND_RC_DO_NOT_RELEASE_CONTEXT   -9999999

#endif /* __commands_h_included */

