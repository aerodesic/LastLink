/*
 * A simple command processor called with a pair of file descriptors for communication.
 */
#include "sdkconfig.h"

#ifdef CONFIG_LASTLINK_COMMAND_INTERFACE

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "os_specific.h"
#include "lsocket.h"
#include "commands.h"
#include "linklayer.h"
#include "configdata.h"

#ifdef CONFIG_LASTLINK_CHECKSUMMED_COMMAND_INTERFACE
#include <termios.h>
#include "esp_crc.h"
#endif

#ifdef CONFIG_SSD1306_I2C_ENABLED
#include "display.h"
#endif /* CONFIG_SSD1306_I2C_ENABLED */

#include "listops.h"
#include "tokenize.h"

#define TAG "commands"

#define COMMAND_PROCESSOR_STACK_SIZE 8192

static list_head_t     command_table;
static os_mutex_t      command_lock;

#ifdef CONFIG_LASTLINK_CHECKSUMMED_COMMAND_INTERFACE
/*
 * implement a checksum protected command_read:
 *   '$'<data>:<checksum>\r
 */
static int command_read(char *buffer, size_t len)
{
    int checksum_position = 0;

    int index = 0;

    bool reading = true;
    enum { SOM, DATA, CHECKSUM } state = SOM;

    struct termios tty_opts_backup, tty_opts_raw;

    // Back up current TTY settings
    tcgetattr(STDIN_FILENO, &tty_opts_backup);

    // Change TTY settings to raw mode
    tty_opts_raw = tty_opts_backup;
    tty_opts_raw.c_lflag = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &tty_opts_raw);

    while (reading) {
        int ch = getchar();
        if (ch < 0) {
            os_delay(5);
        } else {
            if (ch == '$') {
                state = SOM;
                // printf("SOM\n");
            }

            switch (state) {
                case SOM:
                default: {
                    state = SOM;
                    if (ch == '$') {
                        state = DATA;
                        index = 0;
                        // printf("DATA\n");
                    }
                    break;
                }

                case DATA: {
                    if (ch == ';') {
                        state = CHECKSUM;
                        checksum_position = index;
                        // printf("DATA at %d\n", checksum_position);
                    } else if (index < len - 1) {
                    buffer[index++] = ch;
                    } else {
                        /* Message too long - go back to waiting for SOM */
                        state = SOM;
                        // printf("overflow\n");
                    }
                    break;
                }

                case CHECKSUM: {
                    if (ch == '\n') {
                        buffer[index] = '\0';

                        /* Calculate checksum and verify message */
                        unsigned int crc_wanted = strtol(buffer + checksum_position, NULL, 16);
                        unsigned int crc_found = esp_crc16_le(0x0000, (const uint8_t *) buffer, checksum_position);
                        buffer[checksum_position] = '\0';

                        if (crc_wanted == crc_found) {
                            reading = false;
                        } else {
                            ESP_LOGE(TAG, "%s: crc failed: wanted %04x found %04x", __func__, crc_wanted, crc_found);
#ifdef CONFIG_LASTLINK_CHECKSUMMED_COMMAND_INTERFACE_IGNORE_CHECKSUM
                            reading = false;     /* Tread bad checksum as good checksum */
#else
                            /* checksum failed - try again */
                            state = SOM;
#endif
                        }
                    } else if (index < len - 1) {
                        buffer[index++] = ch;
                    } else {
                        state = SOM;
                    }
                    break;
                }
            }
        }
    }

    // Restore previous TTY settings
    tcsetattr(STDIN_FILENO, TCSANOW, &tty_opts_backup);

    return index;
}

void command_reply(command_context_t* context, const char* fmt, ...)
{
    char buffer[256];
    va_list args;

    // Create the header
    int len = sprintf(buffer, "%d:", context ? context->id : 0);

    va_start(args, fmt);
    len += vsnprintf(buffer + len, sizeof(buffer) - len, fmt, args);
    va_end (args);

    os_acquire_recursive_mutex(command_lock);
    printf("$%s;%04x\n", buffer, esp_crc16_le(0x0000, (const uint8_t*) buffer, len));
    os_release_recursive_mutex(command_lock);
}

void command_reply_error(command_context_t* context, const char* fmt, ...)
{
    char buffer[256];
    va_list args;

    // Create the header
    int len = sprintf(buffer, "%d:", context ? context->id : 0);
 
    va_start(args, fmt);
    len += vsnprintf(buffer + len, sizeof(buffer) - len, fmt, args);
    va_end (args);

    os_acquire_recursive_mutex(command_lock);
    printf("$%s;%04x\n", buffer, esp_crc16_le(0x0000, (const uint8_t*) buffer, len));
    os_release_recursive_mutex(command_lock);
}

#else /* ! CONFIG_LASTLINK_CHECKSUMMED_COMMAND_INTERFACE */

/*
 * Simple command_read interface.
 * Return number of characters received.  -1 if interrupted by control-c
 */
static int command_read(char *buffer, size_t len)
{
    int index = 0;

    bool reading = true;

    while (reading) {
        int ch = getchar();

        if (ch < 0) {
            os_delay(5);
        } else {
            if (ch == '\n') {
                buffer[index] = '\0';
                reading = false;
                printf("\n");
            } else if (ch == '\b') {
                if (index > 0) {
                    printf("\b \b");
                    --index;
                }
            } else if ((ch & 0x7F) == '\x03') {
                index = -1;
                reading = false;
            } else if (isprint(ch)) {
                if (index < len-1) {
                    printf("%c", ch);
                    buffer[index++] = ch;
                }
            }
        }
    }

    return index;
}

void command_reply(command_context_t* context, const char *fmt, ...)
{
    (void) context;

    char buffer[256];
    va_list args;

    int len = 0;

    // Create the header if non-zero id
    if (context && context->id != 0) {
        len = sprintf(buffer, "%d:", context ? context->id : 0);
    }
 
    va_start(args, fmt);
    len += vsnprintf(buffer + len, sizeof(buffer) - len, fmt, args);
    va_end (args);

    os_acquire_recursive_mutex(command_lock);
    printf("%s\n", buffer);
    os_release_recursive_mutex(command_lock);
}

void command_reply_error(command_context_t* context, const char *fmt, ...)
{
    (void) context;

    char buffer[256];
    va_list args;

    int len = 0;

    // Create the header if non-zero id
    if (context && context->id != 0) {
        len = sprintf(buffer, "%d:", context ? context->id : 0);
    }
 
    va_start(args, fmt);
    len += vsnprintf(buffer + len, sizeof(buffer) - len, fmt, args);
    va_end (args);

    os_acquire_recursive_mutex(command_lock);
    printf("%s\n", buffer);
    os_release_recursive_mutex(command_lock);
}

#endif /* CONFIG_LASTLINK_CHECKSUMMED_COMMAND_INTERFACE */

/*
 * Absorb all characters in queue looking for match with testch.
 * Return true if one seen.
 */
bool hit_test(int testch)
{
    bool found = false;
    int ch;

    while ((ch = getchar()) >= 0) {
        if (ch == testch) {
            found = true;
        }
    }

    return found;
}

bool context_check_termination_request(command_context_t* context)
{
    if (context->spawned) {
        return context->terminate;
    } else {
        return hit_test('\x03');
    }
}

typedef struct command_param {
    FILE* in;
    FILE* out;
} command_param_t;

static void CommandProcessor(void* params);

/* Start a command processor on the specific fd pair */
os_thread_t start_commands(FILE* in, FILE* out)
{
    command_param_t *param = NULL;

    if (in != NULL && out != NULL) {
        param = (command_param_t*) malloc(sizeof(command_param_t));

        if (param == NULL) {
            printf("Unable to allocate command parameter; starting with default stdin/stdout\n");
        } else {
            param->in = in;
            param->out = out;
        }
    }

    return os_create_thread(CommandProcessor, "commands", COMMAND_PROCESSOR_STACK_SIZE, 10, param);
}

#define MAX_ARGS  20

#define HELP_CMD_FIELD_LEN   30

void show_help(command_context_t* context, const char *params, const char *description)
{
    command_reply(context, "%s %-*s %s", context->argv[0], HELP_CMD_FIELD_LEN - strlen(context->argv[0]), params, description);
}

/**********************************************************************/
/* echo <blah> <blah> <blah>                                          */
/**********************************************************************/
static void echo_command(command_context_t *context)
{
    if (context->argc == 0) {
        show_help(context, "<blah blah>..", "Echo arguments for testing");
    } else {
        for (int arg = 0; arg < context->argc; ++arg) {
            command_reply(context, "arg %d '%s'", arg, context->argv[arg]);
        }
    }
}


/**********************************************************************/
/* memory                                                             */
/**********************************************************************/
static void memory_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "", "Display memory usage");
    } else {
        command_reply(context, "Free Heap:     %08x (%d)", xPortGetFreeHeapSize(), xPortGetFreeHeapSize());
        command_reply(context, "Largest Free:  %08x (%d)", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        command_reply(context, "Minimum Free:  %08x (%d)", xPortGetMinimumEverFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
    }
}

/**********************************************************************/
/* address [ <address> ]                                              */
/**********************************************************************/
static void address_command(command_context_t *context)
{
    if (context->argc == 0) {
        show_help(context, "", "Show address if no parameters");
        show_help(context, "<address>", "Set address to <address>");
    } else if (context->argc > 1) {
        int address = strtol(context->argv[1], NULL, 10);
        if (address >= 1 && address <= 500) {
            linklayer_node_address = address;
        }
    } else {
        command_reply(context, "Address: %d", linklayer_node_address);
    }
}

/**********************************************************************/
/* loglevel <group> <level>                                           */
/**********************************************************************/
/*
 * <group> <level>
 */
typedef struct log_level {
    const char *name;
    int        level;
} log_level_t;

static const log_level_t  log_levels[] = {
    { .name = "none",    .level = ESP_LOG_NONE    },
    { .name = "error",   .level = ESP_LOG_ERROR   },
    { .name = "warn",    .level = ESP_LOG_WARN    },
    { .name = "info",    .level = ESP_LOG_INFO    },
    { .name = "debug",   .level = ESP_LOG_DEBUG   },
    { .name = "verbose", .level = ESP_LOG_VERBOSE },
};

#ifndef ELEMENTS_OF
#define ELEMENTS_OF(x)   (sizeof(x) / sizeof(0[x]))
#endif

static void loglevel_command(command_context_t *context)
{
    if (context->argc == 0) {
        show_help(context, "<tag> <loglevel>", "Set log level for tag");
    } else if (context->argc > 2) {
        bool found = false;
        int level;

        for (int index = 0; !found && index < ELEMENTS_OF(log_levels); ++index) {
            if (strcasecmp(log_levels[index].name, context->argv[2]) == 0) {
                level = log_levels[index].level;
                found = true;
            }
        }

        if (found) {
            esp_log_level_set(context->argv[1], level);
        } else {
            command_reply_error(context, "Bad level name '%s'", context->argv[2]);
        }
    } else {
        command_reply_error(context, "Missing args");
    }
}

#ifdef CONFIG_SSD1306_I2C_ENABLED
/**********************************************************************/
/* contrast <val>                                                     */
/**********************************************************************/
static void contrast_command(command_context_t* context)
{
    extern display_t *display;

    if (context->argc == 0) {
        show_help(context, "<value>", "Set display contrast (brightness)");
    } else if (context->argc == 2) {
        int val = strtol(context->argv[1], NULL, 10);
        if (val < 0 || val > 255) {
            command_reply_error(context, "Value must be from 0 to 255");
        } else {
            display->contrast(display, val);
        }
    }
}
#endif /* CONFIG_SSD1306_I2C_ENABLED */

/**********************************************************************/
/* reboot                                                             */
/**********************************************************************/
static void reboot_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "", "Reboot device");
    } else {
        esp_restart();
    }
}

typedef struct command_entry command_entry_t;
typedef struct command_entry {
    command_entry_t *next;
    command_entry_t *prev;
    const char*      name;
    void             (*function)(command_context_t* context);
    command_mode_t   mode;
} command_entry_t;



static command_entry_t *find_command(const char *name)
{
    command_entry_t *command_entry = NULL;

    os_acquire_recursive_mutex(command_lock);

    command_entry_t *command = (command_entry_t*) FIRST_LIST_ITEM(&command_table);

    while (command != NULL) {
        if (strcmp(command->name, name) == 0) {
            command_entry = command;
        }

        command = NEXT_LIST_ITEM(command, &command_table);
    }

    os_release_recursive_mutex(command_lock);

    return command_entry;
}

/**********************************************************************/
/* help                                                               */
/**********************************************************************/
static void help_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "", "Show help");
    } else {
        os_acquire_recursive_mutex(command_lock);

        command_entry_t *command = (command_entry_t*) FIRST_LIST_ITEM(&command_table);

        while (command != NULL) {

            const char *help_argv[2] = {
                command->name,
                NULL,
            };

            command_context_t help_context = {
                .argc = 0,
                .argv = help_argv,
            };

            command->function(&help_context);

            command = NEXT_LIST_ITEM(command, &command_table);
        }

        os_release_recursive_mutex(command_lock);
    }
}

static const char* task_state(eTaskState state)
{
    switch (state) {
       case eReady:     return "Ready";
       case eRunning:   return "Running";
       case eBlocked:   return "Blocked";
       case eSuspended: return "Suspended";
       case eDeleted:   return "Deleted";
       default:         return "Unknown";
    }
}

/*
 * time
 *
 * Display time in ms
 */
static void time_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "", "Show current time int mS since boot");
    } else {
        command_reply(context, "%llu", get_milliseconds());
    }
}

static void kill_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "<handle>", "Kill a task");
    } else {
        if (context->argc > 1) {
            unsigned int handle = strtol(context->argv[1], NULL, 16);

            command_reply(context, "Killing thread %x", handle);

            os_delete_thread((os_thread_t) handle);

        } else {
            command_reply_error(context, "Insufficient args");
        }
    }
}

static void ps_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "", "List all tasks");
    } else {
        TaskStatus_t *pxTaskStatusArray;
        UBaseType_t uxArraySize;

        /* Take a snapshot of the number of tasks in case it changes while this
        function is executing. */
        uxArraySize = uxTaskGetNumberOfTasks();

        /* Allocate a TaskStatus_t structure for each task.  An array could be
        allocated statically at compile time. */
        pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

        if( pxTaskStatusArray != NULL ) {
            uint32_t ulTotalRunTime;

            /* Generate raw status information about each task. */
            BaseType_t num_tasks = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

            /* For each populated position in the pxTaskStatusArray array,
            format the raw data as human readable ASCII data. */
            command_reply(context, "Task Name         Handle  Runtime  State      Priority  Cpu  High Stack");
            for( size_t x = 0; x < num_tasks; x++ ) {
                char runtime[20];
                sprintf(runtime, "%u.%u", pdTICKS_TO_MS(pxTaskStatusArray[x].ulRunTimeCounter) / 1000, (pdTICKS_TO_MS(pxTaskStatusArray[x].ulRunTimeCounter) / 100) % 10);

                command_reply(context, "%-16s  %-6x  %-7s  %-9s  %-8u  %-3d  %-5d",
                       pxTaskStatusArray[x].pcTaskName,
                       (unsigned int) pxTaskStatusArray[x].xHandle,
                       runtime,
                       task_state(pxTaskStatusArray[x].eCurrentState),
                       pxTaskStatusArray[x].uxCurrentPriority,
                       pxTaskStatusArray[x].xCoreID > 32 ? -1 : pxTaskStatusArray[x].xCoreID,
                       pxTaskStatusArray[x].usStackHighWaterMark);
            }
        }

        /* The array is no longer needed, free the memory it consumes. */
        vPortFree( pxTaskStatusArray );
    }
}

#ifdef NOTUSED
typedef struct spawn_param {
    int (*command)(command_context_t* context);
    command_context_t* context;
    const char **argv;
    const char *args;
    int argc;
} spawn_param_t;

/*
 * Helper thread for spawn command
 */
void spawn_thread(void *param)
{
    spawn_param_t *spawn_params = (spawn_param_t*) param;

    int results = spawn_params->command(spawn_params->context);

    command_reply(spawn_params->context, "exit with %d", results);

    context_release(spawn_params->context);

    os_exit_thread();
}

/*
 * Spawn another command to run in the background
 */
int spawn_command(command_context_t* context)
{
    /* Special results that says do not delete context when exit */
    int results = COMMAND_RC_DO_NOT_RELEASE_CONTEXT;

    if (context->argc == 0) {
        show_help(context, "<command> <params>...", "spawn command as background thread");
    } else if (context->argc < 2) {
        command_reply_error(context, "Insufficient args");
    } else {
        command_entry_t *command = find_command(context->argv[1]);
        if (command == NULL) {
            command_reply_error(context, "command '%s' not found", context->argv[1]);
            results = -1;
        } else {
            /* Compute length of param table */

            int arg;
            int args_length = 0;

            /* Calculate amount of string space needed */
            for (arg = 1; arg < context->argc; ++arg)  {
                args_length += strlen(context->argv[arg]) + 1;
            }

            spawn_param_t *spawn_params = (spawn_param_t*) malloc(sizeof(spawn_param_t));

            /* context */
            spawn_params->context = context;

            /* Argv array */
            spawn_params->argv = (const char **) malloc(context->argc * sizeof(const char*)); /* Room for all args + 1 for NULL */

            /* Point to command */
            spawn_params->command = command->function;

            /* Number of args - 1 from spawn  command */
            spawn_params->argc = context->argc - 1;

            /* Where the parameter strings go */
            char *params = (char*) malloc(args_length);

            /* So we can free it when we are done */
            spawn_params->args = params;

            for (arg = 1; arg < context->argc; ++arg) {
                /* Copy strings of params */
                spawn_params->argv[arg - 1] = (const char*) params;
                int len = strlen(context->argv[arg]);
                strcpy(params, context->argv[arg]);
                params += len;
                *params++ = '\0';
            }

            spawn_params->argv[arg - 1] = NULL;  /* Last argv is a NULL */

            /* Spawn with name of command */
            os_create_thread(spawn_thread, context->argv[1], 8192, 0, (void*) spawn_params);
        }
    }

    context->reults = results;
}
#endif

#ifdef CONFIG_DHT_ENABLE
#include "dht.h"

void dht_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "", "Read RH and Temp for DHT sensor");
    } else {
        int count = 1;
        if (context->argc > 1) {
            count = strtol(context->argv[1], NULL, 10);
        }
        while (count > 0) {
            dht_value_t rh;
            dht_value_t temp;
            dht_ret_t rc = dht_read(&rh, &temp);
            if (rc == 0) {
                command_reply(context, "rh %.1f%% temp %.1f deg", rh, temp);
            } else {
                command_reply_error(context, "dht error %d", rc);
            }
            if (--count > 0) {
                os_delay(2000);
            }
        }
    }
}
#endif /* CONFIG_DHT_ENABLE */

static command_context_t command_contexts[CONFIG_LASTLINK_COMMAND_MAX_CONTEXTS];

static command_context_t* find_context(int id)
{
    command_context_t* context = NULL;

    for (int index = 0; context == NULL && index < CONFIG_LASTLINK_COMMAND_MAX_CONTEXTS; ++index) {
        if (command_contexts[index].id == id) {
            context = &command_contexts[index];
        }
    }

    return context;
}

static int global_context_id;

static command_context_t* create_context(bool spawned, int argc, const char** argv)
{
    command_context_t* context = find_context(0);

    if (context != NULL) {
        context->id = ++global_context_id;
        context->results = 0;
        context->terminate = false;
        context->param = NULL;

        if (spawned) {
            context->spawned = true;
            context->input_queue = os_create_queue(CONFIG_LASTLINK_COMMAND_MAX_PENDING_REQUESTS, sizeof(const char*));
            /* Make private copy of argv array */

            /* Special case: if argc is 0, then allow it to be 1 (the first argv is the function name in any case).
             * (However, this will probably never be a problem as argc=0 only happens in the case of the 'help'
             * activation of a function and that doesn't happen in 'spawn' mode.
             */
             
//ESP_LOGI(TAG, "%s: copy %d args for %s", __func__, argc, argv[0]);

            int eff_argc = argc ? argc : 1;

            /* Base size includes the argv overhead */
            size_t size = sizeof(const char*) * (eff_argc+1);
            for (int arg = 0; arg < eff_argc; ++arg) {
                size += strlen(argv[arg]) + 1;
            }

            char* args = (char*) malloc(size);
 
            if (args != NULL) {
                /* Where the pointers are stored */
                context->argv = (const char**) args;

                /* Where the strings are stored */
                char* dest = args + sizeof(const char*) * (eff_argc + 1);

                /* Copy the args */
                for (int arg = 0; arg < eff_argc; ++arg) {
                    context->argv[arg] = dest;
                    strcpy(dest, argv[arg]);
                    dest += strlen(argv[arg]) + 1;
                }

                /* Store terminating NULL */
                context->argv[eff_argc] = NULL; 
            }
        } else {
            context->argv = argv;
            context->spawned = false;
            context->thread_id = NULL;
            context->input_queue = NULL;
        }

        context->argc = argc;       
    }

    return context;
}

void context_release(command_context_t* context)
{
    if (context != NULL) {
//ESP_LOGI(TAG, "%s: %p id %d spawned %d", __func__, context, context->id, context->spawned);
        if (context->spawned) {
            if (context->input_queue != NULL) {
                os_delete_queue(context->input_queue);
                context->input_queue = NULL;
            }

            free((void*) context->argv);
            context->argv = NULL;
            context->argc = 0;
            context->thread_id = NULL;
            context->terminate = false;
            context->spawned = false;
        }

        context->id = 0;
    }
}

char* command_read_more_with_timeout(command_context_t* context, int timeout)
{
    char* message = NULL;

    if (context->input_queue != NULL) {
        os_get_queue_with_timeout(context->input_queue, (os_queue_item_t) &message, timeout);
    }

    return message;
}

char* command_read_more(command_context_t* context)
{
    return command_read_more_with_timeout(context, -1);
}


static void contexts_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "", "Show list of contexts");
    } else {
        for (int c = 0; c < CONFIG_LASTLINK_COMMAND_MAX_CONTEXTS; ++c) {
            command_reply(context, "%p: id %d spawned %s thread_id %p input_queue %p argc %d param %p terminate %d results %d",
                          command_contexts + c,
                          command_contexts[c].id,
                          command_contexts[c].spawned ? "Yes" : "No",
                          command_contexts[c].thread_id,
                          command_contexts[c].input_queue,
                          command_contexts[c].argc,
                          command_contexts[c].param,
                          command_contexts[c].terminate,
                          command_contexts[c].results);
        }
    }
}

   
void CommandProcessor(void* params)
{
    if (params != NULL) {
        command_param_t *io = (command_param_t*) params;

        ESP_LOGD(TAG, "CommandProcess in %d out %d", fileno(io->in), fileno(io->out));

        stdin = io->in;
        stdout = io->out;

        free((void*) io);
        io = NULL;
    }

    bool running = true;

    ESP_LOGD(TAG, "stdin is %p fd %d  stdout is %p fd %d", stdin, fileno(stdin), stdout, fileno(stdout));

    ESP_LOGI(TAG, "********************************************************\n");
    ESP_LOGI(TAG, "CommandProcessor running\n");
    ESP_LOGI(TAG, "********************************************************\n");

    while (running) {
        char buffer[100];

#ifndef CONFIG_LASTLINK_CHECKSUMMED_COMMAND_INTERFACE
        printf(">> ");
#endif

        int len = command_read(buffer, sizeof(buffer));

        if (len > 0) {
            char *p = skip_blanks(buffer);

            if (isdigit(*p)) {
                /* This is directed input to a running context.  Look up the
                 * the context and send this to its input queue.
                 */
                int id = strtol(p, &p, 10);

                if (id == 0) {
                    command_reply_error(NULL, "bad context");
                } else {
                    command_context_t* context = find_context(id);
//printf("context is %p\n", context);
                    if (context != NULL) {
                        /* Skip past delimiter if one is present  */
                        if (*p == ':' || *p == ' ') {
                            ++p;
                        }
    
                        /* Add message to input queue */
                        char *p_copy = strdup(p);
                        if (!os_put_queue_with_timeout(context->input_queue, (os_queue_item_t) &p_copy, 0)) {
                            command_reply_error(context, "input full");
                            free((void*) p_copy);
                        }
                    } else {
//printf("before reply\n");
                        command_reply_error(NULL, "unknown context");
//printf("after reply\n");
                    }
                }
            } else { 
                /* New command  - look it up, create a context and issue the command */
                const char* args[MAX_ARGS];
                int argc = tokenize(p, args, MAX_ARGS);

                if (argc >= 1) {
                    /* Look for context id number as first item */
                    void (*function)(command_context_t* context) = NULL;
                    command_mode_t mode = COMMAND_ONCE;
    
                    if (argc > 0) {
                        command_entry_t *command = find_command(args[0]);
                        if (command != NULL) {
                            function = command->function;
                            mode = command->mode;
                        }
                    }
    
                    os_release_recursive_mutex(command_lock);
    
                    if (function != NULL) {

                        os_acquire_recursive_mutex(command_lock);

                        int rc = 0;

                        if (mode == COMMAND_ONCE) {
                            command_context_t once_context = {
                                .argc = argc,
                                .argv = args,
                            };


                            (*function)(&once_context);
                            rc = once_context.results;

                        } else {
                            command_context_t* context = create_context(mode == COMMAND_SPAWN, argc, args);
                            if (context == NULL) {
                                command_reply_error(NULL, "out of contexts");
                            
                            } else {
                                /* Run the command as spawned */
                                command_reply(context, "ack %s", context->argv[0]);

                                char command_name[20];
                                snprintf(command_name, sizeof(command_name), "%s_%d", context->argv[0], context->id);
                                context->thread_id = os_create_thread((void (*)(void*)) function, command_name, 8192, 0, (void*) context);

                                if (context->thread_id == NULL) {
                                    rc = -1;
                                    context_release(context);
                                } 
                            }
                        }

                        os_release_recursive_mutex(command_lock);

                        if (rc != 0) {
                            command_reply_error(NULL, "Error: %d", rc);
                        }
                    } else if (argc > 0) {
                        command_reply_error(NULL, "Invalid command: %s", args[0]);
                    }
                }
            }
        } else if (len < 0) {
            command_reply(NULL, "Control-c detected");
        }
    }

    ESP_LOGI(TAG, "********************************************************\n");
    ESP_LOGI(TAG, "CommandProcessor exiting\n");
    ESP_LOGI(TAG, "********************************************************\n");

    os_exit_thread();
}

bool add_command(const char* name, void (*function)(command_context_t* context), command_mode_t mode)
{
    command_entry_t *entry = (command_entry_t*) malloc(sizeof(command_entry_t));
    if (entry != NULL) {
        entry->name = name;
        entry->function = function;
        entry->mode = mode;

        ADD_TO_LIST(&command_table, entry);
    }

    return entry != NULL;
}

static void delete_command(command_entry_t *command_entry)
{
    if (command_entry != NULL) {
        /* Remove from list */
        REMOVE_FROM_LIST(&command_table, command_entry);
        free((void*) command_entry);
    }
}

bool remove_command(const char *name)
{
    os_acquire_recursive_mutex(command_lock);

    command_entry_t *command_entry = find_command(name);
    if (command_entry != NULL) {
        delete_command(command_entry);
    }

    os_release_recursive_mutex(command_lock);

    return command_entry != NULL;
}

static void linklayer_set_inactive_command(command_context_t* context)
{
    if (context->argc == 0) {
        show_help(context, "<mode>", "set linklayer inactive (1) or active(0)");
    } else if (context->argc > 1) {
        int inactive = strtol(context->argv[1], NULL, 10);
        bool rc = linklayer_set_inactive(inactive);
        if (! rc) {
            ESP_LOGE(TAG, "linklayer_set_inactive returned false");
        }
    }
}

void init_commands(void)
{
    command_lock = os_create_recursive_mutex();

    os_acquire_recursive_mutex(command_lock);

    add_command("help",        help_command,                    COMMAND_ONCE);
    add_command("?",           help_command,                    COMMAND_ONCE);
    add_command("mem",         memory_command,                  COMMAND_ONCE);
#ifdef CONFIG_SSD1306_I2C_ENABLED
    add_command("contrast",    contrast_command,                COMMAND_ONCE);
#endif /* CONFIG_SSD1306_I2C_ENABLED */
    add_command("echo",        echo_command,                    COMMAND_ONCE);
    add_command("address",     address_command,                 COMMAND_ONCE);
    add_command("loglevel",    loglevel_command,                COMMAND_ONCE);
    add_command("reboot",      reboot_command,                  COMMAND_ONCE);
    add_command("ps",          ps_command,                      COMMAND_ONCE);
    add_command("kill",        kill_command,                    COMMAND_ONCE);
    add_command("time",        time_command,                    COMMAND_ONCE);
#ifdef NOTUSED
    add_command("spawn",       spawn_command,                   COMMAND_SPAWN);
#endif
    add_command("contexts",    contexts_command,                COMMAND_ONCE);
#ifdef CONFIG_DHT_ENABLE
    add_command("dht",         dht_command,                     COMMAND_ONCE);
#endif
    add_command("setinactive", linklayer_set_inactive_command,  COMMAND_ONCE);

    os_release_recursive_mutex(command_lock);
}

void deinit_commands(void)
{
    os_acquire_recursive_mutex(command_lock);

    while (!IS_LIST_EMPTY(&command_table)) {
        delete_command((command_entry_t*) FIRST_LIST_ITEM(&command_table));
    }

    os_release_recursive_mutex(command_lock);

    os_delete_mutex(command_lock);

    command_lock = NULL;
}

#endif /* CONFIG_LASTLINK_COMMAND_INTERFACE */

