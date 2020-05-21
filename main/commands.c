/*
 * A simple command processor called with a pair of file descriptors for communication.
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "os_freertos.h"
#include "lsocket.h"
#include "commands.h"
#include "linklayer.h"
#include "configdata.h"

#include "display.h"
#include "listops.h"

#define TAG "commands"

#define COMMAND_PROCESSOR_STACK_SIZE 40000
#define PING_COMMAND_STACK_SIZE      4000

/*
 * Go through buffer and tokenize into argv/argc structure.
 */
static int tokenize(char *buffer, const char**args, int maxargs)
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
    args[argc] = NULL;

    return argc;
}

static int readline(char *buffer, size_t len)
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
            } else if (isprint(ch)) {
                if (index < len-1) {
                    printf("%c", ch);
                    buffer[index++] = ch;
                }
            }
        }
    }

//printf("readline returning '%s'\n", buffer);
    return index;
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

void show_help(const char* name, const char *params, const char *description)
{
    printf("%s %-*s %s\n", name, HELP_CMD_FIELD_LEN - strlen(name), params, description);
}

typedef struct {
    int address;
    FILE* out;
} ping_info_t;

void ping_command_thread(void* param)
{
    ping_info_t *info = (ping_info_t*) param;

    int address = info->address;

    /* Set stdout for printing */
    stdout = info->out;

    free((void*) info);

    // printf("Test message from ping_command_thread\n");

    int paths[100];
    int path_len = ping(address, paths, ELEMENTS_OF(paths));

    if (path_len < 0) {
        printf("Ping error %d\n", path_len);
    } else {
        printf("Path:");
        for (int path = 0; path < path_len; ++path) {
            printf(" %d", paths[path]);
        }

        printf("\n");
    }

    os_exit_thread();
}

/**********************************************************************/
/* ping <node address>                                                */
/**********************************************************************/
static int ping_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<node address>", "Find and report path to a node");
    } else {
        int address = strtol(argv[1], NULL, 10);

        ping_info_t *info = (ping_info_t*) malloc(sizeof(ping_info_t));
        if (info != NULL) {
            info->address = address;
            info->out = stdout;
            os_create_thread(ping_command_thread, "ping", PING_COMMAND_STACK_SIZE, 10, (void*) info);
        } else {
            printf("Unable to create ping info object\n");
        }
    }

    return 0;
}

/**********************************************************************/
/* test <blah> <blah> <blah>                                          */
/**********************************************************************/
static int echo_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<blah blah>..", "Echo arguments for testing");
    } else {
        for (int arg = 0; arg < argc; ++arg) {
            printf("arg %d '%s'\n", arg, argv[arg]);
        }
    }

    return 0;
}


/**********************************************************************/
/* memory                                                             */
/**********************************************************************/
static int memory_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Display memory usage");
    } else {
        printf("Free Heap:     %08x (%d)\n", xPortGetFreeHeapSize(), xPortGetFreeHeapSize());
        printf("Largest Free:  %08x (%d)\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        printf("Minimum Free:  %08x (%d)\n", xPortGetMinimumEverFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
    }
    return 0;
}

/**********************************************************************/
/* address [ <address> ]                                              */
/**********************************************************************/
static int address_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Show address if no parameters");
        show_help(argv[0], "<address>", "Set address to <address>");
    } else if (argc > 1) {
        int address = strtol(argv[1], NULL, 10);
        if (address >= 1 && address <= 500) {
            linklayer_node_address = address;
        }
    } else {
        printf("Address: %d\n", linklayer_node_address);
    }

    return 0;
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

static int loglevel_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<tag> <loglevel>", "Set log level for tag");
    } else if (argc > 2) {
        bool found = false;
        int level;

        for (int index = 0; !found && index < ELEMENTS_OF(log_levels); ++index) {
            if (strcasecmp(log_levels[index].name, argv[2]) == 0) {
                level = log_levels[index].level;
                found = true;
            }
        }

        if (found) {
            esp_log_level_set(argv[1], level);
        } else {
            printf("Bad level name '%s'\n", argv[2]);
        }
    } else {
        printf("Missing args\n");
    }

    return 0;
}

/**********************************************************************/
/* config [ <var> [ <value> ] ]                                       */
/**********************************************************************/
static int config_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Show all config vars");
        show_help(argv[0], "<var>", "Show value of config var");
        show_help(argv[0], "<var> <value>", "Set value of config var");
    } else if (argc == 1) {
        write_config(stdout);
    } else {
        const char* var = argv[1];
        if (argc > 2) {
            lock_config();
            set_config_str(var, argv[2]);
            unlock_config();
        } else {
            printf("%s = '%s'\n", var, get_config_str(var, "**UNDEFINED**"));
        }
    }

    return 0;
}


/**********************************************************************/
/* contrast <val>                                                     */
/**********************************************************************/
static int contrast_command(int argc, const char **argv)
{
    extern display_t *display;

    if (argc == 0) {
        show_help(argv[0], "<value>", "Set display contrast (brightness)");
    } else if (argc == 2) {
        int val = strtol(argv[1], NULL, 10);
        if (val < 0 || val > 255) {
            printf("Value must be from 0 to 255\n");
        } else {
            display->contrast(display, val);
        } 
    }

    return 0;
}

/**********************************************************************/
/* reboot                                                             */
/**********************************************************************/
static int reboot_command(int argc, const char **argv)
{
    extern display_t *display;

    if (argc == 0) {
        show_help(argv[0], "", "Reboot device");
    } else {
        esp_restart();
    }

    return 0;
}

typedef struct command_entry command_entry_t;
typedef struct command_entry {
    command_entry_t *next; 
    command_entry_t *prev;
    const char*      name;
    int              (*function)(int argc, const char **argv);
} command_entry_t;


static list_head_t     command_table;
static os_mutex_t      command_lock;


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
static int help_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Show help");
    } else {
        os_acquire_recursive_mutex(command_lock);


        command_entry_t *command = (command_entry_t*) FIRST_LIST_ITEM(&command_table);

        while (command != NULL) {

            const char *argv[2] = { command->name, NULL };
            command->function(0, argv);

            command = NEXT_LIST_ITEM(command, &command_table);
        }

        os_release_recursive_mutex(command_lock);
    }

    return 0;
}


void CommandProcessor(void* params)
{
    if (params != NULL) {
        command_param_t *io = (command_param_t*) params;

        ESP_LOGE(TAG, "CommandProcess in %d out %d", fileno(io->in), fileno(io->out));

        stdin = io->in;
        stdout = io->out;

        free((void*) io);
        io = NULL;
    }

    bool running = true;

    ESP_LOGE(TAG, "stdin is %p fd %d  stdout is %p fd %d", stdin, fileno(stdin), stdout, fileno(stdout));

    printf("********************************************************\n");
    printf("CommandProcessor running\n");
    printf("********************************************************\n");

    while (running) {
        char buffer[100];

        printf(">> ");

        int len = readline(buffer, sizeof(buffer));
        //printf("readline returned %d\n", len);

        if (len >= 1) {

            const char* args[MAX_ARGS];
            int argc = tokenize(buffer, args, MAX_ARGS);

            int (*function)(int argc, const char **argv) = NULL;

            os_acquire_recursive_mutex(command_lock);

            command_entry_t *command = find_command(args[0]);
            if (command != NULL) {
                function = command->function;
            }
            os_release_recursive_mutex(command_lock);

            if (function != NULL) {
                int rc = (*function)(argc, args);
                if (rc != 0) {
                     printf("Error: %d\n", rc);
                }
            } else {
                printf("Invalid command: %s\n", args[0]);
            }

        } else if (len < 0) {
            running = false;
        }
    }
    printf("\n");
    printf("********************************************************\n");
    printf("CommandProcessor exiting\n");
    printf("********************************************************\n");

    os_exit_thread();
}

bool add_command(const char* name, int (*function)(int argc, const char **argv))
{
    command_entry_t *entry = (command_entry_t*) malloc(sizeof(command_entry_t));
    if (entry != NULL) {
        entry->name = name;
        entry->function = function;

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

void init_commands(void)
{
    command_lock = os_create_recursive_mutex();

    os_acquire_recursive_mutex(command_lock);

    add_command("help",       help_command);
    add_command("?",          help_command);
    add_command("mem",        memory_command);
    add_command("contrast",   contrast_command);
    add_command("echo",       echo_command);
    add_command("ping",       ping_command);
    add_command("address",    address_command);
    add_command("loglevel",   loglevel_command);
    add_command("config",     config_command);
    add_command("reboot",     reboot_command);

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


