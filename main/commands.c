/*
 * A simple command processor called with a pair of file descriptors for communication.
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "os_specific.h"
#include "lsocket.h"
#include "commands.h"
#include "linklayer.h"
#include "configdata.h"

#ifdef CONFIG_SSD1306_I2C_ENABLED
#include "display.h"
#endif /* CONFIG_SSD1306_I2C_ENABLED */

#include "listops.h"
#include "tokenize.h"

#define TAG "commands"

#define COMMAND_PROCESSOR_STACK_SIZE 10000

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

/**********************************************************************/
/* echo <blah> <blah> <blah>                                          */
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
        printf("Free Heap:     %08x (%d)\r\n", xPortGetFreeHeapSize(), xPortGetFreeHeapSize());
        printf("Largest Free:  %08x (%d)\r\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        printf("Minimum Free:  %08x (%d)\r\n", xPortGetMinimumEverFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
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


#ifdef CONFIG_SSD1306_I2C_ENABLED
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
#endif /* CONFIG_SSD1306_I2C_ENABLED */
/**********************************************************************/
/* reboot                                                             */
/**********************************************************************/
static int reboot_command(int argc, const char **argv)
{
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
static int time_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Show current time int mS since boot");
    } else {
        printf("%llu\n", get_milliseconds());
    }
    return 0;
}

static int kill_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<handle>", "Kill a task");
    } else {
        if (argc > 1) {
            unsigned int handle = strtol(argv[1], NULL, 16);

            printf("Killing thread %x\n", handle);

            os_delete_thread((os_thread_t) handle);

        } else {
            printf("Insufficient args\n");
        }
    }

    return 0;
}

static int ps_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "List all tasks");
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
            printf("\nTask Name         Handle  Runtime  State      Priority  Cpu  High Stack\n");
            for( size_t x = 0; x < num_tasks; x++ ) {
                char runtime[20];
                sprintf(runtime, "%u.%u", pdTICKS_TO_MS(pxTaskStatusArray[x].ulRunTimeCounter) / 1000, (pdTICKS_TO_MS(pxTaskStatusArray[x].ulRunTimeCounter) / 100) % 10);

                printf("%-16s  %-6x  %-7s  %-9s  %-8u  %-3d  %-5d\n",
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

    return 0;
}

typedef struct spawn_param {
    int (*command)(int argc, const char** argv);
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

    //printf("spawn_thread started\n");
    //for (int arg = 0; arg < spawn_params->argc + 1; ++arg) {
    //    printf("arg %d is \"%s\"\n", arg, spawn_params->argv[arg] ? spawn_params->argv[arg] : "<NULL>");
    //}

    int results = spawn_params->command(spawn_params->argc, spawn_params->argv);
    
    printf("%s: exit with %d\n", spawn_params->argv[0], results);

    /* All parameters are added to end of spawn_params, so they get free'd as well */
    free((void*) spawn_params->argv);
    free((void*) spawn_params->args);
    free((void*) spawn_params);

    os_exit_thread();
}

/*
 * Spawn another command to run in the background
 */
int spawn_command(int argc, const char **argv)
{
    int results = 0;

    if (argc == 0) {
        show_help(argv[0], "<command> <params>...", "spawn command as background thread");
    } else if (argc < 2) {
        printf("Insufficient args\n");
    } else {
        command_entry_t *command = find_command(argv[1]);
        if (command == NULL) {
            printf("command '%s' not found\n", argv[1]);
        } else {
            /* Compute length of param table */

            int arg;
            int args_length = 0;

            /* Calculate amount of string space needed */
            for (arg = 1; arg < argc; ++arg)  {
                args_length += strlen(argv[arg]) + 1;
            }

            spawn_param_t *spawn_params = (spawn_param_t*) malloc(sizeof(spawn_param_t));

            /* Argv array */
            spawn_params->argv = (const char **) malloc(argc * sizeof(const char*)); /* Room for all args + 1 for NULL */

            /* Point to command */
            spawn_params->command = command->function;

            /* Number of args - 1 from spawn  command */
            spawn_params->argc = argc - 1;

            /* Where the parameter strings go */
            char *params = (char*) malloc(args_length);

            /* So we can free it when we are done */
            spawn_params->args = params;

            for (arg = 1; arg < argc; ++arg) {
                /* Copy strings of params */
                spawn_params->argv[arg - 1] = (const char*) params;
                int len = strlen(argv[arg]);
                strcpy(params, argv[arg]);
                params += len;
                *params++ = '\0';
            }

            spawn_params->argv[arg - 1] = NULL;  /* Last argv is a NULL */

            // for (arg = 0; arg < spawn_params->argc + 1; ++arg) {
            //     printf("arg %d is \"%s\"\n", arg, spawn_params->argv[arg] ? spawn_params->argv[arg] : "<NULL>");
            // }

            /* Spawn with name of command */
            os_create_thread(spawn_thread, argv[1], 8192, 0, (void*) spawn_params);
        }
    }

    return results;
}

#ifdef CONFIG_DHT_ENABLED
#include "dht.h"

int dht_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Read RH and Temp for DHT sensor");
    } else {
        int count = 1;
        if (argc > 1) {
            count = strtol(argv[1], NULL, 10);
        }
        while (count > 0) {
            dht_value_t rh;
            dht_value_t temp;
            dht_ret_t rc = dht_read(&rh, &temp);
            if (rc == 0) {
                printf("rc %d: rh %.1f%% temp %.1f deg\n", rc, rh, temp);
            } else {
                printf("dht error %d\n", rc);
            }
            if (--count > 0) {
                os_delay(2000);
            }
        }
    }
    return 0;
}
#endif


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

            if (argc > 0) {
                command_entry_t *command = find_command(args[0]);
                if (command != NULL) {
                    function = command->function;
                }
            }

            os_release_recursive_mutex(command_lock);

            if (function != NULL) {
                int rc = (*function)(argc, args);
                if (rc != 0) {
                     printf("Error: %d\n", rc);
                }
            } else if (argc > 0) {
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

static int linklayer_set_inactive_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<mode>", "set linklayer inactive (1) or active(0)");
    } else if (argc > 1) {
        int inactive = strtol(argv[1], NULL, 10);
        bool rc = linklayer_set_inactive(inactive);
        if (! rc) {
            ESP_LOGE(TAG, "linklayer_set_inactive returned false");
        }
    }

    return 0;
} 
   
void init_commands(void)
{
    command_lock = os_create_recursive_mutex();

    os_acquire_recursive_mutex(command_lock);

    add_command("help",        help_command);
    add_command("?",           help_command);
    add_command("mem",         memory_command);
#ifdef CONFIG_SSD1306_I2C_ENABLED
    add_command("contrast",    contrast_command);
#endif /* CONFIG_SSD1306_I2C_ENABLED */
    add_command("echo",        echo_command);
    add_command("address",     address_command);
    add_command("loglevel",    loglevel_command);
    add_command("config",      config_command);
    add_command("reboot",      reboot_command);
    add_command("ps",          ps_command);
    add_command("kill",        kill_command);
    add_command("time",        time_command);
    add_command("spawn",       spawn_command);
#ifdef CONFIG_DHT_ENABLED
    add_command("dht",         dht_command);
#endif
    add_command("setinactive", linklayer_set_inactive_command);

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


