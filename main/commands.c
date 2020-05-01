/*
 * A simple command processor called with a pair of file descriptors for communication.
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "esp_log.h"

#include "os_freertos.h"
#include "lsocket.h"
#include "commands.h"
#include "linklayer.h"
#include "configdata.h"

#include "display.h"

#define TAG "commands"

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

/*
 * Absorb all characters in queue looking for match with testch.
 * Return true if one seen.
 */
static bool hit_test(int testch)
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
    int in;
    int out;
} command_param_t;

static void CommandProcessor(void* params);

/* Start a command processor on the specific fd pair */
os_thread_t start_commands(int infd, int outfd)
{
    command_param_t *param = NULL;

    if (infd >= 0 && outfd >= 0) {
        param = (command_param_t*) malloc(sizeof(command_param_t));

        if (param == NULL) {
            printf("Unable to allocate command parameter; starting with default stdin/stdout\n");
        } else {
            param->in = infd;
            param->out = outfd;
        }
    }

    return os_create_thread(CommandProcessor, "commands", 40000, 10, param);
}

#define MAX_ARGS  20

#define HELP_CMD_FIELD_LEN   30

static void show_help(const char* command, const char *params, const char *description)
{
    printf("%s %-*s %s\n", command, HELP_CMD_FIELD_LEN - strlen(command), params, description);
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

        int paths[100];
        int path_len = ping(address, paths, 100, 10000);
        if (path_len < 0) {
            printf("Ping error %d\n", path_len);
        } else {
            printf("Path:");
            for (int path = 0; path < path_len; ++path) {
                printf(" %d", paths[path]);
            }
            printf("\n");
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
/* dglisten <port>                                                    */
/**********************************************************************/
static int dglisten_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<port>", "Listen for datagrams on <port>");
    } else if (argc > 1) {
        int port = strtol(argv[1], NULL, 10);
        int socket = ls_socket(LS_DATAGRAM);
        if (socket >= 0) {
            printf("listening socket %d port %d...\n", socket, port);
            int ret = ls_bind(socket, port);
            if (ret >= 0) {
                /* Accept packets from any */
                ret = ls_connect(socket, NULL_ADDRESS, 0);
                if (ret == LSE_NO_ERROR) {
                    ls_dump_socket("receiving", socket);
                    while (!hit_test('\x03')) {
                        char buf[200];
                        int address;

                        int len = ls_read_with_address(socket, buf, sizeof(buf), &address, &port, 50);
                        if (len >= 0) {
                            buf[len] = '\0';
                            printf("Data from %d/%d: '%s'\n", address, port, buf);
                        } else if (len != LSE_TIMEOUT) {
                            printf("ls_read returned %d\n", len);
                        }
                    }
                } else {
                    printf("ls_connect returned %d\n", ret);
                }
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            ls_close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }
    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

/**********************************************************************/
/* dgsend <address> <port> <data>                                     */
/**********************************************************************/
static int dgsend_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<address> <port> <data>", "Send datagram <data> to <port> on <address>");
    } else if (argc >= 4) {
        int address = strtol(argv[1], NULL, 10);
        int port = strtol(argv[2], NULL, 10);
        printf("Sending '%s' to %d/%d\n", argv[3], address, port);
        int socket = ls_socket(LS_DATAGRAM);
        if (socket >= 0) {
            int ret = ls_bind(socket, 0);
            if (ret >= 0) {
                ret = ls_connect(socket, address, port);
                ls_dump_socket("sending", socket);
                if (ret >= 0) {
                    ret = ls_write(socket, argv[3], strlen(argv[3]));
                    printf("ls_write returned %d\n", ret);
                } else {
                    printf("ls_connect returned %d\n", ret);
                }
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            ls_close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }

    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

/**********************************************************************/
/* stlisten <port>                                                    */
/**********************************************************************/
static int stlisten_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<port>", "Listen for stream connection on <port>");
    } else if (argc > 1) {
        int port = strtol(argv[1], NULL, 10);
        int socket = ls_socket(LS_STREAM);
        if (socket >= 0) {
            int ret = ls_bind(socket, port);
            if (ret >= 0) {
                /* Accept packets from any */
                bool done = false;
                printf("listening socket %d port %d...\n", socket, port);
                do {
                    int connection = ls_listen(socket, 5, 50);
                    if (connection >= 0) {
                        printf("got connection on %d\n", connection);
                        char buffer[80];
                        int len;
                        while ((len = ls_read(connection, buffer, sizeof(buffer))) > 0) {
                            fwrite(buffer, len, 1, stdout);
                        }
                        printf("--end--\n");
                        ls_close(connection);
                        done = hit_test('\x03');
                    } else if (connection != LSE_TIMEOUT) {
                        printf("ls_listen returned %d\n", ret);
                        done = true;
                    }
                } while (!done);
                ls_close(socket);
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            ls_close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }
    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

/**********************************************************************/
/* stconnect <address> <port>                                         */
/**********************************************************************/
static int stconnect_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "<address> <port>", "Connect to stream <port> on <address>");
    } else if (argc >= 4) {
        int address = strtol(argv[1], NULL, 10);
        int port = strtol(argv[2], NULL, 10);

        int socket = ls_socket(LS_STREAM);
        if (socket >= 0) {
            int ret = ls_bind(socket, 0);
            if (ret >= 0) {
                ret = ls_connect(socket, address, port);
                ls_dump_socket("sending", socket);
                if (ret >= 0) {
                    for (int line = 1; line <= 10; ++line) {
                        char buffer[80];
                        sprintf(buffer, "%s: line %d\n", argv[3], line);
                        ret = ls_write(socket, argv[3], strlen(argv[3]));
                        printf("ls_write returned %d\n", ret);
                    }
                } else {
                    printf("ls_connect returned %d\n", ret);
                }
            } else {
                printf("ls_bind returned %d\n", ret);
            }
            ls_close(socket);
        } else {
            printf("Unable to open socket: %d\n", socket);
        }

    } else {
        printf("Insufficient params\n");
    }
    return 0;
}

/**********************************************************************/
/* status                                                             */
/**********************************************************************/
static int status_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Show system status");
    } else {
        printf("free packets %d\n", available_packets());
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

typedef struct command_entry {
    const char*    name;
    int            (*function)(int argc, const char **argv);
} command_entry_t;

static int help_command(int argc, const char **argv);

static command_entry_t  command_table[] = {
    { .name = "help",       .function = help_command      },
    { .name = "?",          .function = help_command      },
    { .name = "contrast",   .function = contrast_command  },
    { .name = "echo",       .function = echo_command      },
    { .name = "ping",       .function = ping_command      },
    { .name = "address",    .function = address_command   },
    { .name = "loglevel",   .function = loglevel_command  },
    { .name = "config",     .function = config_command    },
    { .name = "dglisten",   .function = dglisten_command  },
    { .name = "dgsend",     .function = dgsend_command    },
    { .name = "reboot",     .function = reboot_command    },
    { .name = "stlisten",   .function = stlisten_command  },
    { .name = "stconnect",  .function = stconnect_command },
    { .name = "status",     .function = status_command    },
};

/**********************************************************************/
/* help                                                               */
/**********************************************************************/
static int help_command(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Show help");
    } else {
        for (int command = 0; command < ELEMENTS_OF(command_table); ++command) {
            const char *table[2] = { command_table[command].name, NULL };
            command_table[command].function(0, table);
        }
    }

    return 0;
}


void CommandProcessor(void* params)
{
    if (params != NULL) {
        command_param_t *io = (command_param_t*) params;

        ESP_LOGE(TAG, "CommandProcess in %d out %d", io->in, io->out);

        _GLOBAL_REENT->_stdin = fdopen(io->in, "r");
        _GLOBAL_REENT->_stdout = fdopen(io->out, "w");

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

#if 0
            char *savep;

            args[argc++] = strtok_r(buffer, " ", &savep);
            
            while ((argc < MAX_ARGS) && ((args[argc] = strtok_r(NULL, "\"\' ", &savep)) != NULL)) {
                ++argc;
            }
            args[argc] = NULL;
#endif

            command_entry_t* command = NULL;

            for (int entry = 0; command == NULL && entry < ELEMENTS_OF(command_table); ++entry) {
                if (strcmp(args[0], command_table[entry].name) == 0) {
                    command = &command_table[entry];
                }
            }

            if (command != NULL) {
                int rc = command->function(argc, args);
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
}
