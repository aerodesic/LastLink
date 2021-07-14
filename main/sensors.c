/*
 * sensors.c
 *
 * Define a class of sensor.
 *
 * Sensors are devices or values that can be read or optionally posted at intervals or when they change value.
 *
 * Input sensors are only readable or posted
 * Output sensors read readable (as last value) or posted
 *
 */
#include "sdkconfig.h"

#ifdef CONFIG_LASTLINK_SENSORS_ENABLE

#define SENSOR_SCANNER_STACK_SIZE  4096

#include <stdarg.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "esp_vfs.h"
#include "esp_vfs_dev.h"
//#include "esp_attr.h"

#include "esp_system.h"
#include "esp_log.h"

#include "os_specific.h"
#include "listops.h"
#include "service_names.h"
#include "linklayer.h"
#include "simpletimer.h"
#include "lsocket.h"
#include "sensors.h"
#include "tokenize.h"
#include "lsocket_internal.h"
#include "dht.h"
#include "configdata.h"

#ifdef CONFIG_LASTLINK_COMMAND_INTERFACE
#include "commands.h"
#endif /* CONFIG_LASTLINK_COMMAND_INTERFACE */

#define TAG "sensors"

typedef struct sensor_cache {
    sensor_cache_t     *next;
    sensor_cache_t     *prev;

    /* Basic sensor type: Input, Output, Timed, etc. */
    sensor_type_t      type;

    /* Function to call to read/write raw sensor */
    sensor_function_t  *function;

    /* Extra optional value to pass to function parameter */
    void               *param;

    /* An optional place to store the last sensor value */
    char               last_value[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH+1];

    const char         *name;
    const char         *units;
} sensor_cache_t;


static list_head_t     sensor_cache;
static os_mutex_t      sensor_lock;
static bool            sensor_scanner_running;
static os_thread_t     sensor_scanner_thread_id;

static bool read_sensor(sensor_cache_t *sensor, char *value_buffer, size_t value_len)
{
    return sensor->function(SENSOR_READ, sensor->name, sensor->param, value_buffer, value_len);
}

static bool write_sensor(sensor_cache_t *sensor, const char *value, char *reply_buffer, size_t reply_len)
{
    return sensor->function(SENSOR_WRITE, sensor->name, sensor->param, reply_buffer, reply_len, value);
}

static sensor_cache_t* create_sensor(const char* name)
{
    sensor_cache_t* sensor = (sensor_cache_t*) malloc(sizeof(sensor_cache_t));
    if (sensor != NULL) {
        memset(sensor, 0, sizeof(sensor_cache_t));
        if (name != NULL) {
            sensor->name = strdup(name);
        }
    }

    return sensor;
}

static sensor_cache_t *remove_sensor(sensor_cache_t *sensor)
{
    os_acquire_recursive_mutex(sensor_lock);

    REMOVE_FROM_LIST(&sensor_cache, sensor);

    os_release_recursive_mutex(sensor_lock);

    return sensor;
}

static void release_sensor(sensor_cache_t* sensor)
{
    free((void*) sensor->units);
    free((void*) sensor->name);
    free((void*) sensor);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */
}


static sensor_cache_t *lookup_sensor(const char* name)
{
    sensor_cache_t *found = NULL;

    os_acquire_recursive_mutex(sensor_lock);

    sensor_cache_t *sensor = (sensor_cache_t*) FIRST_LIST_ITEM(&sensor_cache);

    while (found == NULL && sensor != NULL) {
        if (strcmp(sensor->name, name) == 0) {
            found = sensor;
        } else {
            sensor = NEXT_LIST_ITEM(sensor, &sensor_cache);
        }
    }

    os_release_recursive_mutex(sensor_lock);

    return found;
}

#ifdef CONFIG_LASTLINK_SENSORS_DHT_ENABLE
/* Initialize sensor scanner */
bool process_dht(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool success = true;

    if (transaction == SENSOR_READ) {
        dht_value_t temperature;
        dht_value_t humidity;
        dht_ret_t rc = dht_read(&humidity, &temperature);
        if (rc != DHT_OK) {
            strncpy(reply_buffer, "READ", reply_len);
            reply_buffer[reply_len - 1] = '\0';
            success = false;
        } else {
            snprintf(reply_buffer, reply_len, "%.1f", param == 0 ? humidity : temperature);
        }
    } else {
        snprintf(reply_buffer, reply_len, "RO");
        success = false;
    }

    return success;
}
#endif

bool process_config_value(sensor_transaction_t transaction, const char* name, void* param, char *reply_buffer, size_t reply_len, ...)
{
    bool success = false;

    lock_config();

    if (transaction == SENSOR_READ) {
        const char* value = get_config_str((char*) param, "");
        if (value != NULL) {
            snprintf(reply_buffer, reply_len, "%s", value);
            success = true;
        }
    } else if (transaction == SENSOR_WRITE) {

        va_list args;
        va_start(args, reply_len);

        const char* value = va_arg(args, const char*);
        success = set_config_str((const char*) param, value);
    }

    unlock_config();

    return success;
}

static void initialize_all_sensors(void)
{
#ifdef CONFIG_LASTLINK_SENSORS_DHT_ENABLE
    dht_ret_t read_rc;

    /* Only enable the sensor if we can read it */
    int count = 0;
    dht_value_t dummy;
    while (count < 5 && (read_rc = dht_read(&dummy, &dummy)) != DHT_OK) { 
        ESP_LOGE(TAG, "%s: error %d reading dht", __func__, read_rc);
        count++;
        os_delay(2500);
    }
 
    if (read_rc == DHT_OK) {
        ESP_LOGI(TAG, "read dht successful - creating sensors");
        /* Add a couple of sensors for testing */
        register_sensor("rh",   SENSOR_TYPE_INPUT, "%rh",  process_dht, (void*) 0);
        register_sensor("temp", SENSOR_TYPE_INPUT, "degC", process_dht, (void*) 1);
    } else {
        ESP_LOGE(TAG, "unable to read dht sensor - not adding to sensors");
    }

    register_sensor("testval",  SENSOR_TYPE_VALUE, "",     process_config_value, (void*) "values.testval");
#endif
}


/*
 * This thread scans the sensors and receives requests for setting/getting sensor values.
 */
static void sensor_scanner_thread(void* param)
{
    sensor_scanner_running = 1;

ESP_LOGI(TAG, "%s: running", __func__);

    initialize_all_sensors();

    /* Register a socket for receiving sensor read/write messages */
    int socket = ls_socket(LS_DATAGRAM);
    if (socket >= 0) {
        int ret = ls_bind(socket, 0);
        if (ret == LSE_NO_ERROR) {
            ret = ls_connect(socket, BROADCAST_ADDRESS, CONFIG_LASTLINK_SENSORS_WELL_KNOWN_PORT);
            if (ret == LSE_NO_ERROR) {
                // Publish the sensor service
                if (!register_service(CONFIG_LASTLINK_SENSORS_PLATFORM_NAME, socket, CONFIG_LASTLINK_SENSORS_SERVICE_LIFETIME)) {
                    /*   Failed */
                    ESP_LOGE(TAG, "%s: Unable to register sensor service '%s'", __func__, CONFIG_LASTLINK_SENSORS_PLATFORM_NAME);
                } else {
                    /* Run service */
                    char reply_buffer[DATAGRAM_MAX_DATA];

                    while (sensor_scanner_running > 0) {
                        char sensor_commands_buf[DATAGRAM_MAX_DATA+1];
                        int sender_address;
                        int sender_port;
    
                        /* Wait for a sensor data request (1 second max and look for termination) */
                        int len = ls_read_with_address(socket, sensor_commands_buf, sizeof(sensor_commands_buf)-1, &sender_address, &sender_port, 1000);
                        if (len > 0) {
                            /************************************************************************
                             * Process a request to update or get value from a sensor.
                             * A '*' will report all sensors available, with units and R/W status.
                             *
                             * A read request is specified by giving the sensor name only.
                             *
                             * A write request is specified by the format <sensor name>=<value>
                             *
                             * A reply will be generated for every 'get' request in the form:
                             *   V:<sensor name>=<value>
                             *
                             * A reply will be generated for every 'set' request in the form:
                             *   E:<sensor name>=<error message>
                             * whenever an error occurs attempting to set the value.  If no error is
                             * detected, no response will be given.
                             ************************************************************************/
                            sensor_commands_buf[len] = '\0';
                            // ESP_LOGI(TAG, "%s: sensor commands: %s", __func__, sensor_commands_buf);

                            /* Parse list of:
                             *   <name>[=<value>],...
                             * <name> requests the value
                             * <name>=<value> sets a value.
                             */
                            char *tokens = sensor_commands_buf;
                            char *rest;
                            char *item;
    
                            char *reply_pointer = reply_buffer;
                            int reply_used = 0;

                            while ((item = strtok_r(tokens, " ", &rest)) != NULL) {
                                tokens = NULL;
                                strstrip(item);
                                char *value = strchr(item, '=');
                                if (value != NULL) {
                                    *value++ = '\0';
                                    /* Assignment */
                                    strstrip(item);
                                    strstrip(value);
                                } else {
                                    /* Query */
                                    value = NULL;
                                }

                                int moved = 0;
                                if (strcmp(item, "*") == 0) {
                                    /* Return log of sensor channels available */
                                    os_acquire_recursive_mutex(sensor_lock);

                                    sensor_cache_t *sensor = (sensor_cache_t*) FIRST_LIST_ITEM(&sensor_cache);

                                    while (sensor != NULL) {
                                        const char *sensor_type;
                                        switch (sensor->type) {
                                            case SENSOR_TYPE_INPUT:              sensor_type = "input";      break;
                                            case SENSOR_TYPE_INPUT_ACCUMULATOR:  sensor_type = "accum";      break;
                                            case SENSOR_TYPE_OUTPUT:             sensor_type = "output";     break;
                                            case SENSOR_TYPE_OUTPUT_TIMED:       sensor_type = "timed";      break;
                                            case SENSOR_TYPE_VALUE:              sensor_type = "value";      break;
                                            default:                             sensor_type = "?";          break;
                                        }
                                        int moved = snprintf(reply_pointer, sizeof(reply_buffer) - reply_used, "N:%s:%s:%s\n", sensor->name, sensor->units, sensor_type);
                                        reply_pointer += moved;
                                        reply_used += moved;
                                        sensor = NEXT_LIST_ITEM(sensor, &sensor_cache);
                                    }

                                    os_release_recursive_mutex(sensor_lock);
                                } else {
                                    //ESP_LOGI(TAG, "%s: sensor request %s: %s", __func__, item, value?value : "NULL");
                                    sensor_cache_t *sensor = lookup_sensor(item);

                                    if (sensor != NULL) {
                                       char sensor_buffer[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH];

                                        if (value != NULL) {
                                            /* Setting value */
                                            if (write_sensor(sensor, value, sensor_buffer, sizeof(sensor_buffer))) {
                                                moved = snprintf(reply_pointer, sizeof(reply_buffer) - reply_used, "V:%s:%s:%s\n", sensor->name, value, sensor->units ? sensor->units : "");
                                            } else {
                                                moved = snprintf(reply_pointer, sizeof(reply_buffer) - reply_used, "E:%s:%s\n", sensor->name, sensor_buffer);
                                            }
                                        } else {
                                            /* Reading value */
                                            if (read_sensor(sensor, sensor_buffer, sizeof(sensor_buffer))) {
                                                moved = snprintf(reply_pointer, sizeof(reply_buffer) - reply_used, "V:%s:%s:%s\n", sensor->name, sensor_buffer, sensor->units ? sensor->units : "");
                                            } else {
                                                moved = snprintf(reply_pointer, sizeof(reply_buffer) - reply_used, "E:%s:%s\n", sensor->name, sensor_buffer);
                                            }
                                        }
                                    } else {
                                        moved = snprintf(reply_pointer, sizeof(reply_buffer) - reply_used, "U:%s\n", item);
                                    }
                                }

                                reply_pointer += moved;
                                reply_used += moved;
                            }

                            /* Send reply if data is present */
                            if (reply_used != 0) {
                                ls_error_t ret = ls_write_to(socket, reply_buffer, reply_used, sender_address, sender_port);
                                if (ret != reply_used) {
                                    ESP_LOGE(TAG, "%s: ls_write_to failed: %d", __func__, ret);
                                }
                            }
                        }
                    }

                    deregister_service(CONFIG_LASTLINK_SENSORS_PLATFORM_NAME);
                }

                ls_close(socket);
            }
        }
    }

    ESP_LOGE(TAG, "%s: scanner stopped: sensor_scanner_running %d socket %d", __func__, sensor_scanner_running, socket);

    sensor_scanner_running = 0;
}

/*
 *  register a new sensor
 *
 *  Entry:
 *      name            Name of sensor as reported and manipulated
 *      type            Type (input/output/timed/value, etc.)
 *      units           Units of sensor (if not NULL) [e.g. "seconds", "cc", "degC", etc.]
 *      function        Function to call to read/write sensor value.
 *      param           An additional parameter, if needed, passed to the read/write function.
 */
bool register_sensor(const char* name, sensor_type_t type, const char* units, sensor_function_t function, void* param)
{
    sensor_cache_t *sensor = NULL;

    os_acquire_recursive_mutex(sensor_lock);

    if (lookup_sensor(name) == NULL) {
        sensor = create_sensor(name);

        if (sensor != NULL) {
            sensor->type = type;
            sensor->function = function;
            sensor->param = param;
            sensor->units = units ? strdup(units) : NULL;

            /* Add to table */
            ADD_TO_LIST(&sensor_cache, sensor);
        }
        
        os_release_recursive_mutex(sensor_lock);

    } else {
        ESP_LOGE(TAG, "%s: Sensor %s already defined", __func__, name);
    }

    
    return sensor != NULL;
}

bool deregister_sensor(const char* name)
{
#ifdef CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    remove_command("sensor");
#endif

    os_acquire_recursive_mutex(sensor_lock);

    sensor_cache_t *sensor = lookup_sensor(name);

    if (sensor != NULL) {
        release_sensor(sensor);
    }

    os_release_recursive_mutex(sensor_lock);

    return sensor != NULL;
}


#ifdef CONFIG_LASTLINK_COMMAND_INTERFACE

#ifdef CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
typedef struct sensor_cache_copy {
    void*             p;
    void*             prev;
    void*             next;
    sensor_type_t     type;
    sensor_function_t *function;
    char              last_value[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH+1];
    char              name[CONFIG_LASTLINK_SENSORS_MAX_NAME_LENGTH+1];
    const char*       units;
} sensor_cache_copy_t;

static void sensor_command(command_context_t* context)
{
    const char* err = NULL;

    if (context->argc == 0) {
        show_help(context, "list", "List active sensors");
        show_help(context, "add <name> <type>", "List active sensors");
    } else if (context->argc == 1 || strcmp(context->argv[1], "list") == 0) {
        /* List sensors configured */
        os_acquire_recursive_mutex(sensor_lock);

        sensor_cache_t *sensor = (sensor_cache_t*) FIRST_LIST_ITEM(&sensor_cache);

        sensor_cache_copy_t sensors[NUM_IN_LIST(&sensor_cache)];
        int num_sensors = 0;

        /* Make copy of list */
        while (sensor != NULL) {
            sensors[num_sensors].p = sensor;

            sensors[num_sensors].next = sensor->next;

            sensors[num_sensors].prev = sensor->prev;

            sensors[num_sensors].type = sensor->type;

            sensors[num_sensors].function = sensor->function;

            strncpy(sensors[num_sensors].last_value, sensor->last_value, sizeof(sensors[num_sensors].last_value));
            sensors[num_sensors].last_value[sizeof(sensors[num_sensors].last_value) - 1] = '\0';

            strncpy(sensors[num_sensors].name, sensor->name, CONFIG_LASTLINK_SENSORS_MAX_NAME_LENGTH);
            sensors[num_sensors].name[CONFIG_LASTLINK_SENSORS_MAX_NAME_LENGTH] = '\0';

            sensors[num_sensors].units = sensor->units ? sensor->units : "";

            ++num_sensors;

            sensor = NEXT_LIST_ITEM(sensor, &sensor_cache);
        }

        os_release_recursive_mutex(sensor_lock);

        if (num_sensors != 0) {
            command_reply(context, "P           Next        Prev        Type  Function    Last Value  Units  Name");
            //     "0xXXXXXXXX  0xXXXXXXXX  0xXXXXXXXX  xx    0xXXXXXXXX  XXXXXXXXXX  XXXXX  XXXXX"
            for (int index = 0; index < num_sensors; ++index) {
                command_reply(context, "%p  %p  %p  %-4d  %p  %-10s  %-5s  %-s",
                       sensors[index].p,
                       sensors[index].next,
                       sensors[index].prev,
                       sensors[index].type,
                       sensors[index].function,
                       sensors[index].last_value,
                       sensors[index].units,
                       sensors[index].name);
            }
        }
    } else if (strcmp(context->argv[1], "set") == 0) {
        /* sensor set <name> <value> */
        if (context->argc == 4) {
            os_acquire_recursive_mutex(sensor_lock);
            sensor_cache_t *sensor = lookup_sensor(context->argv[2]);
            if (sensor != NULL) {
                char reply_buffer[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH]; 
                if (! write_sensor(sensor, context->argv[3], reply_buffer, sizeof(reply_buffer))) {
                    command_reply(context, "set %s error '%s'", context->argv[2], reply_buffer);
                }
            }
            os_release_recursive_mutex(sensor_lock);
        } else {
            err = "Wrong number of parameters";
        }
    } else if (strcmp(context->argv[1], "get") == 0) {
        /* sensor get <name> */
        if (context->argc == 3) {
            os_acquire_recursive_mutex(sensor_lock);
            sensor_cache_t *sensor = lookup_sensor(context->argv[2]);
            if (sensor != NULL) {
                char reply_buffer[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH]; 
                if (read_sensor(sensor, reply_buffer, sizeof(reply_buffer))) {
                    command_reply(context, "get %s value '%s'", context->argv[2], reply_buffer);
                } else {
                    command_reply(context, "get %s error '%s'", context->argv[2], reply_buffer);
                }
            }
            os_release_recursive_mutex(sensor_lock);
        } else {
            err = "Wrong number of parameters";
        }
    }

    if (err) {
        command_reply_error(context, err);
        context->results = -1;
    }
}
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

/**********************************************************************/
/* scon connect to sensor system                                      */
/* Enter a loop and take commands entered and send to the socket,     */
/* while displaying packets received from the socket.                 */
/* Control-C breaks out                                               */
/**********************************************************************/
typedef struct scon_data {
    os_thread_t thread_id;
    int socket;
    int running;
    command_context_t *context;
} scon_data_t;

static void scon_reader(void* param)
{
    scon_data_t* scon_data = (scon_data_t*) param;

    command_context_t* context = scon_data->context;

    scon_data->running = 1;

    while (scon_data->running > 0) {
        /* Read data from socket and echo to terminal */
        char buffer[300];

        int address;
        int port;

        int len = ls_read_with_address(scon_data->socket, buffer, sizeof(buffer) - 1, &address, &port, 500);

        if (len > 0) {
            buffer[len] = '\0';
            const char *p = buffer;
            while (p != NULL) {
                const char *pend = strchr(p, '\n');            
                const char *nextp;

                if (pend == NULL) {
                    pend = p + strlen(p);
                    nextp = NULL;
                } else {
                    nextp = pend + 1;
                }

                command_reply(context, "%*.*s", (pend - p), (pend - p), p);
                p = nextp;
            }
        }
    }

    scon_data->running = 0;

    command_reply(context, "scon_reader stopping");

    os_exit_thread();
}


#define MAX_SCON_ARGS 3
/*
 * scon
 *   -> <nn>:ack
 * <nn>:<address> <port> <data>
 *   -> <nn>:<reply if any>
 * ...
 * <nn>:close
 *   -> <nn>:closed
 */


/*
 * This command is spawned.
 */
static void scon_command(command_context_t* context)
{
    int rc = 0;

    if (context->argc == 0) {
        show_help(context, "", "Connect to sensor system");
    } else if (context->argc == 1) {
        /* Create scon data instance for connection */
        scon_data_t* scon_data = (scon_data_t*) malloc(sizeof(scon_data_t));
        if (scon_data == NULL) {
            command_reply_error(context, "out of memory");
        } else {
            context->param = (void*) scon_data;

            scon_data->socket = ls_socket(LS_DATAGRAM);
            scon_data->context = context;

            if (scon_data->socket >= 0) {
                int ret = ls_bind(scon_data->socket, 0);
                if (ret >= 0) {
                    /* Spawn a reader socket to echo back input packets to the control */
                    scon_data->thread_id = os_create_thread(scon_reader, "scon_reader", 4095, 0, (void*) scon_data);

                    /* Enter read loop from command processor and decode commands */
                    while (!context_check_termination_request(context)) {
                        char *message = command_read_more_with_timeout(context, -1);
                        if (message != NULL) {
                            const char* args[MAX_SCON_ARGS];
                            int nargs = tokenize(message, args, MAX_SCON_ARGS);

                            if (nargs == 1 && strcmp(args[0], "close")) {
                                context->terminate = true;
                            } else if (nargs >= 2) {
                                /* Post data to socket */
                                int address = strtol(args[0], NULL, 10);
                                int port = strtol(args[1], NULL, 10);
                    
                                int rc = ls_connect(scon_data->socket, address, port);
                    
                                if (rc >= 0) {
                                    if (nargs > 2) {
                                        rc = ls_write(scon_data->socket, args[2], strlen(args[2]));
                                    } else {
                                        rc = ls_write(scon_data->socket, "", 0);
                                    }
                                }                
                     
                                if (rc < 0) {
                                    command_reply_error(context, "error %d", rc);
                                }
                            } else {
                                command_reply_error(context, "insufficent params");
                            }

                            free((void*) message);
                        }
                    }

                    /* Close down connection server */
                    scon_data->running = -1;
                    while (scon_data->running != 0) {
                        os_delay(100);
                    }
                    ls_close(scon_data->socket);
                    scon_data->thread_id = NULL; 

                    command_reply(context, "closed");
                } else {
                    command_reply_error(context, "bind failed");
                }
            } else {
                command_reply_error(context, "socket failed");
            }

        }
    } else {
        command_reply_error(context, "invalid args");
    }

    if (context->param != NULL) {
        free(context->param);
        context->param = NULL;
    }

    if (context->spawned) {
        context_release(context);
    }

    context->results = rc;
}
#endif /* CONFIG_LASTLINK_COMMAND_INTERFACE */

bool init_sensors(void)
{
    sensor_lock = os_create_recursive_mutex();

    sensor_scanner_thread_id = os_create_thread(sensor_scanner_thread, "sensors", SENSOR_SCANNER_STACK_SIZE, 0, NULL);

#ifdef CONFIG_LASTLINK_COMMAND_INTERFACE
  #ifdef CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    add_command("sensors", sensor_command,  COMMAND_ONCE);
  #endif
    add_command("scon", scon_command,       COMMAND_SPAWN);
#endif

    ESP_LOGI(TAG, "%s: called", __func__);

    return true;
}

bool deinit_sensors(void)
{
#ifdef CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    remove_command("sensors");
#endif

    if (sensor_scanner_running != 0) {
        sensor_scanner_running = -1;

        /* Wait for scanner to terminate */
        while (sensor_scanner_running != 0) {
            os_delay(100);
        }
    }

    os_delete_thread(sensor_scanner_thread_id);
    sensor_scanner_thread_id = NULL;

    while (NUM_IN_LIST(&sensor_cache) != 0) {
        release_sensor(remove_sensor((sensor_cache_t*) FIRST_LIST_ITEM(&sensor_cache)));
    }

    os_delete_mutex(sensor_lock);
    sensor_lock = NULL;

    return true;
}

#endif /* CONFIG_LASTLINK_SENSORS_ENABLE */
