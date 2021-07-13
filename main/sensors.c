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

#ifdef CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
#include "commands.h"
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

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

    /* Timer that controls when to test sensor value */
    simpletimer_t      timer;

    /* True if to automatically notify even if value has not changed */
    bool               notify;

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
        register_sensor("rh",   SENSOR_TYPE_INPUT, "%rh",  30, process_dht, (void*) 0);
        register_sensor("temp", SENSOR_TYPE_INPUT, "degC", 30, process_dht, (void*) 1);
    } else {
        ESP_LOGE(TAG, "unable to read dht sensor - not adding to sensors");
    }

    register_sensor("testval",  SENSOR_TYPE_VALUE, "",     0,  process_config_value, (void*) "values.testval");
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
    
                        /* Wait for a sensor data request */
                        int len = ls_read_with_address(socket, sensor_commands_buf, sizeof(sensor_commands_buf)-1, &sender_address, &sender_port, CONFIG_LASTLINK_SENSORS_SCAN_INTERVAL);
                        //if (len >= 0) {
                        //    ESP_LOGI(TAG, "%s: packet %d bytes from %d/%d", __func__, len, sender_address, sender_port);
                        //}
    
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

                            while ((item = strtok_r(tokens, "\n", &rest)) != NULL) {
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
                                            case SENSOR_TYPE_INPUT:              sensor_type = "IN";         break;
                                            case SENSOR_TYPE_INPUT_ACCUMULATOR:  sensor_type = "IN_AC";      break;
                                            case SENSOR_TYPE_OUTPUT:             sensor_type = "OUT";        break;
                                            case SENSOR_TYPE_OUTPUT_TIMED:       sensor_type = "OUT_TIMED";  break;
                                            case SENSOR_TYPE_VALUE:              sensor_type = "VAL";        break;
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
                                            if (! write_sensor(sensor, value, sensor_buffer, sizeof(sensor_buffer))) {
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
    
                        /* Do a scan and send notifications for values that have changed or need periodically posting */
                        os_acquire_recursive_mutex(sensor_lock);
    
                        sensor_cache_t *sensor = (sensor_cache_t*) FIRST_LIST_ITEM(&sensor_cache);
    
                        while (sensor != NULL) {
                            // ESP_LOGD(TAG, "%s: looking at sensor %p: '%s'", __func__, sensor, sensor->name);
    
                            /* If timer is expired, announce the sensor value */
                            if (simpletimer_is_expired(&sensor->timer)) {
                                /* Try to read it's value */
                                char value_buffer[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH+1];
                                if (read_sensor(sensor, value_buffer, sizeof(value_buffer))) {
                                    /* If always notify or if value has changed, send the value */
                                    if (sensor->notify || strcmp(value_buffer, sensor->last_value) != 0)  {
                                        strncpy(sensor->last_value, value_buffer, sizeof(sensor->last_value));
                                        sensor->last_value[sizeof(sensor->last_value)-1] = '\0';
                                        int len = snprintf(reply_buffer, sizeof(reply_buffer), "V:%s=%s %s", sensor->name, value_buffer, sensor->units ? sensor->units : "");
    
                                        ret = ls_write(socket, reply_buffer, len);

                                        if (ret < 0) {
                                            ESP_LOGE(TAG, "%s: write %s failed: %d", __func__, sensor->name, ret);
                                        // } else {
                                        //       ESP_LOGI(TAG, "%s: service %s type %s on port %d posted", __func__, service->name, (service->socket_type == LS_STREAM ? "S" : "D"), service->port);
                                        }
                                    }
                                } else {
                                    ESP_LOGE(TAG, "%s: unable to read sensor '%s'", __func__, sensor->name);
                                }

                                simpletimer_restart(&sensor->timer);
                            }

                            sensor = NEXT_LIST_ITEM(sensor, &sensor_cache);
                        }
    
                        os_release_recursive_mutex(sensor_lock);
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
 *      notify_time     If > 0 then automatically broadcast sensor value every <notify_time> seconds.
 *                      If < 0, then automatically broadcast sensor when value changes
 *                      If == 0, then only send value when requested.
 *                      (time is in seconds)
 *      function        Function to call to read/write sensor value.
 *      param           An additional parameter, if needed, passed to the read/write function.
 */
bool register_sensor(const char* name, sensor_type_t type, const char* units, int notify_time, sensor_function_t function, void* param)
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

            /* Set the timer based upon definition of notify_time:
             *  If < 0, test every abs(n) ms but report only when changed.
             *  If > 0, report every (n) ms
             *  If == 0, don't report - timer is stopped.
             */ 
             
            if (notify_time != 0) {
                int abs_notify = notify_time > 0 ? notify_time : -notify_time;

                /* Period of reporting sensor automatically is abs(notify_time) */
                simpletimer_start(&sensor->timer, abs_notify * 1000);

                /* Randomize the time by extending for a 0..<notify_time> milliseconds with a minimum of 1 seconds */
                simpletimer_set_expire_in(&sensor->timer, (os_urandom() % abs_notify) * 1000);

                /* Notify every <notify_time> seconds when set; otherwise only notify every <notify_time> seconds when changed. */
                sensor->notify = notify_time > 0;
            } else {
                simpletimer_stop(&sensor->timer);
            }


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


#ifdef CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
typedef struct sensor_cache_copy {
    void*             p;
    void*             prev;
    void*             next;
    sensor_type_t     type;
    sensor_function_t *function;
    bool              notify;
    int               timer;
    char              last_value[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH+1];
    char              name[CONFIG_LASTLINK_SENSORS_MAX_NAME_LENGTH+1];
    const char*       units;
} sensor_cache_copy_t;

int sensor_commands(int argc, const char **argv)
{
    const char* err = NULL;

    if (argc == 0) {
        show_help(argv[0], "list", "List active sensors");
        show_help(argv[0], "add <name> <type>", "List active sensors");
        printf("%s: print some help here\n", __func__);
    } else if (argc == 1 || strcmp(argv[1], "list") == 0) {
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

            sensors[num_sensors].timer = simpletimer_is_running(&sensor->timer) ? simpletimer_remaining(&sensor->timer) : -1;

            sensors[num_sensors].notify = sensor->notify;

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
            printf("P           Next        Prev        Type  Notify  Function    Timer  Last Value  Units  Name\n");
            //     "0xXXXXXXXX  0xXXXXXXXX  0xXXXXXXXX  xx    xxxxxx  0xXXXXXXXX  XXXXX  XXXXXXXXXX  XXXXX  XXXXX"
            for (int index = 0; index < num_sensors; ++index) {
                printf("%p  %p  %p  %-4d  %-6s  %p  %-5d  %-10s  %-5s  %-s\n",
                       sensors[index].p,
                       sensors[index].next,
                       sensors[index].prev,
                       sensors[index].type,
                       sensors[index].notify ? "True" : "False",
                       sensors[index].function,
                       sensors[index].timer / 1000,
                       sensors[index].last_value,
                       sensors[index].units,
                       sensors[index].name);
            }
        }
    } else if (strcmp(argv[1], "set") == 0) {
        /* sensor set <name> <value> */
        if (argc == 4) {
            os_acquire_recursive_mutex(sensor_lock);
            sensor_cache_t *sensor = lookup_sensor(argv[2]);
            if (sensor != NULL) {
                char reply_buffer[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH]; 
                if (! write_sensor(sensor, argv[3], reply_buffer, sizeof(reply_buffer))) {
                    printf("%s: set %s error '%s'\n", __func__, argv[2], reply_buffer);
                }
            }
            os_release_recursive_mutex(sensor_lock);
        } else {
            err = "Wrong  number of parameters";
        }
    } else if (strcmp(argv[1], "get") == 0) {
        /* sensor get <name> */
        if (argc == 3) {
            os_acquire_recursive_mutex(sensor_lock);
            sensor_cache_t *sensor = lookup_sensor(argv[2]);
            if (sensor != NULL) {
                char reply_buffer[CONFIG_LASTLINK_SENSORS_MAX_VALUE_LENGTH]; 
                if (read_sensor(sensor, reply_buffer, sizeof(reply_buffer))) {
                    printf("%s: get %s value '%s'\n", __func__, argv[2], reply_buffer);
                } else {
                    printf("%s: get %s error '%s'\n", __func__, argv[2], reply_buffer);
                }
            }
            os_release_recursive_mutex(sensor_lock);
        } else {
            err = "Wrong number of parameters";
        }
    }

    int rc = 0;

    if (err != NULL) {
        printf("%s\n", err);
        rc = 1;
    }

    return rc;
}
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

bool init_sensors(void)
{
    sensor_lock = os_create_recursive_mutex();

    sensor_scanner_thread_id = os_create_thread(sensor_scanner_thread, "sensors", SENSOR_SCANNER_STACK_SIZE, 0, NULL);

#ifdef CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    add_command("sensors", sensor_commands);
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
