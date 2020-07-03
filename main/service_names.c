/*
 * service_names.c
 *
 * Protocol used to locate services by name.
 */

#include "sdkconfig.h"

#ifdef CONFIG_LASTLINK_SERVICE_NAMES_ENABLE

#define SERVICES_STACK_SIZE  4096

#include <stdarg.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

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

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
#include "commands.h"
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

#define TAG "service_names"

typedef struct service_cache service_cache_t;

typedef struct service_cache {
    service_cache_t   *next;
    service_cache_t   *prev;

    /* This is the socket type of the service.  If LS_UNUSED, then this is a pending service lookup entry */
    ls_socket_type_t  socket_type;

    /* Node address with service */
    int               address;

    /* Port used for service */
    ls_port_t         port;

    simpletimer_t     timer;

    /* Name of service */
    const char        *name;
} service_cache_t;

/* Cache table */
static list_head_t           service_cache;
static os_mutex_t            service_lock;      /* Mutex for exclusive access */
static os_thread_t           service_thread_id; /* Thread ID of scanner */
static int                   service_running;   /* 0 if stopped; 1 if running; -1 to kill */


static service_cache_t* create_service(const char* name)
{
    service_cache_t* service = (service_cache_t*) malloc(sizeof(service_cache_t));
    if (service != NULL) {
        memset(service, 0, sizeof(service_cache_t));
        if (name != NULL) {
            service->name = strdup(name);
        }
    }

    return service;
}

/*
 * Add a service entry to the table.
 */
static void add_service(service_cache_t *service)
{
    os_acquire_recursive_mutex(service_lock);

    ADD_TO_LIST(&service_cache, service);

    os_release_recursive_mutex(service_lock);
}

static service_cache_t *remove_service(service_cache_t *service)
{
    os_acquire_recursive_mutex(service_lock);

    REMOVE_FROM_LIST(&service_cache, service);

    os_release_recursive_mutex(service_lock);

    return service;
}

static void release_service(service_cache_t* service)
{
    free((void*) service->name);
    free((void*) service);

#ifdef CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK
    assert(heap_caps_check_integrity_all(true));
#endif /* CONFIG_LASTLINK_ADDED_HEAP_CAPS_CHECK */

}


static service_cache_t *lookup_service_by_name(const char* name)
{
    service_cache_t *found = NULL;

    os_acquire_recursive_mutex(service_lock);

    service_cache_t *service = (service_cache_t*) FIRST_LIST_ITEM(&service_cache);

    while (found == NULL && service != NULL) {
        if (strcmp(service->name, name) == 0) {
            found = service;
        } else {
            service = NEXT_LIST_ITEM(service, &service_cache);
        }
    }

    os_release_recursive_mutex(service_lock);

    return found;
}

/*
 * Return service if newly created, otherwise NULL
 */
static service_cache_t *create_service_by_name(const char* name)
{
    service_cache_t *service = NULL;

    os_acquire_recursive_mutex(service_lock);

    if (! lookup_service_by_name(name)) {
        service = create_service(name);

        if (service != NULL) {
            service->address = NULL_ADDRESS;
            service->socket_type = LS_UNUSED;
            service->port = 0;

            add_service(service);
        }
    }

    os_release_recursive_mutex(service_lock);

    return service;
}

static void service_thread(void* param)
{
    service_running = 1;

ESP_LOGI(TAG, "%s: running", __func__);

    int socket = ls_socket(LS_DATAGRAM);
    if (socket >= 0) {
        int ret = ls_bind(socket, SERVICE_NAMES_PORT);
        if (ret == LSE_NO_ERROR) {
            ret = ls_connect(socket, NULL_ADDRESS, 0);
            if (ret == LSE_NO_ERROR) {
                while (service_running) {
                    char service_name[SERVICE_NAMES_MAX_LEN + 1];
                    int address;
                    int port;

                    /* Wait for a service request (and acts as timer) */
                    int len = ls_read_with_address(socket, service_name, SERVICE_NAMES_MAX_LEN, &address, &port, SERVICE_NAMES_WAIT_TIMEOUT);
                    if (len >= 0) {
                        service_name[len] = '\0';
//ESP_LOGI(TAG, "%s: service request '%s' from %d/%d", __func__, service_name, address, port);

                        os_acquire_recursive_mutex(service_lock);

                        service_cache_t *service = lookup_service_by_name(service_name);
                        if (service != NULL) {
                            service_answer_t service_answer;

                            service_answer.port = service->port;
                            service_answer.socket_type = service->socket_type;
                            strncpy(service_answer.name, service->name, SERVICE_NAMES_MAX_LEN);

                            /* This returns data to caller since our dest addresses have been renamed */
                            ls_write(socket, &service_answer, sizeof(service_answer));
//ESP_LOGI(TAG, "%s: sending answer '%s' type %d port %d to %d/%d", __func__, service_answer.name, service_answer.socket_type, service_answer.port, address, port);
                        }

                        os_release_recursive_mutex(service_lock);
                    }

                    /* Do a scan and expire entries */
                    os_acquire_recursive_mutex(service_lock);

                    service_cache_t *service = (service_cache_t*) FIRST_LIST_ITEM(&service_cache);

                    while (service != NULL) {
                        if (simpletimer_is_expired(&service->timer)) {
                            release_service(remove_service(service));
                            /* Only do one per scan */
                            service = NULL;
                        }
                        service = NEXT_LIST_ITEM(service, &service_cache);
                    }

                    os_release_recursive_mutex(service_lock);
                }

                ls_close(socket);
            }
        }
    }

    ESP_LOGE(TAG, "%s: scanner stopped", __func__);

    service_running = 0;
}

/************************************************************************************
 * User API for service_names below                                                 *
 ************************************************************************************/

/*
 * User-callable function to locate a service, sending packets and necessary.
 *
 * Returns LSE_NO_ERROR if found and values returned otherwise ls_error_t.
 */
ls_error_t find_service_by_name(const char* name, int *address, ls_socket_type_t *socket_type, ls_port_t* port)
{
    ls_error_t  ret = LSE_NO_ERROR;

    service_cache_t *service = lookup_service_by_name(name);

    if (service == NULL) {
        int socket = ls_socket(LS_DATAGRAM);
        if (socket >= 0) {
            ret = ls_bind(socket, 0);
            if (ret == LSE_NO_ERROR) {
                ret = ls_connect(socket, BROADCAST_ADDRESS, SERVICE_NAMES_PORT);
                if (ret == LSE_NO_ERROR) {
                    int tries = SERVICE_NAMES_REPLY_RETRIES;
                    do {
                        /* Send request for service info */
                        ret = ls_write(socket, name, strlen(name));
                        if (ret >= 0) {
                            service_answer_t answer;
                            int address;
                            int port;

                            ret = ls_read_with_address(socket, (void*) &answer, sizeof(answer), &address, &port, SERVICE_NAMES_REPLY_TIMEOUT);
//printf("%s: service read returned %d\n", __func__, ret);

//ESP_LOGI(TAG, "%s: ls_read_with_address returned %d", __func__, ret);
                            if (ret == sizeof(answer)) {
                                ret = LSE_NO_ERROR;

                                /* Create a cache entry locally */
                                service = create_service_by_name(answer.name);
                                if (service != NULL) {
                                    service->socket_type = answer.socket_type;
                                    service->port = answer.port;
                                    service->address = address;
                                    service->name = strdup(answer.name);
                                    simpletimer_start(&service->timer, SERVICE_NAMES_LIFETIME*1000);
                                } else {
                                    ret = LSE_NO_MEM;
                                }
                            }
                        // } else {
                            //printf("service 'write' returned %d\n", ret);
                        }
                    } while (ret != LSE_NO_ERROR && --tries != 0);
                }
            }
            ls_close(socket);
        } else {
            ret = socket;
        }
    }

    if (service != NULL) {
        /* Found it so return the values */
        *address = service->address;
        *socket_type = service->socket_type;
        *port = service->port;

        ret = LSE_NO_ERROR;
    }

    return ret;
}


/*
 * Register a new service
 */
bool register_service_name(const char* name, ls_socket_type_t socket_type, ls_port_t port)
{
    bool success = false;

    if (! lookup_service_by_name(name)) {
        service_cache_t *service = create_service(name);
        if (service != NULL) {

            service->socket_type = socket_type;
            service->port = port;
            service->address = linklayer_node_address;
            /* Leave timer stopped for permanent (until deregistered) service entries */
            simpletimer_stop(&service->timer);

            add_service(service);

            success = true;
        }
    }
    return success;
}


bool deregister_service_name(const char* name)
{
    service_cache_t* service = lookup_service_by_name(name);

    if (service != NULL) {
        release_service(remove_service(service));
    }

    return service != NULL;
}

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
typedef struct service_cache_copy {
    void*             p;
    void*             prev;
    void*             next;
    int               address;       /* Owner of this service */
    int               timer;         /* Timeout for autodestruct of service table entry */
    ls_socket_type_t  socket_type;
    ls_port_t         port;
    char              name[SERVICE_NAMES_MAX_LEN+1];
} service_cache_copy_t;

static int service_table_commands(int argc, const char **argv)
{
    int rc = 0;

    if (argc == 0) {
        show_help(argv[0], "list or <empty>", "List service cache");
        show_help(argv[0], "add <service name> <type> <port>", "Add service");
        show_help(argv[0], "del <service name>", "Remove service");
        show_help(argv[0], "<name>", "Lookup service");

    } else if (argc == 1 || strcmp(argv[1] , "list") == 0) {
        os_acquire_recursive_mutex(service_lock);

        service_cache_t *service = (service_cache_t*) FIRST_LIST_ITEM(&service_cache);

        service_cache_copy_t services[NUM_IN_LIST(&service_cache)];

        int num_services = 0;

        while (service != NULL) {
            services[num_services].p = service;
            services[num_services].next = service->next;
            services[num_services].prev = service->prev;
            services[num_services].address = service->address;
            services[num_services].timer = simpletimer_is_running(&service->timer) ? simpletimer_remaining(&service->timer) : -1;
            services[num_services].socket_type = service->socket_type;
            services[num_services].port = service->port;
            strncpy(services[num_services].name, service->name, SERVICE_NAMES_MAX_LEN);
            services[num_services].name[SERVICE_NAMES_MAX_LEN] = '\0';

            num_services++;

            service = NEXT_LIST_ITEM(service, &service_cache);
        }

        os_release_recursive_mutex(service_lock);

        if (num_services != 0) {
            printf("P           Next        Prev        Address  Timer  Type  Port  Name\n");
            for (int index = 0; index < num_services; ++index) {
                printf("%p  %p  %p  %-7d  %-5d  %-4s  %-4d  %-s\n",
                       services[index].p,
                       services[index].next,
                       services[index].prev,
                       services[index].address,
                       services[index].timer / 1000,
                       services[index].socket_type == LS_DATAGRAM ? "D" : "S",
                       services[index].port,
                       services[index].name);
            }
        }
    } else if (strcmp(argv[1], "add") == 0) {
        if (argc == 5) {
            const char*       name = argv[2];
            ls_socket_type_t  type = toupper(argv[3][0]) == 'D' ? LS_DATAGRAM : LS_STREAM;
            int               port = strtol(argv[4], NULL, 0);

            if (! register_service_name(name, type, port)) {
                printf("Failed to register service %s on type %d port %d\n", name, type, port);
            }
        } else {
            printf("wrong number of parameters\n");
        }
    } else if (strcmp(argv[1], "del") == 0) {
        if (argc == 3) {
            if (!deregister_service_name(argv[2])) {
                printf("Failed to deregister service %s\n", argv[2]);
            }
        } else {
            printf("Wrong number of parameters\n");
        }
    } else {
        int               address;
        ls_socket_type_t  socket_type;
        ls_port_t         port;

        if (find_service_by_name(argv[1], &address, &socket_type, &port) == LSE_NO_ERROR) {
            printf("Service %s found on %d type %d port %d\n", argv[1], address, socket_type, port);
        } else {
            printf("Service %s not found\n", argv[1]);
        }
    }

    return rc;
}
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

bool init_service_names(void)
{
    INIT_LIST(&service_cache);

    service_lock = os_create_recursive_mutex();
    service_thread_id = os_create_thread(service_thread, "services", SERVICES_STACK_SIZE, 0, NULL);

    bool ok = service_lock && service_thread_id;

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    if (ok) {
        ok = add_command("service", service_table_commands);
    }
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

    return ok;
}

bool deinit_service_names(void)
{
    bool ok = true;

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    ok = remove_command("service");
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

    service_running = -1;

    /* Wait for service scanner to terminate */
    while (service_running != 0) {
        os_delay(100);
    }

    os_delete_thread(service_thread_id);
    service_thread_id = NULL;

    while (NUM_IN_LIST(&service_cache) != 0) {
        release_service(remove_service((service_cache_t*) FIRST_LIST_ITEM(&service_cache)));
    }

    os_delete_mutex(service_lock);

    return ok;
}

#endif /* CONFIG_LASTLINK_SERVICE_NAMES_ENABLE */
