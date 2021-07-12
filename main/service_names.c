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
#include "lsocket_internal.h"
#include "configdata.h"

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

    /* source address of connection */
    ls_address_t      src_addr;

    /* Local port used for service */
    ls_port_t         src_port;

    /* Destination address receiving requests */
    ls_address_t      dest_addr;

    /* Destination port receiving requests */
    ls_port_t         dest_port;

    simpletimer_t     timer;

    uint16_t          lifetime;

    bool              local_service;

    /* Name of service */
    const char        *name;
} service_cache_t;

/* Service announcement */
typedef struct service_announce {
    ls_port_t    src_port;            /* Port to receive requests; address is sender of packet */
    ls_port_t    dest_port;           /* Dest port where packets are sent */
    ls_address_t dest_addr;
    uint8_t      socket_type;         /* LS_DATAGRAM or LS_STREAM */
    char         name[SERVICE_NAMES_MAX_LEN+1];
    uint16_t     lifetime;            /* Lifetime in seconds before renewal; auto renewed in lifetime/2 seconds */
} service_announce_t;

/* Cache table */
static list_head_t           service_cache;
static os_mutex_t            service_lock;      /* Mutex for exclusive access */
static os_thread_t           service_thread_id; /* Thread ID of scanner */
static int                   service_running;   /* 0 if stopped; 1 if running; -1 to kill */


static service_cache_t* create_service_entry(const char* name, int address)
{
    service_cache_t* service = (service_cache_t*) malloc(sizeof(service_cache_t));
    if (service != NULL) {
        memset(service, 0, sizeof(service_cache_t));
        if (name != NULL) {
            service->name = strdup(name);
            service->src_addr = address;
        }
    }

    return service;
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


static service_cache_t *lookup_service(const char* name, int address)
{
    service_cache_t *found = NULL;

    os_acquire_recursive_mutex(service_lock);

    service_cache_t *service = (service_cache_t*) FIRST_LIST_ITEM(&service_cache);

    while (found == NULL && service != NULL) {
        if (strcmp(service->name, name) == 0 && service->src_addr == address) {
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
static service_cache_t *create_service(const char* name, int address)
{
    service_cache_t *service = NULL;

    os_acquire_recursive_mutex(service_lock);

    if (! lookup_service(name, address)) {
        service = create_service_entry(name, address);

        if (service != NULL) {
            service->socket_type = LS_UNUSED;
            service->src_port = 0;
            service->dest_port = 0;
            simpletimer_stop(&service->timer);

            ADD_TO_LIST(&service_cache, service);
        }
    }

    os_release_recursive_mutex(service_lock);

    return service;
}

/*
 * Thread the periodically publishes service names to the network and receives published service names from the network.
 */
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
                while (service_running > 0) {
                    service_announce_t service_announce;
                    int service_address;
                    int sender_port;

                    /* Wait for a service announcement (and acts as timer) */
                    int len = ls_read_with_address(socket, (char*) &service_announce, sizeof(service_announce), &service_address, &sender_port, SERVICE_NAMES_WAIT_TIMEOUT);
//  if (len >= 0) {
//  ESP_LOGD(TAG, "%s: packet %d bytes from %d/%d", __func__, len, service_address, sender_port);
//}

                    if (len == sizeof(service_announce)) {

                        //ESP_LOGI(TAG, "%s: announce: '%s' %s src_addr %d src_port %d dest_addr %d dest_port %d lifetime %d", __func__,
                        //               service_announce.name,
                        //               service_announce.socket_type == LS_DATAGRAM ? "Datagram" : "Stream",
                        //               service_address,
                        //               service_announce.src_port,
                        //               service_announce.dest_addr,
                        //               service_announce.dest_port,
                        //               service_announce.lifetime);

                        os_acquire_recursive_mutex(service_lock);

                        service_cache_t *service = lookup_service(service_announce.name, service_address);

                        //ESP_LOGI(TAG, "%s: service %s/%d %sfound", __func__, service_announce.name, service_address, service ? "" : "not ");

                        /* If service entry not found, create one */
                        if (service == NULL) {
                            service = create_service(service_announce.name, service_address);
                            //ESP_LOGI(TAG, "%s: service %s/%d %screated", __func__, service_announce.name, service_address, service ? "" : "not ");
                        }

                        if (service != NULL) {
                            service->socket_type = service_announce.socket_type;
                            service->src_port    = service_announce.src_port;
                            service->dest_addr   = service_announce.dest_addr;
                            service->dest_port   = service_announce.dest_port;
                            service->lifetime    = service_announce.lifetime;

                            /* Set service lifetime timer - goes away when this timer expires */
                            simpletimer_start(&service->timer, service_announce.lifetime * 1000);
                        }

                        os_release_recursive_mutex(service_lock);
                    }

                    /* Do a scan and expire entries and renew service announcements for local services */
                    os_acquire_recursive_mutex(service_lock);

                    service_cache_t *service = (service_cache_t*) FIRST_LIST_ITEM(&service_cache);

                    while (service != NULL) {
                        // ESP_LOGD(TAG, "%s: looking at service %p: '%s' address %d port %d", __func__, service, service->name, service->address, service->port);

                        if (simpletimer_is_expired(&service->timer)) {
                            /* If local service - re-announce */
                            if (service->local_service) {
                                int announce_socket = ls_socket(LS_DATAGRAM);
                                if (announce_socket >= 0) {
                                    int ret = ls_bind(announce_socket, 0);
                                    if (ret == LSE_NO_ERROR) {
                                        ret = ls_connect(announce_socket, BROADCAST_ADDRESS, SERVICE_NAMES_PORT);
                                        if (ret == LSE_NO_ERROR) {
                                            /* Announce the service */
                                            service_announce_t service_announce;

                                            service_announce.src_port = service->src_port;
                                            service_announce.dest_addr = service->dest_addr;
                                            service_announce.dest_port = service->dest_port;
                                            service_announce.socket_type = service->socket_type;
                                            service_announce.lifetime = service->lifetime;
                                            strncpy(service_announce.name, service->name, SERVICE_NAMES_MAX_LEN);

                                            ret = ls_write(announce_socket, &service_announce, sizeof(service_announce));
                                            if (ret < 0) {
                                                ESP_LOGE(TAG, "%s: write failed %d", __func__, ret);
                                            } else {
//                                                ESP_LOGI(TAG, "%s: service %s type %s on port %d posted", __func__, service->name, (service->socket_type == LS_STREAM ? "S" : "D"), service->port);
                                            }
                                        } else {
                                            ESP_LOGE(TAG, "%s: connect to %d failed %d", __func__, announce_socket, ret);
                                        }
                                    } else {
                                        ESP_LOGE(TAG, "%s: bind failed %d", __func__, ret);
                                    }
                                    ls_close(announce_socket);
                                } else {
                                    ESP_LOGE(TAG, "%s: announce_socket create failed %d", __func__, announce_socket);
                                }

                                /* Start the timer for 1/2 the lifetime */
                                simpletimer_start(&service->timer, service->lifetime * 1000 / 2);
                            } else {
                                /* Service has expired */
                                release_service(remove_service(service));
                            }
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

    ESP_LOGE(TAG, "%s: scanner stopped: service_running %d socket %d", __func__, service_running, socket);

    service_running = 0;
}

/************************************************************************************
 * User API for service_names below                                                 *
 ************************************************************************************/

/*
 * User-callable function to locate a service.
 *
 * Entry:
 *    name                 Name of service
 *    src_addr             Source address owner of service
 *    socket_type          pointer to return socket_type of service
 *    src_port             pointer to return port number of service
 *    dest_addr            pointer to return node address receiving service notifications
 *    dest_port            pointer to return port address receiving service notifications
 *                        
 * Returns LSE_NO_ERROR if found and values returned otherwise ls_error_t.
 */
ls_error_t find_service(const char* name, ls_address_t src_addr, ls_socket_type_t *socket_type, ls_port_t* src_port, ls_address_t* dest_addr, ls_port_t* dest_port)
{
    ls_error_t  ret = LSE_FAILED;

    service_cache_t *service = lookup_service(name, src_addr);

    if (service != NULL) {
        /* Found it so return the values */
        *socket_type = service->socket_type;
        if (src_port != NULL) {
            *src_port = service->src_port;
        }
        if (dest_addr != NULL) {
            *dest_addr = service->dest_addr;
        }
        if (dest_port != NULL) {
            *dest_port = service->dest_port;
        }

        ret = LSE_NO_ERROR;
    }

    return ret;
}

/*
 * Find all services that match the name and return their source addresses.
 *
 * Don't store past the end of the source address list but return the number
 * of services found.
 */
int find_all_services(const char* name, ls_address_t* addresses, int num_addresses)
{
    int services_found = 0;

    os_acquire_recursive_mutex(service_lock);

    service_cache_t *service = (service_cache_t*) FIRST_LIST_ITEM(&service_cache);
    while (service != NULL) {
        if (strcmp(name, service->name) == 0) {
            if (services_found < num_addresses) {
                addresses[services_found] = service->src_addr;
            }
            ++services_found;
        }
        service = NEXT_LIST_ITEM(service, &service_cache);
    }

    os_release_recursive_mutex(service_lock);

    return services_found;
}


/*
 * Register a new service
 *
 * Entry:
 *     name             Name of service
 *     s                Socket handling registered service
 *     lifetime         lifetime of service announcement (0 is default)
 *                      Republishes service each lifetime / 2 seconds.
 *
 * Returns true if successful.
 */
bool register_service(const char* name, int s, uint16_t lifetime)
{
    service_cache_t *service = NULL;

    os_acquire_recursive_mutex(service_lock);

    ls_socket_t* socket = validate_socket(s);

    if (socket != NULL) {

        service = create_service(name, get_config_int("lastlink.address", -1));

        if (service != NULL) {

            service->socket_type = socket->socket_type;
            service->src_port = socket->src_port;
            service->dest_port = socket->dest_port;
            service->dest_addr = socket->dest_addr;
            service->local_service = true;

            /* Leave timer stopped for permanent (until deregistered) service entries */
            if (lifetime == 0) {
                service->lifetime = SERVICE_NAMES_DEFAULT_LIFETIME;
            } else {
                service->lifetime = lifetime;
            }

            /* Expire the timer so it gets published immediately */
            simpletimer_set_expired(&service->timer);
        }

        unlock_socket(socket);
    }

    os_release_recursive_mutex(service_lock);


    return service != NULL;
}


bool deregister_service(const char* name)
{
    os_acquire_recursive_mutex(service_lock);

    service_cache_t* service = lookup_service(name, get_config_int("lastlink.address", -1));

    if (service != NULL) {
        if (service->local_service) {
            release_service(remove_service(service));
        } else {
            printf("Service %s on %d is not a local service\n", name, service->src_addr);
        }
    }

    os_release_recursive_mutex(service_lock);

    return service != NULL;
}

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
typedef struct service_cache_copy {
    void*             p;
    void*             prev;
    void*             next;
    ls_socket_type_t  socket_type;
    bool              local_service;
    ls_address_t      src_addr;
    ls_port_t         src_port;
    ls_address_t      dest_addr;
    ls_port_t         dest_port;
    int               timer;         /* Timeout for autodestruct of service table entry */
    char              name[SERVICE_NAMES_MAX_LEN+1];
} service_cache_copy_t;

static int service_table_commands(int argc, const char **argv)
{
    int rc = 0;

    if (argc == 0) {
        show_help(argv[0], "", "List service cache");

    } else if (argc == 2) {
#define MAX_ADDRESSES_IN_SEARCH   20
        ls_address_t      addresses[MAX_ADDRESSES_IN_SEARCH];

        int addresses_found = find_all_services(argv[1], addresses, MAX_ADDRESSES_IN_SEARCH);

        for (int index = 0; index < addresses_found; ++index) {
            ls_socket_type_t  socket_type = LS_UNUSED;
            int               src_port = -1;
            int               dest_addr = -1;
            int               dest_port = -1;

            if (find_service(argv[1], addresses[index], &socket_type, &src_port, &dest_addr, &dest_port) == LSE_NO_ERROR) {
                printf("Service %s found source %d/%d dest %d/%d type %d\n", argv[1], addresses[index], src_port, dest_addr, dest_port, socket_type);
            } else {
                printf("Service %s not found\n", argv[1]);
            }

            if (addresses_found > MAX_ADDRESSES_IN_SEARCH) {
                printf("Only the first %d of %d services displayed\n", MAX_ADDRESSES_IN_SEARCH, addresses_found);
            }
        }
    } else {
        os_acquire_recursive_mutex(service_lock);

        service_cache_t *service = (service_cache_t*) FIRST_LIST_ITEM(&service_cache);

        service_cache_copy_t services[NUM_IN_LIST(&service_cache)];

        int num_services = 0;

        while (service != NULL) {
            services[num_services].p = service;
            services[num_services].next = service->next;
            services[num_services].prev = service->prev;
            services[num_services].socket_type = service->socket_type;
            services[num_services].local_service = service->local_service;
            services[num_services].src_addr = service->src_addr;
            services[num_services].src_port = service->src_port;
            services[num_services].dest_addr = service->dest_addr;
            services[num_services].dest_port = service->dest_port;
            services[num_services].timer = simpletimer_is_running(&service->timer) ? simpletimer_remaining(&service->timer) : -1;
            strncpy(services[num_services].name, service->name, SERVICE_NAMES_MAX_LEN);
            services[num_services].name[SERVICE_NAMES_MAX_LEN] = '\0';

            num_services++;

            service = NEXT_LIST_ITEM(service, &service_cache);
        }

        os_release_recursive_mutex(service_lock);

        if (num_services != 0) {
            printf("P           Next        Prev        Local  Type  Source       Destination  Timer  Name\n");
            //      0xXXXXXXXX  0xXXXXXXXX  0xXXXXXXXX  XXXXX  XXXX  AAAAAA/PPPP  AAAAAA/PPPP  XXXXX  XX..."
            for (int index = 0; index < num_services; ++index) {
                char src_address[30];
                char dest_address[30];

                if (services[index].src_addr == BROADCAST_ADDRESS) {
                    sprintf(src_address, "BCAST/%d", services[index].src_port);
                } else {
                    sprintf(src_address, "%d/%d", services[index].src_addr, services[index].src_port);
                }

                if (services[index].dest_addr == BROADCAST_ADDRESS) {
                    sprintf(dest_address, "BCAST/%d", services[index].dest_port);
                } else {
                    sprintf(dest_address, "%d/%d", services[index].dest_addr, services[index].dest_port);
                }

                printf("%p  %p  %p  %-5s  %-4s  %-11s  %-11s  %-5d  %-s\n",
                       services[index].p,
                       services[index].next,
                       services[index].prev,
                       services[index].local_service ? "True" : "False",
                       services[index].socket_type == LS_DATAGRAM ? "D" : "S",
                       src_address,
                       dest_address,
                       services[index].timer / 1000,
                       services[index].name);
            }
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
        ok = add_command("services", service_table_commands);
    }
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

    return ok;
}

bool deinit_service_names(void)
{
    bool ok = true;

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    ok = remove_command("services");
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

    if (service_running != 0) {
        service_running = -1;

        /* Wait for service scanner to terminate */
        while (service_running != 0) {
            os_delay(100);
        }
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
