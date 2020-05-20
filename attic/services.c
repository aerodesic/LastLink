/*
 * services.c
 *
 * Protocol used to locate services by name.
 */

#include "sdkconfig.h"

#ifdef CONFIG_LASTLINK_SERVICE_NAME_ENABLE

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

#include "os_freertos.h"
#include "lsocket_internal.h"
#include "packets.h"
#include "linklayer.h"

#include "listops.h"
#include "services.h"

#if CONFIG_LASTLINK_TABLE_COMMANDS
#include "commands.h"
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */

#define TAG "services"

typedef struct service_cache service_cache_t;

#define SERVICE_NAME_REPLY_TIME    CONFIG_LASTLINK_SERVICE_NAME_REPLY_TIMEOUT  /* Time before re-request during REQUESTING state */
#define SERVICE_NAME_ROUTED_TIME   CONFIG_LASTLINK_SERVICE_NAME_ROUTED_TIME    /* Amount of time to remember a packet to prevent redundant flooding */
#define SERVICE_NAME_LIFETIME      CONFIG_LASTLINK_SERVICE_NAME_LIFETIME       /* Time before discard in ESTABLISHED state */
#define SERVICE_NAME_TICK_TIME     CONFIG_LASTLINK_SERVICE_NAME_TICK           /* Tick interval */
#define SERVICE_NAME_MAX_LEN       CONFIG_LASTLINK_SERVICE_NAME_MAX_LEN        /* Max length of a name */
#define SERVICE_NAME_RETRIES       CONFIG_LASTLINK_SERVICE_NAME_RETRIES        /* Max number of retries */

typedef struct service_cache {
    service_cache_t   *next;
    service_cache_t   *prev;

    enum {
         SCS_IDLE = 0,    /* Service is idle - scanner won't touch */
         SCS_ROUTED,      /* Marker to remember we have already seen this packet - don't forward */
                          /* Lookup is by address and sequence only */
         SCS_LOCAL,       /* Locally defined service, does not time out */
         SCS_REQUESTING,  /* Service is requestined - scanner will resend servicerequests */
         SCS_ESTABLISHED, /* Service has been announced - scanner will delete on timeout */
    } state;

    os_queue_t        announce_queue;   /* Announce queue - announced services come here */

    int               address;          /* Owner of this service */
    int               sequence;         /* If a cached sequence pending to avoid retransmission, this is non-zero, otherwise 0 if fixed entry */
    int               timer;            /* Kills service by this sequence in this many scans */
    int               retries;          /* Retry announce countdown */

    /* This is the socket type of the service.  If LS_UNKNOWN, then this is a pending service lookup entry */
    ls_socket_type_t  socket_type;

    /* Port used for service */
    ls_port_t         port;

    /* Name of service if known */
    char              name[SERVICE_NAME_MAX_LEN+1];
} service_cache_t;

/* Cache table */
static list_head_t           service_cache;
static os_mutex_t            service_lock;      /* Mutex for exclusive access */
static os_thread_t           service_thread;    /* Thread ID of scanner */
static int                   service_running;   /* 0 if stopped; 1 if running; -1 to kill */

/****************************************************************************************************
 * Service packet processing                                                                        *
 ****************************************************************************************************/

/*
 * ServiceAnnounce is used to advertise a route to a node with a service.
 */
packet_t* serviceannounce_packet_create(int dest, int sequence, const char* name, ls_socket_type_t socket_type, ls_port_t port)
{
    packet_t* packet = linklayer_create_generic_packet(dest, SERVICEANNOUNCE_PROTOCOL, SERVICEANNOUNCE_LEN);
    if (packet != NULL) {
        set_uint_field(packet, SERVICEANNOUNCE_SEQUENCE, SEQUENCE_NUMBER_LEN, sequence);
        set_uint_field(packet, SERVICEANNOUNCE_SOCKET_TYPE, SOCKET_TYPE_LEN, socket_type);
        set_uint_field(packet, SERVICEANNOUNCE_PORT, PORT_NUMBER_LEN, port);
        set_str_field(packet,  SERVICEANNOUNCE_NAME, SERVICE_NAME_MAX_LEN, name);
        set_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
        set_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN, linklayer_node_address);
    }

    return packet;
}

static service_cache_t *lookup_service_by_sequence_and_state(int address, int sequence, int state)
{
    os_acquire_recursive_mutex(service_lock);

    service_cache_t *first_service = (service_cache_t*) HEAD_OF_LIST(&service_cache);
    service_cache_t *service = first_service;

    service_cache_t *found = NULL;

    if (service != NULL) {
        do {
            if (state == service->state && address == service->address && sequence == service->sequence) {
                found = service;
            } else {
                service = service->next;
            }
        } while (found == NULL && (service != first_service));
    }

    os_release_recursive_mutex(service_lock);

    return found;
}

static service_cache_t* create_service(const char* name)
{
    service_cache_t* service = (service_cache_t*) malloc(sizeof(service_cache_t));
    if (service != NULL) {
        memset(service, 0, sizeof(service_cache_t));
        if (name != NULL) {
            strncpy(service->name, name, SERVICE_NAME_MAX_LEN);
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
    if (service->announce_queue != NULL) {
        os_delete_queue(service->announce_queue);
    }
    free((void*) service);
}


/* return true if packet has already been handled */
static bool service_packet_already_routed(packet_t *packet)
{
    int address = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
    int sequence = get_uint_field(packet, SERVICEREQUEST_SEQUENCE, SEQUENCE_NUMBER_LEN);

    service_cache_t *service = lookup_service_by_sequence_and_state(address, sequence, SCS_ROUTED);

    if (service == NULL) {
        /* First time through - create marker and it will prevent new routes */
        service_cache_t *routed = create_service(NULL);
        if (routed != NULL) {
            routed->address = address;
            routed->sequence = sequence;
            routed->state = SCS_ROUTED;
            routed->timer = SERVICE_NAME_ROUTED_TIME;

            /* Add to cache - will be deleted by timer if not otherwise */
            add_service(routed);
        }
    }

    return service != NULL;
}

/*
 * Return service if newly created, otherwise NULL
 */
static service_cache_t *create_service_by_announce_packet(packet_t *packet)
{
    service_cache_t *service = NULL;

    int address = get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN);
    int sequence = get_uint_field(packet, SERVICEANNOUNCE_SEQUENCE, SEQUENCE_NUMBER_LEN);

    os_acquire_recursive_mutex(service_lock);

    if (! lookup_service_by_sequence_and_state(address, sequence, SCS_REQUESTING)) {
        service = create_service(NULL);

        if (service != NULL) {
            service->sequence = sequence;
            service->address = address;
            service->socket_type = get_uint_field(packet, SERVICEANNOUNCE_SOCKET_TYPE, SOCKET_TYPE_LEN);
            service->port = get_uint_field(packet, SERVICEANNOUNCE_PORT, PORT_NUMBER_LEN);
            const char* name = get_str_field(packet, SERVICEANNOUNCE_NAME, SERVICE_NAME_MAX_LEN);
            strncpy(service->name, name, SERVICE_NAME_MAX_LEN);
            free((void*) name);

            service->state = SCS_ESTABLISHED;
            service->timer = SERVICE_NAME_LIFETIME;

            add_service(service);
        }
    }

    os_release_recursive_mutex(service_lock);

    return service;
}

static void release_pending_service_request_by_name(service_cache_t *new_service)
{
    service_cache_t *first_service = (service_cache_t*) HEAD_OF_LIST(&service_cache);
    service_cache_t *service = first_service;

    if (service != NULL) {
        do {
            if (service->state == SCS_REQUESTING && strcmp(new_service->name, service->name) == 0) {
                os_put_queue(service->announce_queue, (os_queue_item_t) new_service);
            }
            service = service->next;
        } while (service != first_service);
    }
}

static bool serviceannounce_packet_process(packet_t* packet)
{
    bool processed = false;

    if (packet != NULL) {
        /* If packet is for us, see if we have an outstanding service request */
        if (linklayer_packet_is_for_this_node(packet)) {
            os_acquire_recursive_mutex(service_lock);

            service_cache_t *service = create_service_by_announce_packet(packet);

            if (service != NULL) {
                release_pending_service_request_by_name(service);
            }

            os_release_recursive_mutex(service_lock);

            processed = true;

        } else {
            /* Route packet */
            set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, NULL_ADDRESS);
            linklayer_send_packet_update_ttl(ref_packet(packet));
            processed = true;
        }

        release_packet(packet);
    }

    return processed;
}

static const char* serviceannounce_packet_format(const packet_t* packet)
{
    char* info;

    int sequence = get_uint_field(packet, SERVICEANNOUNCE_SEQUENCE, SEQUENCE_NUMBER_LEN);
    int socket_type = get_uint_field(packet, SERVICEANNOUNCE_SOCKET_TYPE, SOCKET_TYPE_LEN);
    int port = get_uint_field(packet, SERVICEANNOUNCE_PORT, PORT_NUMBER_LEN);
    const char* name = get_str_field(packet, SERVICEANNOUNCE_NAME, SERVICE_NAME_MAX_LEN);

    asprintf(&info, "Service Announce: seq %d socket_type %d port %d name '%s'", sequence, socket_type, port, name);
    free((void*) name);

    return info;
}

/*
 * A service request is sent by a node when it needs to know a route to a node with
 * a specific service.
 *
 * This packet will be rebroadcast until it reaches all nodes.
 */
packet_t* servicerequest_packet_create(const char* name)
{
    packet_t* packet = linklayer_create_generic_packet(BROADCAST_ADDRESS, SERVICEREQUEST_PROTOCOL, SERVICEREQUEST_LEN);

    if (packet != NULL) {
        set_uint_field(packet, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, BROADCAST_ADDRESS);
        set_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN, linklayer_node_address);
        set_uint_field(packet, HEADER_SENDER_ADDRESS, ADDRESS_LEN, linklayer_node_address);
        set_uint_field(packet, SERVICEREQUEST_SEQUENCE, SEQUENCE_NUMBER_LEN, linklayer_allocate_sequence());
        set_str_field(packet, SERVICEREQUEST_NAME, SERVICE_NAME_MAX_LEN, name);
    }

    return packet;
}

static service_cache_t *lookup_service_by_name(const char* name)
{
    service_cache_t *found = NULL;

    os_acquire_recursive_mutex(service_lock);

    service_cache_t *first_service = (service_cache_t*) HEAD_OF_LIST(&service_cache);
    service_cache_t *service = first_service;

    if (service != NULL) {
        do {
            if ((service->state == SCS_LOCAL || service->state == SCS_ESTABLISHED) && strcmp(service->name, name) == 0) {
                found = service;
            } else {
                service = service->next;
            }
        } while (found == NULL && (service != first_service));
    }

    os_release_recursive_mutex(service_lock);

    return found;
}

static bool servicerequest_packet_process(packet_t* packet)
{
    bool processed = false;

    if (packet != NULL) {

        /* If packet is for one of our services, process it */
        const char* name = get_str_field(packet, SERVICEREQUEST_NAME, SERVICE_NAME_MAX_LEN);
        service_cache_t * service = lookup_service_by_name(name);

        if (service != NULL) {
            /* WIN! - send announce back to caller */
            packet_t *announce = serviceannounce_packet_create(
                                        get_uint_field(packet, HEADER_ORIGIN_ADDRESS, ADDRESS_LEN),
                                        get_uint_field(packet, SERVICEREQUEST_SEQUENCE, SEQUENCE_NUMBER_LEN),
                                        service->name,
                                        service->socket_type,
                                        service->port);
            linklayer_send_packet(announce);
            processed  = true;

        } else if (! service_packet_already_routed(packet)) {
            /* Decide it to forward */
            linklayer_send_packet_update_ttl(ref_packet(packet));
            processed = true;
        }

        release_packet(packet);
    }

    return processed;
}

static const char* servicerequest_packet_format(const packet_t* packet)
{
    char* info;

    int sequence = get_uint_field(packet, SERVICEREQUEST_SEQUENCE, SEQUENCE_NUMBER_LEN);
    const char* name = get_str_field(packet, SERVICEREQUEST_NAME, SERVICE_NAME_MAX_LEN);

    asprintf(&info, "Service Request: seq %d name '%s'", sequence, name);
    free((void*) name);

    return info;
}

/*
 * Send a new service request packet
 *
 * Always called with service_lock set.
 *
 * Returns true of packet send; false otherwise.
 */
static bool send_service_request_packet(service_cache_t *service)
{
    bool success = false;

    if (service->state == SCS_IDLE) {
        /* Create announce queue and set retries */
        service->announce_queue = os_create_queue(1, sizeof(service_cache_t*));
        service->retries = SERVICE_NAME_RETRIES + 1;
        service->state = SCS_REQUESTING;
    }

    if (--(service->retries) != 0) {
        /* Create a new service request packet */
        packet_t* packet = servicerequest_packet_create(service->name);

        if (packet != NULL) {
            /* Save it's sequence number for reply test (new one generated on each request) */
            service->sequence = get_uint_field(packet, SERVICEREQUEST_SEQUENCE, SEQUENCE_NUMBER_LEN);

            /* Wait this long for reply before retransmitting */
            service->timer = SERVICE_NAME_REPLY_TIME;

            /* Send the packet */
            linklayer_send_packet(packet);       

            success = true;
        }
    }

    return success;
}

static void service_scanner_thread(void* param)
{
    service_running = 1;

ESP_LOGI(TAG, "%s: running", __func__);

    while (service_running > 0) {

        os_acquire_recursive_mutex(service_lock);

        service_cache_t *first_service = (service_cache_t*) HEAD_OF_LIST(&service_cache);
        service_cache_t *service = first_service;

        if (service != NULL) {
            do {
//ESP_LOGI(TAG, "%s: testing %s state %d", __func__, service->name, service->state);
                if (service->state != SCS_IDLE && service->state != SCS_LOCAL) {
                    if (service->timer > 0) {
                        service->timer -= SERVICE_NAME_TICK_TIME;
                    }

                    if (service->timer <= 0) {
                        bool processed = false;

                        /* Established and temporary marked items are deleted on timeout */
                        if (service->state == SCS_ESTABLISHED || service->state == SCS_ROUTED) {
                            release_service(remove_service(service));
                            processed = true;

                        /* Requesting items issue a new request on timeout and disappear if retry fails */
                        } else if (service->state == SCS_REQUESTING) {
                            if (send_service_request_packet(service)) {
                                /* New request sent - wait until next time */
ESP_LOGI(TAG, "%s: resending service request for %s", __func__, service->name);
                            } else {
                                /* Send failure to announce queue (caller will tear down infrastructure) */
ESP_LOGI(TAG, "%s: service request for %s failed", __func__, service->name);
                                os_put_queue(service->announce_queue, (os_queue_item_t) NULL);
                                processed = true;
                            }
                        }

                        if (processed) {
                            /* Only do one release per cycle */
                            service = first_service = NULL;
                        }
                    }
                }

                if (service != NULL) {
                    service = service->next;
                }
            } while (service != first_service);
        }

        os_release_recursive_mutex(service_lock);

        os_delay(SERVICE_NAME_TICK_TIME);
    }

    ESP_LOGE(TAG, "%s: scanner stopped", __func__);

    service_running = 0;
}

/************************************************************************************
 * User API for services below                                                      *
 ************************************************************************************/

/*
 * User-callable function to locate a service, broadcasting packets and necessary.
 */
bool find_service_by_name(const char* name, int *address, ls_socket_type_t *socket_type, ls_port_t* port)
{

    service_cache_t *service = lookup_service_by_name(name);

    if (service == NULL) {
        os_acquire_recursive_mutex(service_lock);

        /* Create a temporary service in the table with an address of zero */
        service = create_service(name);

        add_service(service);

        /* Send out a service request packet (broadcast flood) */
        send_service_request_packet(service);

        os_release_recursive_mutex(service_lock);

        /* Wait for the serviceannounce packet to arrive or a timeout */
        service_cache_t *reply;
        if (service->announce_queue != NULL) {
            os_get_queue(service->announce_queue, (os_queue_item_t*) &reply);
        } else {
            reply = NULL;
        }

        /* Release the temporary request service */
        release_service(service);

        /* Use the reply from the request (it might be NULL) */
        service = reply;
    }

    if (service != NULL) {
        /* Found it so return the values */
        *address = service->address;
        *socket_type = service->socket_type;
        *port = service->port;
    }

    return service != NULL;;
}


/*
 * Register a new service
 */
bool register_service(const char* name, ls_socket_type_t socket_type, ls_port_t port)
{
    bool success = false;

    if (! lookup_service_by_name(name)) {
        service_cache_t *service = create_service(name);
        if (service != NULL) {

            service->socket_type = socket_type;
            service->port = port;
            service->state = SCS_LOCAL;

            add_service(service);

            success = true;
        }
    }
    return success;
}


bool deregister_service(const char* name)
{
    service_cache_t* service = lookup_service_by_name(name);

    if (service != NULL) {
        free((void*) remove_service(service));
    }

    return service != NULL;
}

#if CONFIG_LASTLINK_TABLE_COMMANDS
typedef struct service_cache_copy {
    void*             p;
    void*             prev;
    void*             next;
    int               address;       /* Owner of this service */
    int               sequence;      /* If a cached sequence pending to avoid retransmission, this is non-zero, otherwise 0 if fixed entry */
    int               timer;         /* Kills service by this sequence in this many scans */
    ls_socket_type_t  socket_type;
    ls_port_t         port;
    char              name[SERVICE_NAME_MAX_LEN+1];
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

        service_cache_t *first_service = (service_cache_t*) HEAD_OF_LIST(&service_cache);
        service_cache_t *service = first_service;

        service_cache_copy_t services[NUM_IN_LIST(&service_cache)];

        int num_services = 0;
    
        if (service != NULL) {
            do {
                services[num_services].p = service;
                services[num_services].next = service->next;
                services[num_services].prev = service->prev;
                services[num_services].sequence = service->sequence;
                services[num_services].address = service->address;
                services[num_services].timer = service->timer;
                services[num_services].socket_type = service->socket_type;
                services[num_services].port = service->port;
                strncpy(services[num_services].name, service->name, SERVICE_NAME_MAX_LEN);
                services[num_services].name[SERVICE_NAME_MAX_LEN] = '\0';

                num_services++;

                service = service->next;
            } while (service != first_service);
        }

        os_release_recursive_mutex(service_lock);
    
        if (num_services != 0) {
            printf("P           Next        Prev        Address  Sequence  Timer  Type  Port  Name\n");
            for (int index = 0; index < num_services; ++index) {
                printf("%p  %p  %p  %-7d  %-8d  %-5d  %-4s  %-4d  %-s\n",
                       services[index].p,
                       services[index].next,
                       services[index].prev,
                       services[index].address,
                       services[index].sequence,
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

            if (! register_service(name, type, port)) {
                printf("Failed to register service %s on type %d port %d\n", name, type, port);
            }
        } else {
            printf("wrong number of parameters\n");
        } 
    } else if (strcmp(argv[1], "del") == 0) {
        if (argc == 3) {
            if (!deregister_service(argv[2])) {
                printf("Failed to deregister service %s\n", argv[2]);
            }
        } else {
            printf("Wrong number of parameters\n");
        }
    } else {
        int               address;
        ls_socket_type_t  socket_type;
        ls_port_t         port; 
        
        if (find_service_by_name(argv[1], &address, &socket_type, &port)) {
            printf("Service %s found on %d type %d port %d\n", argv[1], address, socket_type, port);
        } else {
            printf("Service %s not found\n", argv[1]);
        }
    }

    return rc;
}
#endif /* CONFIG_LASTLINK_TABLE_COMMANDS */

bool init_services(void)
{
    service_lock = os_create_recursive_mutex();
    service_thread = os_create_thread(service_scanner_thread, "service_scanner", 8192, 0, NULL);

    bool ok = service_lock && service_thread;

ESP_LOGI(TAG, "%s: service_lock and service_thread %s", __func__, ok ? "TRUE" : "FALSE");

    if (ok) {
        ok = linklayer_register_protocol(SERVICEREQUEST_PROTOCOL, servicerequest_packet_process, servicerequest_packet_format) &&
             linklayer_register_protocol(SERVICEANNOUNCE_PROTOCOL, serviceannounce_packet_process, serviceannounce_packet_format);
    }

ESP_LOGI(TAG, "%s: protocol register %s", __func__, ok ? "TRUE" : "FALSE");

#if CONFIG_LASTLINK_TABLE_COMMANDS
    if (ok) {
        ok = add_command("service", service_table_commands);
ESP_LOGI(TAG, "%s: add service command %s", __func__, ok ? "TRUE" : "FALSE");
    }
#endif

    return ok;
}

bool deinit_services(void)
{
    bool ok = true;

#if CONFIG_LASTLINK_TABLE_COMMANDS
    ok = remove_command("service");
#endif

    ok = linklayer_unregister_protocol(SERVICEREQUEST_PROTOCOL) &&
         linklayer_unregister_protocol(SERVICEANNOUNCE_PROTOCOL);

    service_running = -1;

    /* Wait for service scanner to terminate */
    while (service_running != 0) {
        os_delay(100);
    }

    os_delete_thread(service_thread);
    service_thread = NULL;

    while (NUM_IN_LIST(&service_cache) != 0) {
        release_service(remove_service((service_cache_t*) HEAD_OF_LIST(&service_cache)));
    }

    os_delete_mutex(service_lock);

    return ok;
}

#endif /* CONFIG_LASTLINK_SERVICE_NAME_ENABLE */
