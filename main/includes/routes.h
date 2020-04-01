/*
 * routes.h
 *
 * Defines a route and methods on it.
 */
#ifndef __routes_h_included
#define __routes_h_included

#include "os_freertos.h"

#include "sdkconfig.h"

#define MAX_NUM_ROUTES              CONFIG_LASTLINK_MAX_ROUTES
#define ROUTE_LIFETIME              CONFIG_LASTLINK_ROUTE_LIFETIME
#define ROUTE_REQUEST_TIMEOUT       CONFIG_LASTLINK_ROUTE_REQUEST_TIMEOUT
#define ROUTE_REQUEST_RETRIES       CONFIG_LASTLINK_ROUTE_REQUEST_RETRIES
#define ROUTE_MAX_QUEUED_PACKETS    CONFIG_LASTLINK_ROUTE_MAX_QUEUED_PACKETS

typedef struct route_table route_table_t;

typedef struct route {
    struct route*       next;
    struct route*       prev;
    route_table_t*      table;
    int                 dest;
    int                 radio_num;
    int                 sequence;
    int                 metric;
    int                 routeto;
    uint8_t             flags;
    int                 lifetime;
    packet_t*           pending_request;
    os_queue_t          pending_packets;
    os_timer_t          pending_timer;
    int                 pending_retries;
} route_t;

typedef struct route_table {
    route_t*            routes;
    int                 num_routes;
    os_mutex_t          lock;
    os_thread_t         expire_thread;
} route_table_t;

route_table_t* route_table_init(route_table_t* rt);
bool route_table_deinit(route_table_t* rt);
bool route_table_lock(route_table_t* rt);
bool route_table_unlock(route_table_t* rt);

route_t* route_create(route_table_t* rt, int radio_num, int target, int nexthop, int sequence, int metric, uint8_t flags);
route_t* route_update(route_table_t* rt, int radio_num, int target, int nexthop, int sequence, int metric, uint8_t flags);
bool route_remove(route_table_t* rt, int address);
bool route_delete(route_t* r);

void route_update_lifetime(route_t* r, int lifetime);
bool route_put_pending_packet(route_t* r, packet_t* p);
void route_set_pending_request(route_t* r, packet_t* request, int retries, int timeout);
void route_release_packets(route_t* r);
bool route_is_expired(route_t* r);

route_t* route_find(route_table_t* rt, int address);


#endif /* __routes_h_included */

