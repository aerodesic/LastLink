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
    int                 dest;                 /* Destination we are trying to reach */
    int                 radio_num;            /* Radio number(s) of destination */
    int                 source;               /* Source address supplying the route */
    int                 sequence;             /* Sequence number of route */
    int                 metric;               /* Metric (hops) to this destination */
    int                 routeto;              /* Where we send the packet in order to reach the destination */
    uint8_t             flags;                /* Misc flags of destination */
    int                 lifetime;             /* Lifetime in milliseconds of this route before it expires */
    os_queue_t          pending_packets;      /* Packets to destination waiting for a route */
    os_timer_t          pending_timer;        /* Timer for retries to get route */
    int                 pending_retries;      /* Number of retries remaining */
} route_t;

/*
 * For delivering the contents for display.
 */
typedef struct {
    int                 dest;
    int                 radio_num;
    int                 source;
    int                 sequence;
    int                 metric;
    int                 routeto;
    uint8_t             flags;
    int                 lifetime;
    int                 pending_packets;
    bool                timer;
    int                 pending_retries;
} route_table_values_t;

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

route_t* route_create(route_table_t* rt, int radio_num, int dest, int routeto, int source, int sequence, int metric, uint8_t flags);
route_t* route_update(route_table_t* rt, int radio_num, int dest, int routeto, int source, int sequence, int metric, uint8_t flags);
bool route_remove(route_table_t* rt, int address);
bool route_delete(route_t* r);

void route_update_lifetime(route_t* r, int lifetime);
bool route_put_pending_packet(route_t* r, packet_t* p);
void route_start_routerequest(route_t* r);
void route_release_packets(route_t* r);
bool route_is_expired(route_t* r);

route_t* route_find(route_table_t* rt, int address);

int read_route_table(route_table_t* rt, route_table_values_t *values, int rtv_max);

#endif /* __routes_h_included */

