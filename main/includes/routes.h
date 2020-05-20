/*
 * routes.h
 *
 * Defines a route and methods on it.
 */
#ifndef __routes_h_included
#define __routes_h_included

#include "os_freertos.h"
#include "sdkconfig.h"
#include "listops.h"

#include "packets.h"

#define MAX_NUM_ROUTES              CONFIG_LASTLINK_MAX_ROUTES
#define ROUTE_LIFETIME              CONFIG_LASTLINK_ROUTE_LIFETIME
#define ROUTE_REQUEST_TIMEOUT       CONFIG_LASTLINK_ROUTE_REQUEST_TIMEOUT
#define ROUTE_REQUEST_RETRIES       CONFIG_LASTLINK_ROUTE_REQUEST_RETRIES
#define ROUTE_MAX_QUEUED_PACKETS    CONFIG_LASTLINK_ROUTE_MAX_QUEUED_PACKETS

typedef struct route route_t; // Forward decl

typedef struct route {
    route_t            *next;
    route_t            *prev;
    int                 dest;                 /* Destination we are trying to reach */
    int                 metric;               /* Metric (hops) to this destination */
    uint8_t             flags;                /* Misc flags of destination */
    int                 radio_num;            /* Radio number(s) of destination */
    int                 origin;              /* Address of node providing this route info */
    int                 sequence;             /* Sequence number of route */
    int                 routeto;              /* Where we send the packet in order to reach the destination */
    int                 lifetime;             /* Lifetime in milliseconds of this route before it expires */
    os_queue_t          pending_packets;      /* Packets to destination waiting for a route */
    os_timer_t          pending_timer;        /* Timer for retries to get route */
    int                 pending_retries;      /* Number of retries remaining */
} route_t;

void route_table_init(void);
bool route_table_deinit(void);
bool route_table_lock(void);
bool route_table_unlock(void);

route_t* create_route(int radio_num, int dest, int metric, int routeto, int origin, int sequence, int flags);
route_t* update_route(int radio_num, int dest, int metric, int routeto, int origin, int sequence, int flags);

bool route_remove(int address);
bool route_delete(route_t* r);

void route_update_lifetime(route_t* r, int lifetime);
bool route_put_pending_packet(route_t* r, packet_t* p);
void route_start_routerequest(route_t* r);
void route_release_packets(route_t* r);
bool route_is_expired(route_t* r);

route_t* find_route(int address);

#endif /* __routes_h_included */

