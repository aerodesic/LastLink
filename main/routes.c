/*
 * routes.c
 *
 * Handle routes
 */

#include "time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "linklayer.h"
#include "packets.h"
#include "routes.h"

static route_t*           routes;
static int                num_routes;
static SemaphoreHandle_t  route_mutex;

bool route_table_lock(route_table_t* rt)
{
    bool ok = false;

    if (rt != NULL) {
        ok = xSemaphoreTakeRecursive(rt->lock, 0) == pdTRUE;
    }
    return ok;
}

bool route_table_unlock(route_table_t *rt)
{
    bool ok = false;

    if (rt != NULL) {
        ok = xSemaphoreGiveRecursive(rt->lock) == pdTRUE;
    }
    return ok;
}
    
route_table_t* route_table_init(route_table_t* rt)
{
    /* Do not recreate a table if it exists */
    if (rt != NULL && rt->lock == NULL) {

        rt->lock = xSemaphoreCreateRecursiveMutex();
        rt->routes = NULL;
        rt->num_routes = 0;
    }

    return rt;
}

bool route_table_deinit(route_table_t* rt)
{
    if (rt != NULL) {
        if (rt->lock != NULL) {

            route_table_lock(rt);

	        /* Remove all routes */
            while (rt->routes != NULL) {
                route_delete(rt->routes);
            }
	    }
	    route_table_unlock(rt);

        vSemaphoreDelete(rt->lock);
        rt->lock = NULL;
    }

    return rt->lock == NULL;
}
    
route_t* route_create(route_table_t* rt, int target, int nexthop, int sequence, int metric, uint8_t flags)
{
    route_t* r = NULL;

    if (rt != NULL) {
        if (rt->num_routes < MAX_NUM_ROUTES) {

            r = (route_t*) malloc(sizeof(route_t));
            if (r != NULL) {
                r->target = target;
            }

	        r->sequence = sequence;
	        r->metric = metric;
	        r->nexthop = nexthop;
	        r->flags = flags;

	        route_update_lifetime(r, ROUTE_LIFETIME);

	        r->pending_request = NULL;
	        r->pending_timer = NULL;
	        r->pending_retries = 0;
	        r->pending_packets = xQueueCreate((UBaseType_t) ROUTE_MAX_QUEUED_PACKETS, (UBaseType_t) sizeof(packet_t*));

	        /* Add to end of route list */
	        if (route_table_lock(rt)) {

	            /* Increate the number of routes allocated */
	            rt->num_routes ++;

                if (routes != NULL) {
                    /* New route next is beginning of list */
                    r->next = rt->routes;
	                /* New route previous is end of list */
	                r->prev = rt->routes->prev;
	                /* Last in list points to new route */
	                rt->routes->prev->next = r;
	                /* First in list previous points to new end of list */
	                rt->routes->prev = r;
	            } else {
                    /* First entry */
                    r->next = r;
	                r->prev = r;
	                rt->routes = r;
	            }
	            route_table_unlock(rt);
            }
	    }
    }

    return r;
}

bool route_delete(route_t* r)
{
    bool ok = false;

    if (r != NULL) {
        route_table_t* rt = r->table;

        if (rt != NULL && route_table_lock(rt)) {

	        /* Remove route from queue */
	        r->prev->next = r->next;
	        r->next->prev = r->prev;

	        /* If last of routes, empty the list */
	        if (--(rt->num_routes) == 0) {
	            rt->routes = NULL;
	            /* else if we deleted the head, advance head pointer */
	        } else if (routes == r) {
	            rt->routes = r->next;
    	    }

    	    free((void*) r);

	        route_table_unlock(rt);

            ok = true;
        }
    }

    return ok;
}

void route_update_lifetime(route_t* r, int lifetime)
{
    if (r != NULL) {
        route_table_t* rt = r->table; 
    	route_table_lock(rt);
        r->lifetime = time(NULL) + lifetime;
	    route_table_unlock(rt);
    }
}

/*
 * Find and update the route indicated by the address.
 *
 * If a new route is created or the sequence number is different or the metric improves,
 * return the route_t* otherwise return NULL
 */
route_t* route_update(route_table_t* rt, int target, int nexthop, int sequence, int metric, uint8_t flags)
{
    route_t* r = NULL;

    if (route_table_lock(rt->lock)) {

        r = route_find(rt, target);
        if (r == NULL) {
            /* No route, so create a new one and return it */
            r = route_create(rt, target, sequence, metric, nexthop, flags);
        } else if (r != NULL && r->sequence == sequence && r->metric <= metric) {
            /* Ignore it - it's no better */
            r = NULL;
        }

        route_table_unlock(rt);
    }

    return r;
}

bool route_put_pending_packet(route_t* r, packet_t* p)
{
    bool ok = false;

    if (r != NULL && p != NULL) {
        route_table_t* rt = r->table;

	    if (route_table_lock(rt)) {
            /* Add to packet queue */
	        ok = xQueueSend(r->pending_packets, (void*) &p, 0) == pdPASS;
	        route_table_unlock(rt);
        }
    }

    return ok;
}

void route_request_retry(TimerHandle_t timer_id)
{
    route_t* r = (route_t*) pvTimerGetTimerID(timer_id);
    if (r != NULL) {
        route_table_t* rt = r->table;

        route_table_lock(rt);

	    if (r->pending_request != NULL && --(r->pending_retries) > 0) {
            send_packet(packet_ref(r->pending_request));
	    } else {
            r = NULL;
	    }

	    route_table_unlock(rt);
    }

    if (r == NULL) {
	 /* Cancel timer */
	 xTimerStop(timer_id, pdMS_TO_TICKS(ROUTE_REQUEST_TIMEOUT));
    }
}

void route_set_pending_request(route_t* r, packet_t* request, int retries, int timeout)
{
    if (r != NULL) {
        route_table_t* rt = r->table;

        if (route_table_lock(rt)) {

            r->pending_request = packet_ref(request);
            r->pending_retries = retries;
            r->pending_timer = xTimerCreate("route_timer", pdMS_TO_TICKS(ROUTE_REQUEST_TIMEOUT), pdTRUE, (void*) r, route_request_retry);

            route_table_unlock(rt);
        }
    }
}

void route_release_packets(route_t* r)
{
    if (r != NULL) {
        route_table_t* rt = r->table;
        route_table_lock(rt);

        if (r->pending_request != NULL) {
            r->pending_request = NULL;

	        if (r->pending_timer != NULL) {
                xTimerStop(r->pending_timer, pdMS_TO_TICKS(ROUTE_REQUEST_TIMEOUT));
	            r->pending_timer = NULL;
	            r->pending_retries = 0;
	        }

	        /* Move all packets to the send queue */
	        packet_t* p;
	        while (xQueueReceive(r->pending_packets, &p, 0) == pdPASS) {
	            send_packet(p);
	        }
        }
        route_table_unlock(rt);
    }
}

bool route_is_expired(route_t* r)
{
    bool expired = false;

    if (r != NULL) {
        if (route_table_lock(r->table)) {
            bool expired = time(NULL) > r->lifetime;
            route_table_unlock(r->table);
        }
    }

    return expired;
}

route_t* route_find(route_table_t* rt, int address)
{
    route_t* found = NULL;

    if (rt != NULL && route_table_lock(rt)) {
        route_t* start = rt->routes;
        route_t* route = start; 

        do {
            if (route->target == address) {
                found = route;
            }
            route = route->next;
        } while (!found && route != start);

        route_table_unlock(rt);
    }

    return found;
}

