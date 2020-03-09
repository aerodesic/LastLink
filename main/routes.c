/*
 * routes.c
 *
 * Handle routes
 */

#include "esp_system.h"
#include "esp_log.h"

#include "os_freertos.h"

#include "linklayer.h"
#include "packets.h"
#include "routes.h"

#define TAG  "routes"

//static route_t*           routes;

bool route_table_lock(route_table_t* rt)
{
    bool ok = false;

    if (rt != NULL) {
        ok = os_acquire_mutex(rt->lock);
    }
    return ok;
}

bool route_table_unlock(route_table_t *rt)
{
    bool ok = false;

    if (rt != NULL) {
        ok = os_release_mutex(rt->lock); 
    }
    return ok;
}
    
route_table_t* route_table_init(route_table_t* rt)
{
    /* Do not recreate a table if it exists */
    if (rt != NULL && rt->lock == NULL) {

        rt->lock = os_create_recursive_mutex();
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

        os_delete_mutex(rt->lock);
        rt->lock = NULL;
    }

    return rt->lock == NULL;
}
    
route_t* route_create(route_table_t* rt, int target, int radio_num, int nexthop, int sequence, int metric, uint8_t flags)
{
    route_t* r = NULL;

    if (rt != NULL) {
        if (rt->num_routes < MAX_NUM_ROUTES) {

            r = (route_t*) malloc(sizeof(route_t));
            if (r != NULL) {
                r->target = target;
	            r->sequence = sequence;
	            r->metric = metric;
	            r->nexthop = nexthop;
	            r->flags = flags;

	            route_update_lifetime(r, ROUTE_LIFETIME);

                r->radio_num = radio_num;
	            r->pending_request = NULL;
	            r->pending_timer = NULL;
	            r->pending_retries = 0;
	            r->pending_packets = os_create_queue(ROUTE_MAX_QUEUED_PACKETS, sizeof(packet_t*));

	            /* Add to end of route list */
	            if (route_table_lock(rt)) {

	                /* Increate the number of routes allocated */
	                rt->num_routes ++;

                    if (rt->routes != NULL) {
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
	        } else if (rt->routes == r) {
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
        r->lifetime = get_milliseconds() + lifetime;
	    route_table_unlock(rt);
    }
}

/*
 * Find and update the route indicated by the address.
 *
 * If a new route is created or the sequence number is different or the metric improves,
 * return the route_t* otherwise return NULL
 */
route_t* route_update(route_table_t* rt, int target, int radio_num, int nexthop, int sequence, int metric, uint8_t flags)
{
    route_t* r = NULL;

    if (route_table_lock(rt->lock)) {

        r = route_find(rt, target);
        if (r == NULL) {
            /* No route, so create a new one and return it */
            r = route_create(rt, target, radio_num, sequence, metric, nexthop, flags);
        } else if (r != NULL && r->sequence == sequence && r->metric <= metric) {
            /* Ignore it - it's no better */
            r = NULL;
        } else {
            r->sequence = sequence;
            r->metric = metric;
            r->flags = flags;
            r->nexthop = nexthop;
            r->radio_num = radio_num;
            r->target = target;
            route_update_lifetime(r, ROUTE_LIFETIME);
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
	        ok = os_put_queue(r->pending_packets, p);
	        route_table_unlock(rt);
        }
    }

    return ok;
}

void route_request_retry(os_timer_t timer)
{
    route_t* r = (route_t*) os_get_timer_data(timer);

    if (r != NULL) {
        route_table_t* rt = r->table;

        route_table_lock(rt);

	    if (r->pending_request != NULL && --(r->pending_retries) > 0) {
            linklayer_send_packet(ref_packet(r->pending_request));
	    } else {
            r = NULL;
	    }

	    route_table_unlock(rt);
    }

    if (r == NULL) {
	    /* Cancel timer */
        os_stop_timer(r->pending_timer);
        os_delete_timer(r->pending_timer);
        r->pending_timer = NULL;
    }
}

void route_set_pending_request(route_t* r, packet_t* request, int retries, int timeout)
{
    if (r != NULL) {
        route_table_t* rt = r->table;

        if (route_table_lock(rt)) {

            r->pending_request = ref_packet(request);
            r->pending_retries = retries;
            r->pending_timer = os_create_timer("route_timer", ROUTE_REQUEST_TIMEOUT, true, (void*) r, route_request_retry);
            os_start_timer(r->pending_timer);

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
                os_stop_timer(r->pending_timer);
                os_delete_timer(r->pending_timer);
	            r->pending_timer = NULL;
	            r->pending_retries = 0;
	        }

	        /* Move all packets to the send queue */
	        packet_t* p;
	        while (os_get_queue_with_timeout(r->pending_packets, (os_queue_item_t*) &p, 0)) {

                if (r->radio_num == ALL_RADIOS) {
                    ESP_LOGE(TAG, "send on route without a radio: target %d nexthop %d sequence %d", r->target, r->nexthop, r->sequence);
                }

                /* Assign the delivery route */
                set_uint_field(p, HEADER_NEXTHOP_ADDRESS, ADDRESS_LEN, r->nexthop);
                /* Direct it to specific radio */
                p->radio_num = r->radio_num;
	            linklayer_send_packet(p);
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
            expired = get_milliseconds() > r->lifetime;
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

