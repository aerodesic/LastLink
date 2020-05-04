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
        ok = os_acquire_recursive_mutex(rt->lock);
    }
    return ok;
}

bool route_table_unlock(route_table_t *rt)
{
    bool ok = false;

    if (rt != NULL) {
        ok = os_release_recursive_mutex(rt->lock);
    }
    return ok;
}


static void route_expire_thread(void* param)
{
    route_table_t* rt = (route_table_t*) param;

    while (true) {
        if (route_table_lock(rt)) {
            route_t* start = rt->routes;
            route_t* route = start;

            if (start != NULL) {
                do {
                    if (route_is_expired(route)) {
ESP_LOGI(TAG, "%s: expiring route for address %d", __func__, route->dest);
                        route_delete(route);

                        /* Stop now, only do one release per cycle */
                        route = start = NULL;
                    } else {
                        route = route->next;
                    }
                } while (route != start);
            }
            route_table_unlock(rt);
        }

        /* one second wait */
        os_delay(1000);
    }
}

route_table_t* route_table_init(route_table_t* rt)
{
    /* Do not recreate a table if it exists */
    if (rt != NULL && rt->lock == NULL) {

        rt->lock = os_create_recursive_mutex();
        rt->routes = NULL;
        rt->num_routes = 0;
    }

    /* Start a thread to expire routes when they get too old */
    if (rt->expire_thread == NULL) {
        rt->expire_thread = os_create_thread(route_expire_thread, "route_expire", 8192, 0, rt);
    }

    return rt;
}

bool route_table_deinit(route_table_t* rt)
{
    if (rt != NULL) {
        if (rt->lock != NULL) {

            route_table_lock(rt);

            if (rt->expire_thread != NULL) {
                os_delete_thread(rt->expire_thread);
                rt->expire_thread = NULL;
            }

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

route_t* route_create(route_table_t* rt, int radio_num, int dest, int routeto, int source, int sequence, int metric, uint8_t flags)
{
    route_t* r = NULL;

    if (rt != NULL) {
        if (rt->num_routes < MAX_NUM_ROUTES) {

            r = (route_t*) malloc(sizeof(route_t));
            if (r != NULL) {
                r->dest = dest;
                r->source = source;
	            r->sequence = sequence;
	            r->metric = metric;
	            r->routeto = routeto;
	            r->flags = flags;
                r->table = rt;

	            route_update_lifetime(r, ROUTE_LIFETIME);

                r->radio_num = radio_num;
	            r->pending_timer = NULL;
	            r->pending_retries = 0;
	            r->pending_packets = NULL;  /* No queue yet.  created when needed */

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

/*
 * Remove a route from the table by address.  Deletes all pending stuff on the route.
 */
bool route_remove(route_table_t* rt, int address)
{
    bool ok = false;

    route_table_lock(rt);
    route_t *route = route_find(rt, address);
    if (route != NULL) {
        ok = route_delete(route);
    }
    route_table_unlock(rt);

    return ok;
}

/*
 * Remove route from table, delete it's contents and free it.
 */
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

	        /* Cancel timer */
            if (r->pending_timer) {
                ESP_LOGI(TAG, "%s: canceling pending route request", __func__);
                os_stop_timer(r->pending_timer);
                os_delete_timer(r->pending_timer);
                r->pending_timer = NULL;
            }

            if (r->pending_packets != NULL) {
                /* Discard the packets waiting */
	            packet_t* p;
	            while (os_get_queue_with_timeout(r->pending_packets, (os_queue_item_t*) &p, 0)) {
                    linklayer_print_packet("PENDING PACKETS", p);

                    /* Tell supplier if it needs to know when route is complete */
                    packet_tell_routed_callback(p, false);

                    release_packet(p);
                }
                os_delete_queue(r->pending_packets);
                r->pending_packets = NULL;
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
 * If a new route is created or the source/sequence number is different or the metric improves,
 * return the route_t* otherwise return NULL
 */
route_t* route_update(route_table_t* rt, int radio_num, int dest, int routeto, int source, int sequence, int metric, uint8_t flags)
{
    route_t* r = NULL;

    if (route_table_lock(rt->lock)) {

        r = route_find(rt, dest);
        if (r == NULL) {
            /* No route, so create a new one and return it */
            r = route_create(rt, radio_num, dest, routeto, source, sequence, metric, flags);
ESP_LOGI(TAG, "%s: creating route D=%04x Radio %d R=%04x Source %d Sequence %d Metric %d Flags %02x", __func__, dest, radio_num, routeto, source, sequence, metric, flags);
        } else if (r != NULL && r->source == source && r->sequence == sequence && r->metric <= metric) {
            /* Ignore it - it's no better */
ESP_LOGI(TAG, "%s: ignored route D=%04x Radio %d R=%04x Source %d Sequence %d Metric %d Flags %02x", __func__, dest, radio_num, routeto, source, sequence, metric, flags);
            r = NULL;
        } else {
ESP_LOGI(TAG, "%s: updating route D=%04x Radio %d R=%04x Source %d Sequence %d Metric %d Flags %02x", __func__, dest, radio_num, routeto, source, sequence, metric, flags);
            r->source = source;
            r->sequence = sequence;
            r->metric = metric;
            r->flags = flags;
            r->routeto = routeto;
            r->radio_num = radio_num;
            r->dest = dest;
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
            if (r->pending_packets == NULL) {
	            r->pending_packets = os_create_queue(ROUTE_MAX_QUEUED_PACKETS, sizeof(packet_t*));
            }
            if (r->pending_packets == NULL) {
                ESP_LOGE(TAG, "%s: Unable to create pending packet queue", __func__);
            } else {
	            ok = os_put_queue(r->pending_packets, p);
            }
	        route_table_unlock(rt);
        }
    }

    return ok;
}

/*
 * Called by timer
 */
void route_request_retry(os_timer_t timer)
{
    ESP_LOGI(TAG, "%s: timer fired", __func__);

    route_t* r = (route_t*) os_get_timer_data(timer);

    if (r != NULL) {
        route_table_t* rt = r->table;

        if (route_table_lock(rt)) {

	        if (r->pending_retries != 0) {
                r->pending_retries--;

                /* Create a route request and send it */
                packet_t* packet = routerequest_packet_create(r->dest);

	            linklayer_send_packet(packet);

	        } else {
                /* No joy, delete the route; will rebuild if necessary */
                route_delete(r);
	        }

	        route_table_unlock(rt);
        }
    }
}

void route_start_routerequest(route_t* r)
{
    if (r != NULL) {
        route_table_t* rt = r->table;

        if (route_table_lock(rt)) {

            r->pending_retries = ROUTE_REQUEST_RETRIES;
            r->pending_timer = os_create_repeating_timer("route_timer", ROUTE_REQUEST_TIMEOUT, (void*) r, route_request_retry);
            os_start_timer(r->pending_timer);

            /* Create a route request and send it */
            packet_t* packet = routerequest_packet_create(r->dest);
	        linklayer_send_packet(packet);

            route_table_unlock(rt);
        }
    }
}

void route_release_packets(route_t* r)
{
    if (r != NULL) {
        route_table_t* rt = r->table;
        route_table_lock(rt);

	    if (r->pending_timer != NULL) {
            os_stop_timer(r->pending_timer);
            os_delete_timer(r->pending_timer);
	        r->pending_timer = NULL;
	        r->pending_retries = 0;
	    }

        if (r->pending_packets != NULL) {
	        /* Move all packets to the send queue */
	        packet_t* p;
	        while (os_get_queue_with_timeout(r->pending_packets, (os_queue_item_t*) &p, 0)) {

                if (r->radio_num == UNKNOWN_RADIO) {
                    ESP_LOGE(TAG, "send on route without a radio: dest %d routeto %d source %d sequence %d", r->dest, r->routeto, r->source, r->sequence);
                }

                /* Assign the delivery route */
                set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, r->routeto);

                /* Direct it to specific radio */
                p->radio_num = r->radio_num;
                if (packet_tell_routed_callback(p, true)) {
	                linklayer_send_packet(p);
                } else {
                    /* Cancel the packet */
                    release_packet(p);
                }
            }

            os_delete_queue(r->pending_packets);
            r->pending_packets = NULL;
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

        if (start != NULL) {
            do {
                if (route->dest == address) {
//ESP_LOGI(TAG, "%s: found %d at %d", __func__, address, route->dest);
                    found = route;

                    /* Update the expire time */
	                route_update_lifetime(route, ROUTE_LIFETIME);
                }
                route = route->next;
            } while (!found && route != start);
        }

        route_table_unlock(rt);
    }

    return found;
}

int read_route_table(route_table_t* rt, route_table_values_t *table, int rtv_len)
{
    route_table_lock(rt);

    int rt_used = 0;
    route_t* start = rt->routes;
    route_t* route = start;

    if (start != NULL) {
        do {
            table[rt_used].dest = route->dest;
            table[rt_used].radio_num = route->radio_num;
            table[rt_used].source = route->source;
            table[rt_used].sequence = route->sequence;
            table[rt_used].metric = route->metric;
            table[rt_used].routeto = route->routeto;
            table[rt_used].flags = route->flags;
            table[rt_used].lifetime = route->lifetime - get_milliseconds();
            table[rt_used].pending_packets = route->pending_packets ? os_items_in_queue(route->pending_packets) : 0;
            table[rt_used].timer = route->pending_timer != NULL;
            table[rt_used].pending_retries = route->pending_retries;
            
            route = route->next;

            ++rt_used;

        } while (rt_used < rtv_len && route != start);
    }

    route_table_unlock(rt);

    return rt_used;
}

