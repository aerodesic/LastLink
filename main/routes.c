/*
 * routes.c
 *
 * Handle routes
 */

#include "esp_system.h"
#include "esp_log.h"

#include "os_specific.h"

#include "linklayer.h"
#include "packets.h"
#include "routes.h"
#include "listops.h"
#include "simpletimer.h"

#define ROUTE_SCANNER_STACK_SIZE  8192

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
#include "commands.h"
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

#define TAG  "routes"

static list_head_t         routes;
static os_mutex_t          routes_lock;
static os_thread_t         route_scanner_thread_id;

bool route_table_lock(void)
{
    return os_acquire_recursive_mutex(routes_lock);
}

bool route_table_unlock(void)
{
    return os_release_recursive_mutex(routes_lock);
}

/*
 * Called by timer
 */
static bool route_request_retry(route_t *route)
{
    bool deleted = false;

	if (simpletimer_is_expired(&route->pending_timer)) {
        if (route->pending_retries != 0) {
            route->pending_retries--;

            simpletimer_restart(&route->pending_timer);

            /* Create a route request and send it */
            packet_t* packet = routerequest_packet_create(route->dest);

	        linklayer_route_packet(packet);

	    } else {
            /* No joy, delete the route; will rebuild if necessary */
            route_delete(route);
            deleted = true;
	    }
    }

    return deleted;
}

/*
 * Called by thread
 */
static bool check_route_expired(route_t *route)
{
    bool deleted = false;

    if (simpletimer_is_expired(&route->lifetime)) {
//ESP_LOGI(TAG, "%s: expiring route for address %d", __func__, route->dest);
        route_delete(route);
        deleted = true;
    }

    return deleted;
}

static void route_scanner_thread(void* param)
{
    while (true) {
        if (route_table_lock()) {
            route_t *route = (route_t *) FIRST_LIST_ITEM(&routes);

            while (route != NULL) {
                 if (route_request_retry(route) || check_route_expired(route)) {
                     /* End the search if we have deleted something */
                     route = NULL;
                 }
                 route = NEXT_LIST_ITEM(route, &routes);
            }

            route_table_unlock();
        }

        /* one second wait */
        os_delay(1000);
    }
}

route_t* create_route(int radio_num, int dest, int metric, int routeto, int origin, int sequence, int flags)
{
    route_t* r = NULL;

    if (NUM_IN_LIST(&routes) < MAX_NUM_ROUTES) {

        r = (route_t*) malloc(sizeof(route_t));
        if (r != NULL) {
            r->radio_num = radio_num;
            r->dest      = dest;
	        r->metric    = metric;
	        r->routeto   = routeto;
            r->origin    = origin;
	        r->sequence  = sequence;
	        r->flags     = flags;

            simpletimer_start(&r->lifetime, ROUTE_LIFETIME);
            simpletimer_stop(&r->pending_timer);

	        r->pending_retries = 0;
	        r->pending_packets = NULL;  /* No queue yet.  created when needed */

	        /* Add to end of route list */
	        if (route_table_lock()) {

                /* Append to list */
                ADD_TO_LIST(&routes, r);
	            route_table_unlock();
            }
	    }
    }

    return r;
}

/*
 * Remove a route from the table by address.  Deletes all pending stuff on the route.
 */
bool route_remove(int address)
{
    bool ok = false;

    route_table_lock();
    route_t *route = find_route(address);
    if (route != NULL) {
        ok = route_delete(route);
    }
    route_table_unlock();

    return ok;
}

/*
 * Remove route from table, delete it's contents and free it.
 */
bool route_delete(route_t* r)
{
    bool ok = false;

    if (r != NULL) {
        REMOVE_FROM_LIST(&routes, r);
	    /* Cancel timer */
        simpletimer_stop(&r->pending_timer);

        if (r->pending_packets != NULL) {
            /* Discard the packets waiting */
	        packet_t* p;
	        while (os_get_queue_with_timeout(r->pending_packets, (os_queue_item_t) &p, 0)) {
                linklayer_print_packet("PENDING PACKETS", p);

                /* Tell supplier if it needs to know when route is complete */
                packet_tell_routed_callback(p, false);

                release_packet(p);
            }
            os_delete_queue(r->pending_packets);
            r->pending_packets = NULL;
        }

    	free((void*) r);

	    route_table_unlock();

        ok = true;
    }

    return ok;
}

/*
 * Find and update the route indicated by the address.
 *
 * If a new route is created or the origin/sequence number is different or the metric improves,
 * return the route_t* otherwise return NULL
 */
route_t* update_route(int radio_num, int dest, int metric, int routeto, int origin, int sequence, int flags)
{
    route_t* r = NULL;

    if (route_table_lock()) {

        r = find_route(dest);
        if (r == NULL) {
            /* No route, so create a new one and return it */
            r = create_route(radio_num, dest, metric, routeto, origin, sequence, flags);
        } else if (r != NULL && r->origin == origin && r->sequence == sequence && r->metric <= metric) {
            /* Ignore it - it's no better */
            r = NULL;
        } else {
            r->origin     = origin;
            r->sequence   = sequence;
            r->metric     = metric;
            r->routeto    = routeto;
            r->radio_num  = radio_num;
            r->dest       = dest;
            r->flags      = flags;
            simpletimer_restart(&r->lifetime);
        }

        route_table_unlock();
    }

    return r;
}

bool route_put_pending_packet(route_t* r, packet_t* p)
{
    bool ok = false;

    if (r != NULL && p != NULL) {
	    if (route_table_lock()) {
            /* Add to packet queue */
            if (r->pending_packets == NULL) {
	            r->pending_packets = os_create_queue(ROUTE_MAX_QUEUED_PACKETS, sizeof(packet_t*));
            }
            if (r->pending_packets == NULL) {
                ESP_LOGE(TAG, "%s: Unable to create pending packet queue", __func__);
            } else {
	            ok = os_put_queue(r->pending_packets, (os_queue_item_t) &p);
            }
	        route_table_unlock();
        }
    }

    return ok;
}

void route_start_routerequest(route_t* r)
{
    if (r != NULL) {
        if (route_table_lock()) {

            r->pending_retries = ROUTE_REQUEST_RETRIES;
            simpletimer_start(&r->pending_timer, ROUTE_REQUEST_TIMEOUT);

            /* Create a route request and send it */
            packet_t* packet = routerequest_packet_create(r->dest);
	        linklayer_route_packet(packet);

            route_table_unlock();
        }
    }
}

void route_release_packets(route_t* r)
{
    if (r != NULL) {
        route_table_lock();

        simpletimer_stop(&r->pending_timer);

        if (r->pending_packets != NULL) {
	        /* Move all packets to the send queue */
	        packet_t* p;
	        while (os_get_queue_with_timeout(r->pending_packets, (os_queue_item_t) &p, 0)) {
                if (r->radio_num == UNKNOWN_RADIO) {
                    ESP_LOGE(TAG, "send on route without a radio: dest %d routeto %d origin %d sequence %d", r->dest, r->routeto, r->origin, r->sequence);
                }

                /* Assign the delivery route */
                set_uint_field(p, HEADER_ROUTETO_ADDRESS, ADDRESS_LEN, r->routeto);

                /* Direct it to specific radio */
                p->radio_num = r->radio_num;
                if (packet_tell_routed_callback(p, true)) {
	                linklayer_route_packet(p);
                } else {
                    /* Cancel the packet */
                    release_packet(p);
                }
            }

            os_delete_queue(r->pending_packets);
            r->pending_packets = NULL;
	    }
        route_table_unlock();
    }
}

route_t* find_route(int address)
{
    route_t* found = NULL;

    if (route_table_lock()) {
        route_t* route = (route_t*) FIRST_LIST_ITEM(&routes);

        while (found == NULL && route != NULL) {
            if (route->dest == address) {
//ESP_LOGI(TAG, "%s: found %d at %d", __func__, address, route->dest);
                found = route;

                /* Update the expire time */
                simpletimer_restart(&route->lifetime);
            }

            route = NEXT_LIST_ITEM(route, &routes);
        }

        route_table_unlock();
    }

    return found;
}

/*
 * Return the metric (hops) to the target destination.
 * For no route, assume the worst.
 */
int route_metric(int address)
{
    int metric = MAX_METRIC;

    route_table_lock();
    route_t *route = find_route(address);

    if (route != NULL) {
        metric = route->metric;
    }

    route_table_unlock();

    return metric;
}

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
/*
 * For delivering the contents for display.
 */
typedef struct {
    int                 dest;
    int                 radio_num;
    int                 origin;
    int                 sequence;
    int                 metric;
    int                 routeto;
    uint8_t             flags;
    int                 lifetime;
    int                 pending_packets;
    int                 pending_timer;
    int                 pending_retries;
} route_table_values_t;

static int print_route_table(int argc, const char **argv)
{
    if (argc == 0) {
        show_help(argv[0], "", "Print route table");
    } else {
        route_table_values_t table[CONFIG_LASTLINK_MAX_ROUTES];

        route_table_lock();

        /* Capture the table contents so we don't slow things down when we print */
        route_t* route = (route_t*) FIRST_LIST_ITEM(&routes);
        int rt_used = 0;

        while (route != NULL) {
            table[rt_used].dest = route->dest;
            table[rt_used].radio_num = route->radio_num;
            table[rt_used].origin = route->origin;
            table[rt_used].sequence = route->sequence;
            table[rt_used].metric = route->metric;
            table[rt_used].routeto = route->routeto;
            table[rt_used].flags = route->flags;
            table[rt_used].lifetime = simpletimer_remaining(&route->lifetime),
            table[rt_used].pending_packets = route->pending_packets ? os_items_in_queue(route->pending_packets) : 0;
            table[rt_used].pending_timer = simpletimer_remaining(&route->pending_timer);
            table[rt_used].pending_retries = route->pending_retries;

            ++rt_used;

            route = NEXT_LIST_ITEM(route, &routes);
        }

        route_table_unlock();

        if (rt_used != 0) {
            printf("Dest  Radio  Source  Sequence  Metric  RouteTo  Flags  Life  Pending  Timer  Retries\n");

            for (int index = 0; index < rt_used; ++index) {
                printf("%-4d  %-5d  %-6d  %-8d  %-6d  %-7d  %02x     %-4d  %-7d  %-5d  %-d\n",
                        table[index].dest,
                        table[index].radio_num,
                        table[index].origin,
                        table[index].sequence,
                        table[index].metric,
                        table[index].routeto,
                        table[index].flags,
                        table[index].lifetime / 1000,
                        table[index].pending_packets,
                        table[index].pending_timer / 1000,
                        table[index].pending_retries);
            }
        }
    }

    return 0;
}
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

void route_table_init(void)
{
    /* Do not recreate a table if it exists */
    if (routes_lock == NULL) {
        routes_lock = os_create_recursive_mutex();
        INIT_LIST(&routes);
    }

    /* Start a thread to expire routes when they get too old */
    if (route_scanner_thread_id == NULL) {
        route_scanner_thread_id = os_create_thread(route_scanner_thread, "route_scanner", ROUTE_SCANNER_STACK_SIZE, 0, NULL);
    }

#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    add_command("routes", print_route_table);
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */
}

bool route_table_deinit(void)
{
#if CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS
    remove_command("routes");
#endif /* CONFIG_LASTLINK_EXTRA_DEBUG_COMMANDS */

    if (routes_lock != NULL) {

        route_table_lock();

        if (route_scanner_thread_id != NULL) {
            os_delete_thread(route_scanner_thread_id);
            route_scanner_thread_id = NULL;
        }

	    /* Remove all routes */
        while (NUM_IN_LIST(&routes) != 0) {
            route_delete((route_t*) FIRST_LIST_ITEM(&routes));
	    }

	    route_table_unlock();

        os_delete_mutex(routes_lock);
        routes_lock = NULL;
    }

    return routes_lock == NULL;
}

