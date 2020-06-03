/*
 * simpletimer.h
 *
 * A simple timer mechanism.
 */
#ifndef __simpletimer_h_included
#define __simpletimer_h_included

#include "os_specific.h"

typedef struct simpletimer {
    enum {
        ST_STOPPED = 0,
        ST_RUNNING,
        ST_EXPIRED,
        ST_TIMING,
    } state;

    uint32_t interval;
    uint64_t target;
} simpletimer_t;

/*
 * Forward declarations.
 */
static inline void simpletimer_set_interval(simpletimer_t *timer, uint32_t interval);
static inline bool simpletimer_is_stopped(simpletimer_t *timer);
static inline bool simpletimer_is_running(simpletimer_t *timer);
static inline void simpletimer_start(simpletimer_t *timer, uint32_t interval);
static inline void simpletimer_extend(simpletimer_t *timer, uint32_t interval);
static inline void simpletimer_set_expired(simpletimer_t *timer);
static inline void simpletimer_stop(simpletimer_t *timer);
static inline bool simpletimer_is_expired(simpletimer_t *timer);
static inline uint32_t simpletimer_elapsed(simpletimer_t *timer);
static inline uint32_t simpletimer_remaining(simpletimer_t *timer);
static inline bool simpletimer_is_expired_or_remaining(simpletimer_t *timer, uint32_t *remaining);
static inline void simpletimer_restart(simpletimer_t *timer);
static inline void simpletimer_wait_for(simpletimer_t *timer);

static inline void simpletimer_set_interval(simpletimer_t *timer, uint32_t interval)
{
    timer->interval = interval;
}

static inline bool simpletimer_is_stopped(simpletimer_t *timer)
{
    return timer->state == ST_STOPPED;
}

static inline bool simpletimer_is_running(simpletimer_t *timer)
{
    return !simpletimer_is_stopped(timer) && !simpletimer_is_expired(timer);
}

static inline void simpletimer_start(simpletimer_t *timer, uint32_t interval)
{
    simpletimer_set_interval(timer, interval);
    timer->target = get_milliseconds() + timer->interval;
    timer->state = ST_RUNNING;
}

/*
 * Make sure the timer is set for at least <interval>.  Don't decrease if smaller.
 */
static inline void simpletimer_extend(simpletimer_t *timer, uint32_t interval)
{
    if (!simpletimer_is_running(timer) || simpletimer_remaining(timer) < interval) {
        /* Increase to new interval */
        simpletimer_start(timer, interval);
    }
}

static inline void simpletimer_set_expired(simpletimer_t *timer)
{
    timer->state = ST_EXPIRED;
}

static inline void simpletimer_stop(simpletimer_t *timer)
{
    timer->state = ST_STOPPED;
}

static inline bool simpletimer_is_expired(simpletimer_t *timer)
{
    if (timer->state == ST_RUNNING && get_milliseconds() >= timer->target) {
        timer->state = ST_EXPIRED;
    }

    return timer->state == ST_EXPIRED;
}

static inline uint32_t simpletimer_remaining(simpletimer_t *timer)
{
    uint32_t remaining;

    if (simpletimer_is_running(timer)) {
        remaining = timer->target - get_milliseconds();
    } else if (simpletimer_is_stopped(timer)) {
        remaining = 0xFFFFFFFF;
    } else {
        remaining = 0;
    }
    return remaining;
}

static inline uint32_t simpletimer_elapsed(simpletimer_t *timer)
{
    return simpletimer_is_running(timer) ? get_milliseconds() - (timer->target - timer->interval) : 0;
}


static inline bool simpletimer_is_expired_or_remaining(simpletimer_t *timer, uint32_t *remaining)
{
    bool expired = simpletimer_is_expired(timer);

    /* If it didn't fire, calculate next time to fire */
    if (!expired) {
        uint32_t time_to_fire = simpletimer_remaining(timer);
        if (time_to_fire < *remaining) {
            *remaining = time_to_fire;
        }
    }

    return expired;
}

static inline void simpletimer_restart(simpletimer_t *timer)
{
    timer->target = get_milliseconds() + timer->interval;
    timer->state = ST_RUNNING;
}

/*
 * Wait until timer expires
 */
static inline void simpletimer_wait_for(simpletimer_t *timer)
{
    uint32_t remaining = simpletimer_remaining(timer);
    if (remaining != 0) {
        os_delay(remaining);
    }
}


#endif  /* __simpletimer_h_include */

