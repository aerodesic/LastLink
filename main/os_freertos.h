/*
 * os_freertos.h
 *
 * Provides an overlay of functions of the FreeRTOS using a common
 * naming condition and return codes.
 */
#ifndef __os_freertos_h_included
#define __os_freertos_h_included

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

typedef SemaphoreHandle_t os_mutex_t;
typedef QueueHandle_t     os_queue_t;
typedef void*             os_queue_item_t;
typedef TimerHandle_t     os_timer_t;
typedef TaskHandle_t      os_thread_t;

os_mutex_t os_create_mutex(void);
os_mutex_t os_create_recursive_mutex(void);
bool os_delete_mutex(os_mutex_t mutex);
bool os_acquire_mutex_with_timeout(os_mutex_t mutex, int timeout);
bool os_acquire_mutex(os_mutex_t mutex);
bool os_acquire_mutex_recursive_with_timeout(os_mutex_t mutex, int timeout);
bool os_acquire_mutex_recursive(os_mutex_t mutex);
bool os_acquire_mutex_from_isr(os_mutex_t mutex);
bool os_release_mutex(os_mutex_t mutex);
bool os_release_mutex_from_isr(os_mutex_t mutex);

os_queue_t os_create_queue(int depth, size_t size);
bool os_delete_queue(os_queue_t queue);
bool os_put_queue_with_timeout(os_queue_t queue, void* item, int timeout);
bool os_put_queue(os_queue_t queue, os_queue_item_t item);
bool os_put_queue_from_isr(os_queue_t queue, os_queue_item_t item);
bool os_get_queue_with_timeout(os_queue_t queue, os_queue_item_t* item, int timeout);
bool os_get_queue(os_queue_t queue, os_queue_item_t* item);
bool os_get_queue_from_isr(os_queue_t queue, os_queue_item_t* item);

os_timer_t os_create_timer(const char* name, int timeout, bool reload, void* param, void (*function)(void* param));
bool os_start_timer(os_timer_t timer);
bool os_stop_timer(os_timer_t timer);
bool  os_reset_timer(os_timer_t timer);
void* os_get_timer_data(os_timer_t timer);
bool os_delete_timer(os_timer_t timer);

os_thread_t os_create_thread(void (*process)(void* param), const char* name, size_t stack_size, int priority, void* param);
bool os_delete_thread(os_thread_t thread);

uint64_t get_milliseconds(void);

#endif /* __os_freertos_h_included */

