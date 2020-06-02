/*
 * os_freertos.h
 *
 * Provides an overlay of functions of the FreeRTOS using a common
 * naming condition and return codes.
 */
#ifndef __os_freertos_h_included
#define __os_freertos_h_included

#define ELEMENTS_OF(x) (sizeof(x) / sizeof((x)[0]))

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

typedef SemaphoreHandle_t os_mutex_t;
typedef SemaphoreHandle_t os_semaphore_t;
typedef QueueHandle_t     os_queue_t;
typedef void*             os_queue_item_t;
typedef TimerHandle_t     os_timer_t;
typedef TaskHandle_t      os_thread_t;

/* Init */
void os_init(void);

/* Thread-specific stuff */
os_semaphore_t* os_thread_sem_get(void);
os_semaphore_t* os_thread_sem_init(void);
void os_thread_sem_deinit(void);

/* Mutexes */
os_mutex_t os_create_mutex(void);
os_mutex_t os_create_recursive_mutex(void);
bool os_delete_mutex(os_mutex_t mutex);
bool os_acquire_mutex(os_mutex_t mutex);
bool os_acquire_recursive_mutex(os_mutex_t mutex);
bool os_acquire_mutex_with_timeout(os_mutex_t mutex, int timeout);
bool os_acquire_recursive_mutex_with_timeout(os_mutex_t mutex, int timeout);
bool os_acquire_mutex_from_isr(os_mutex_t mutex, bool* awakened);
bool os_release_mutex(os_mutex_t mutex);
bool os_release_recursive_mutex(os_mutex_t mutex);
bool os_release_mutex_from_isr(os_mutex_t mutex, bool* awakened);
int os_get_mutex_count(os_mutex_t mutex);

/* Semaphores */
os_semaphore_t os_create_counting_semaphore(int max_count, int initial_count);
os_semaphore_t os_create_binary_semaphore(void);
bool os_acquire_semaphore(os_semaphore_t sem);
bool os_release_semaphore(os_semaphore_t sem);
bool os_release_semaphore_from_isr(os_semaphore_t sem, bool *awakened);
bool os_release_counting_semaphore(os_semaphore_t sem, int count);
bool os_acquire_counting_semaphore(os_semaphore_t sem);
bool os_acquire_counting_semaphore_with_timeout(os_semaphore_t sem, int timeout);
bool os_delete_semaphore(os_semaphore_t sem);

/* Queues */
os_queue_t os_create_queue(int depth, size_t size);
bool os_delete_queue(os_queue_t queue);
bool os_put_queue_with_timeout(os_queue_t queue, void* item, int timeout);
bool os_put_queue(os_queue_t queue, os_queue_item_t item);
bool os_put_queue_from_isr(os_queue_t queue, os_queue_item_t item, bool* awakened);
bool os_get_queue_with_timeout(os_queue_t queue, os_queue_item_t* item, int timeout);
bool os_get_queue(os_queue_t queue, os_queue_item_t* item);
bool os_get_queue_from_isr(os_queue_t queue, os_queue_item_t* item, bool* awakened);
bool os_peek_queue(os_queue_t queue, os_queue_item_t* item);
bool os_peek_queue_from_isr(os_queue_t queue, os_queue_item_t* item);
int os_items_in_queue_from_isr(os_queue_t queue);
int os_items_in_queue(os_queue_t queue);

/* Timers */
os_timer_t os_create_timer(const char* name, int period, void* param, void (*function)(TimerHandle_t xTimer));
os_timer_t os_create_repeating_timer(const char* name, int period, void* param, void (*function)(TimerHandle_t xTimer));
bool os_start_timer(os_timer_t timer);
bool os_stop_timer(os_timer_t timer);
bool  os_reset_timer(os_timer_t timer);
bool os_set_timer(os_timer_t timer, int value);
void* os_get_timer_data(os_timer_t timer);
bool os_delete_timer(os_timer_t timer);

os_thread_t os_create_thread(void (*process)(void* param), const char* name, size_t stack_size, int priority, void* param);
os_thread_t os_create_thread_on_core(void (*process)(void* param), const char* name, size_t stack_size, int priority, void* param, int core);
bool os_delete_thread(os_thread_t thread);
bool os_exit_thread(void);

/* Allocate DMA compatible memory */
void* os_alloc_dma_memory(size_t size);

bool os_delay(int ms);

bool os_attach_gpio_interrupt(int gpio, GPIO_INT_TYPE edge, gpio_pullup_t pullup, gpio_pulldown_t pulldown, void (*handler)(void*), void* param);

uint64_t get_milliseconds(void);

#endif /* __os_freertos_h_included */

