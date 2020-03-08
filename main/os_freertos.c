/*
 * os_freertos.c
 *
 * Provides an overlay of functions of the FreeRTOS using a common
 * naming condition and return codes.
 */
#include "os_freertos.h"

os_mutex_t os_create_mutex(void)
{
    return xSemaphoreCreateMutex();
}

os_mutex_t os_create_recursive_mutex(void)
{
    return xSemaphoreCreateRecursiveMutex();
}

bool os_delete_mutex(os_mutex_t mutex)
{
    vSemaphoreDelete(mutex);
    return true;
}

bool os_acquire_mutex_with_timeout(os_mutex_t mutex, int timeout)
{
    return xSemaphoreTake(mutex, timeout) == pdTRUE;
}

bool os_acquire_mutex(os_mutex_t mutex)
{
    return os_acquire_mutex_with_timeout(mutex, portMAX_DELAY);
}

bool os_acquire_recursive_mutex_with_timeout(os_mutex_t mutex, int timeout)
{
    return xSemaphoreTakeRecursive(mutex, timeout) == pdTRUE;
}

bool os_acquire_recursive_mutex(os_mutex_t mutex)
{
    return os_acquire_recursive_mutex_with_timeout(mutex, portMAX_DELAY);
}

bool os_acquire_mutex_from_isr(os_mutex_t mutex)
{
    return xSemaphoreTakeFromISR(mutex, NULL) == pdTRUE;
}

bool os_release_mutex(os_mutex_t mutex)
{
    xSemaphoreGive(mutex);
    return true;
}

bool os_release_recursive_mutex(os_mutex_t mutex)
{
    xSemaphoreGiveRecursive(mutex);
    return true;
}

bool os_release_mutex_from_isr(os_mutex_t mutex)
{
    xSemaphoreGiveFromISR(mutex, NULL);
    return true;
}

os_queue_t os_create_queue(int depth, size_t size)
{
    return xQueueCreate((UBaseType_t) depth, (UBaseType_t) size);
}

bool os_delete_queue(os_queue_t queue)
{
    vQueueDelete(queue);
    return true;
}

bool os_put_queue_with_timeout(os_queue_t queue, os_queue_item_t item, int timeout)
{
    return xQueueSend(queue, &item, timeout) == pdTRUE;
}

bool os_get_queue_from_isr(os_queue_t queue, os_queue_item_t* item)
{
    return xQueueSendFromISR(queue, item, NULL) == pdTRUE;
}

bool os_put_queue(os_queue_t queue, os_queue_item_t item)
{
    return os_put_queue_with_timeout(queue, item, portMAX_DELAY);
}

bool os_put_queue_from_isr(os_queue_t queue, os_queue_item_t item)
{
    return xQueueSendFromISR(queue, &item, NULL) == pdTRUE;
}

bool os_get_queue_with_timeout(os_queue_t queue, os_queue_item_t* item, int timeout)
{
    return (xQueueReceive(queue, item, timeout) == pdTRUE);
}

bool os_get_queue(os_queue_t queue, os_queue_item_t* item)
{
    return os_get_queue_with_timeout(queue, item, portMAX_DELAY);
}

int os_items_in_queue(os_queue_t queue)
{
    return uxQueueMessagesWaiting(queue);
}

int os_items_in_queue_from_isr(os_queue_t queue)
{
    return uxQueueMessagesWaitingFromISR(queue);
}

os_timer_t os_create_timer(const char* name, int timeout, bool reload, void* param, void (*function)(void* param))
{
    return xTimerCreate(name, pdMS_TO_TICKS(timeout), reload, param, function);
}

bool os_start_timer(os_timer_t timer)
{
    return xTimerStart(timer, portMAX_DELAY);
}

void* os_get_timer_data(os_timer_t timer)
{
    return pvTimerGetTimerID(timer);
}

bool os_stop_timer(os_timer_t timer)
{
    return xTimerStop(timer, portMAX_DELAY);
}

bool os_reset_timer(os_timer_t timer)
{
    return xTimerReset(timer, portMAX_DELAY);
}

bool os_delete_timer(os_timer_t timer)
{
    return xTimerDelete(timer, portMAX_DELAY);
}


os_thread_t os_create_thread(void (*process)(void* param), const char* name, size_t stack_size, int priority, void* param)
{
    os_thread_t thread;
    if (xTaskCreate(process, name, stack_size ? stack_size : configMINIMAL_STACK_SIZE, param, priority, &thread) != pdTRUE) {
        thread = 0;
    }
    return thread;
}

bool os_delete_thread(os_thread_t thread)
{
    if (thread != NULL) {
        vTaskDelete(thread);
        return true;
    } else {
        return false;
    }
}

extern volatile unsigned long ulHighFrequencyTimerTicks;
static unsigned long last_ticks;
static uint64_t total_ticks;
static os_mutex_t ticks_mutex;

#define u64TICKS_TO_MS(ticks) ((uint64_t) (ticks) * 1000 / configTICK_RATE_HZ)

uint64_t get_milliseconds(void)
{
    uint64_t ret = 0;

    if (ticks_mutex == NULL) {
        ticks_mutex = os_create_mutex();
    }
    if (os_acquire_mutex(ticks_mutex)) {
        unsigned long current_ticks = ulHighFrequencyTimerTicks;
        total_ticks += current_ticks - last_ticks;
        ret = u64TICKS_TO_MS(total_ticks);
    }

    return ret;
}

