/*
 * os_freertos.c
 *
 * Provides an overlay of functions of the FreeRTOS using a common
 * naming condition and return codes.
 */
#include "esp_system.h"
#include "esp_log.h"

#include "os_freertos.h"

#define TAG "os_freertos"

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
    return xSemaphoreTake(mutex, pdMS_TO_TICKS(timeout)) == pdTRUE;
}

bool os_acquire_mutex(os_mutex_t mutex)
{
    return os_acquire_mutex_with_timeout(mutex, portMAX_DELAY);
}

bool os_acquire_recursive_mutex_with_timeout(os_mutex_t mutex, int timeout)
{
    return xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(timeout)) == pdTRUE;
}

bool os_acquire_recursive_mutex(os_mutex_t mutex)
{
    return os_acquire_recursive_mutex_with_timeout(mutex, portMAX_DELAY);
}

bool os_acquire_mutex_from_isr(os_mutex_t mutex, bool *awakened)
{
    BaseType_t task_awakened = pdFALSE;

    bool results = xSemaphoreTakeFromISR(mutex, &task_awakened) == pdTRUE;

    if (awakened != NULL) {
        *awakened = task_awakened == pdTRUE;
    }

    return results;
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

bool os_release_mutex_from_isr(os_mutex_t mutex, bool* awakened)
{
    BaseType_t task_awakened = pdFALSE;

    xSemaphoreGiveFromISR(mutex, &task_awakened);

    if (awakened != NULL) {
        *awakened = task_awakened == pdTRUE;
    }

    return true;
}

os_queue_t os_create_queue(int depth, size_t size)
{
    ESP_LOGI(TAG, "%s: depth %d size %u", __func__, depth, size);

    os_queue_t q = xQueueCreate((UBaseType_t) depth, (UBaseType_t) size);

    ESP_LOGI(TAG, "%s: returned %p", __func__, q);

    return q;
}

bool os_delete_queue(os_queue_t queue)
{
    vQueueDelete(queue);
    return true;
}

bool os_put_queue_with_timeout(os_queue_t queue, os_queue_item_t item, int timeout)
{
    return xQueueSend(queue, &item, pdMS_TO_TICKS(timeout)) == pdTRUE;
}

bool os_get_queue_from_isr(os_queue_t queue, os_queue_item_t* item, bool *awakened)
{
    BaseType_t task_awakened = pdFALSE;

    bool results =  xQueueSendFromISR(queue, item, &task_awakened) == pdTRUE;

    if (awakened != NULL) {
        *awakened = task_awakened == pdTRUE;
    }

    return results;
}

bool os_put_queue(os_queue_t queue, os_queue_item_t item)
{
    return os_put_queue_with_timeout(queue, item, portMAX_DELAY);
}

bool os_put_queue_from_isr(os_queue_t queue, os_queue_item_t item, bool* awakened)
{
    BaseType_t task_awakened = pdFALSE;

    bool results = xQueueSendFromISR(queue, &item, &task_awakened) == pdTRUE;
    if (awakened != NULL) {
        *awakened = task_awakened == pdTRUE;
    }

    return results;
}

bool os_get_queue_with_timeout(os_queue_t queue, os_queue_item_t* item, int timeout)
{
    return (xQueueReceive(queue, item, pdMS_TO_TICKS(timeout)) == pdTRUE);
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

bool os_peek_queue(os_queue_t queue, os_queue_item_t* item)
{
    return xQueuePeek(queue, (void**) item, 0) == pdTRUE;
}

bool os_peek_queue_from_isr(os_queue_t queue, os_queue_item_t* item)
{
    return xQueuePeekFromISR(queue, (void**) item) == pdTRUE;
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

os_thread_t os_create_thread_on_core(void (*process)(void* param), const char* name, size_t stack_size, int priority, void* param, int core)
{
    os_thread_t thread;
    if (xTaskCreatePinnedToCore(process, name, stack_size ? stack_size : configMINIMAL_STACK_SIZE, param, priority, &thread, core) != pdTRUE) {
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

void os_delay(int ms)
{
    int ticks = 0;

    /* If a time is specified, compute nearest ticks, but always delay at least one. */
    if (ms != 0) {
        ticks = pdMS_TO_TICKS(ms);
        if (ticks == 0) {
            ticks = 1;
        }
    }

    vTaskDelay(ticks);
}

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

        unsigned long current_ticks = xTaskGetTickCount();
        total_ticks += current_ticks - last_ticks;
        last_ticks = current_ticks;

        ret = u64TICKS_TO_MS(total_ticks);

        os_release_mutex(ticks_mutex);
    }

    return ret;
}

static int num_isrs_installed;

bool os_attach_gpio_interrupt(int gpio, GPIO_INT_TYPE edge, gpio_pullup_t pullup, gpio_pulldown_t pulldown, void (*handler)(void*), void* param)
{
    bool ok = true;

    ESP_LOGI(TAG, "%s: gpio %d edge %d handler %p param %p", __func__, gpio, edge, handler, param);

    gpio_config_t     io;

    io.pin_bit_mask  = 1ULL << gpio;
    io.mode          = GPIO_MODE_INPUT;
    io.intr_type     = edge;
    io.pull_up_en    = pullup;
    io.pull_down_en  = pulldown;

    /* Program the pin */
    if (gpio_config(&io) == ESP_OK) {

        /* If attaching vector */
        if (handler != NULL) {
            if (num_isrs_installed == 0) {
                esp_err_t err = gpio_install_isr_service(0);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "%s: gpio_install_isr_service failed with %s", __func__, esp_err_to_name(err));
                    ok = false;
                }
            }

            if (ok) {
                esp_err_t err = gpio_isr_handler_add(gpio, handler, param);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "%s: gpio_isr_handler_add failed with %s", __func__, esp_err_to_name(err));
                    ok = false;
                }
            }

            if (ok) {
                ++num_isrs_installed;
            } else if (num_isrs_installed == 0) {
                /* First one failed so remove handler */
                gpio_uninstall_isr_service();
            }
        } else {
            /* Remove handler */
            ok = gpio_isr_handler_remove(gpio) == ESP_OK;

            /* Remove service if no other attached */
            if (ok && --num_isrs_installed == 0) {
                gpio_uninstall_isr_service();
            }
        }
    }

    return ok;
}

void* os_alloc_dma_memory(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_DMA);
}

