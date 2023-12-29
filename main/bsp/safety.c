#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "hardwareprofile.h"
#include "debounce.h"
#include <esp_log.h>


#define EVENT_NEW_INPUT 0x01


static void periodic_read(TimerHandle_t timer);


static debounce_filter_t filter = {0};
static SemaphoreHandle_t sem    = NULL;
static const char       *TAG    = "Safety";


void safety_init(void) {
    gpio_config_t config_in = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT64(HAP_SAFETY_INPUT),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config_in));

    debounce_filter_init(&filter);
    static StaticSemaphore_t semaphore_buffer;
    sem = xSemaphoreCreateMutexStatic(&semaphore_buffer);

    static StaticTimer_t timer_buffer;
    TimerHandle_t        timer =
        xTimerCreateStatic("timerSafety", pdMS_TO_TICKS(10), pdTRUE, NULL, periodic_read, &timer_buffer);
    xTimerStart(timer, portMAX_DELAY);
}


uint8_t safety_ok(void) {
    uint8_t res = 0;
    xSemaphoreTake(sem, portMAX_DELAY);
    res = debounce_read(&filter, 0);
    xSemaphoreGive(sem);
    return res != 0;
}


static uint8_t take_reading(void) {
    unsigned int input = 0;
    input |= !gpio_get_level(HAP_SAFETY_INPUT);
    ESP_LOGD(TAG, "%i", input);
    return debounce_filter(&filter, input, 5);
}


static void periodic_read(TimerHandle_t timer) {
    (void)timer;
    xSemaphoreTake(sem, portMAX_DELAY);
    if (take_reading()) {
        // New value
    }
    xSemaphoreGive(sem);
}
