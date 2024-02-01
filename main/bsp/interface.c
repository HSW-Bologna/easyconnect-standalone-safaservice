#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "hardwareprofile.h"
#include "interface.h"
#include "keypad.h"
#include <esp_err.h>
#include "services/system_time.h"
#include "esp_log.h"


#define NUM_KEYS 1


typedef struct {
    gpio_num_t gpio;
    uint8_t    state;
} led_ballast_t;


static void ballast_timer_cb(TimerHandle_t timer);
static void warning_timer_cb(TimerHandle_t timer);


static const char   *TAG = "Interface";
static TimerHandle_t timers[NUM_LED_BALLAST];
static TimerHandle_t warning_timer;
static keypad_key_t  keys[NUM_KEYS + 1] = {
    {.bitvalue = 1, .code = 1},
    KEYPAD_NULL_KEY,
};


void interface_init(void) {
    keypad_reset_keys(keys);

    gpio_config_t config_out = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(HAP_BAL1) | BIT64(HAP_BAL2) | BIT64(HAP_BAL3) | BIT64(HAP_BAL4) |
                        BIT64(HAP_SAFETY_ALARM) | BIT64(HAP_EXP_LIFE_LAMP) | BIT64(HAP_LED_R),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config_out));

    gpio_config_t config_in = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT64(HAP_PULS_RESET_LIFETIME_LAMP) | BIT64(HAP_LEG_G),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config_in));

    static led_ballast_t led_ballast[NUM_LED_BALLAST];
    static StaticTimer_t timer_buffers[NUM_LED_BALLAST];
    const gpio_num_t     gpios[NUM_LED_BALLAST] = {HAP_BAL1, HAP_BAL2, HAP_BAL3, HAP_BAL4};

    for (size_t i = 0; i < NUM_LED_BALLAST; i++) {
        led_ballast[i].gpio  = gpios[i];
        led_ballast[i].state = 0;
        timers[i] =
            xTimerCreateStatic(TAG, pdMS_TO_TICKS(500), 1, &led_ballast[i], ballast_timer_cb, &timer_buffers[i]);
    }

    static StaticTimer_t timer_buffer;
    warning_timer = xTimerCreateStatic(TAG, pdMS_TO_TICKS(1000), 1, NULL, warning_timer_cb, &timer_buffer);

    interface_set_warning_alarm_off();
    interface_set_led_state_off(INTERFACE_LED_1);
    interface_set_led_state_off(INTERFACE_LED_2);
    interface_set_led_state_off(INTERFACE_LED_3);
    interface_set_led_state_off(INTERFACE_LED_4);
}


uint8_t interface_manage(void) {
    uint32_t       level = !gpio_get_level(HAP_PULS_RESET_LIFETIME_LAMP);
    keypad_event_t event = keypad_routine(keys, 40, 5000, 500, get_millis(), level);
    return event.tag == KEYPAD_EVENT_TAG_LONGCLICK || event.tag == KEYPAD_EVENT_TAG_LONGPRESSING;
}


void interface_set_safety(uint8_t led) {
    gpio_set_level(HAP_SAFETY_ALARM, led);
}


void interface_set_led_state_off(interface_led_t led) {
    const gpio_num_t gpios[NUM_LED_BALLAST] = {HAP_BAL1, HAP_BAL2, HAP_BAL3, HAP_BAL4};
    xTimerStop(timers[led], portMAX_DELAY);
    gpio_set_level(gpios[led], 0);
}


void interface_set_led_state_on(interface_led_t led) {
    const gpio_num_t gpios[NUM_LED_BALLAST] = {HAP_BAL1, HAP_BAL2, HAP_BAL3, HAP_BAL4};
    xTimerStop(timers[led], portMAX_DELAY);
    gpio_set_level(gpios[led], 1);
}


void interface_set_led_state_blink(interface_led_t led, unsigned long millis) {
    xTimerChangePeriod(timers[led], pdMS_TO_TICKS(millis), portMAX_DELAY);
    xTimerStart(timers[led], portMAX_DELAY);
}


void interface_set_warning_alarm_off(void) {
    xTimerStop(warning_timer, portMAX_DELAY);
    gpio_set_level(HAP_EXP_LIFE_LAMP, 0);
}


void interface_set_warning(void) {
    xTimerStart(warning_timer, portMAX_DELAY);
}


void interface_set_alarm(void) {
    xTimerStop(warning_timer, portMAX_DELAY);
    gpio_set_level(HAP_EXP_LIFE_LAMP, 1);
}


static void ballast_timer_cb(TimerHandle_t timer) {
    led_ballast_t *led_ballast = pvTimerGetTimerID(timer);
    gpio_set_level(led_ballast->gpio, led_ballast->state);
    led_ballast->state = !led_ballast->state;
}


static void warning_timer_cb(TimerHandle_t timer) {
    static uint8_t blink = 0;
    gpio_set_level(HAP_EXP_LIFE_LAMP, blink);
    blink = !blink;
}
