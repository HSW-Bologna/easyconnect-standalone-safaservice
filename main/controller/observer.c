#include "watcher.h"
#include "model/model.h"
#include "observer.h"
#include "services/system_time.h"
#include "bsp/interface.h"
#include "esp_log.h"
#include "modbus.h"
#include "easyconnect_interface.h"


static void sequence_changed_cb(void *old_value, const void *new_value, watcher_size_t size, void *user_ptr, void *arg);
static void ballast_changed_cb(void *old_value, const void *new_value, watcher_size_t size, void *user_ptr, void *arg);
static void work_hours_changed_cb(void *old_value, const void *new_value, watcher_size_t size, void *user_ptr,
                                  void *arg);
static void update_all_ballast(model_t *pmodel, int value);


static const char *TAG     = "Observer";
static watcher_t   watcher = {0};


void observer_init(model_t *pmodel) {
    WATCHER_INIT_STD(&watcher, (void *)pmodel);

    WATCHER_ADD_ENTRY(&watcher, &pmodel->sequence, sequence_changed_cb, NULL);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[0].comm_ok, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_1);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[0].state, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_1);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[0].alarms, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_1);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[0].class, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_1);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[0].work_hours, work_hours_changed_cb, NULL);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[1].comm_ok, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_2);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[1].state, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_2);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[1].alarms, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_2);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[1].class, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_2);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[1].work_hours, work_hours_changed_cb, NULL);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[2].comm_ok, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_3);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[2].state, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_3);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[2].alarms, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_3);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[2].class, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_3);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[2].work_hours, work_hours_changed_cb, NULL);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[3].comm_ok, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_4);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[3].state, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_4);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[3].alarms, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_4);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[3].class, ballast_changed_cb, (void *)(uintptr_t)INTERFACE_LED_4);
    WATCHER_ADD_ENTRY(&watcher, &pmodel->ballast[3].work_hours, work_hours_changed_cb, NULL);

    watcher_trigger_all(&watcher);

    ESP_LOGI(TAG, "Initialized");
}


void observer_manage(void) {
    static unsigned long ts = 0;

    watcher_watch(&watcher, get_millis());


    if (is_expired(ts, get_millis(), 4000)) {
        // Every 4 seconds refresh everything
        watcher_trigger_all(&watcher);
        ts = get_millis();
    }
}


static void ballast_changed_cb(void *old_value, const void *new_value, watcher_size_t size, void *user_ptr, void *arg) {
    (void)old_value;
    (void)new_value;
    (void)size;

    model_t        *pmodel  = user_ptr;
    interface_led_t ballast = (interface_led_t)(uintptr_t)arg;

    if (model_is_ballast_configured_correctly(pmodel, ballast)) {
        if (pmodel->ballast[ballast].state == 0) {
            ESP_LOGD(TAG, "Ballast %i off", ballast);
            interface_set_led_state_off(ballast);
        } else if (pmodel->ballast[ballast].alarms) {
            ESP_LOGD(TAG, "Ballast %i with alarms", ballast);
            interface_set_led_state_blink(ballast);
        } else {
            ESP_LOGD(TAG, "Ballast %i on", ballast);
            interface_set_led_state_on(ballast);
        }
    } else {
        ESP_LOGD(TAG, "Ballast %i error", ballast);
        interface_set_led_state_off(ballast);
    }
}


static void work_hours_changed_cb(void *old_value, const void *new_value, watcher_size_t size, void *user_ptr,
                                  void *arg) {
    (void)old_value;
    (void)new_value;
    (void)size;
    (void)arg;

    model_t *pmodel = user_ptr;
    if (model_get_working_hours_alarm(pmodel)) {
        interface_set_alarm();
    } else if (model_get_working_hours_warning(pmodel)) {
        interface_set_warning();
    } else {
        interface_set_warning_alarm_off();
    }
}


static void sequence_changed_cb(void *old_value, const void *new_value, watcher_size_t size, void *user_ptr,
                                void *arg) {
    (void)old_value;
    (void)new_value;
    (void)size;
    (void)arg;

    model_t *pmodel = user_ptr;
    switch (pmodel->sequence) {
        case BALLAST_SEQUENCE_NONE:
            update_all_ballast(pmodel, 0);
            break;

        case BALLAST_SEQUENCE_1:
            modbus_set_device_output(1, 1, 0);
            modbus_set_device_output(2, 0, 0);
            modbus_set_device_output(3, 0, 0);
            modbus_set_device_output(4, 0, 0);
            modbus_read_device_state(1);
            break;

        case BALLAST_SEQUENCE_2:
            modbus_set_device_output(2, 1, 0);
            modbus_set_device_output(1, 1, 0);
            modbus_set_device_output(3, 0, 0);
            modbus_set_device_output(4, 0, 0);
            modbus_read_device_state(2);
            break;

        case BALLAST_SEQUENCE_3:
            modbus_set_device_output(3, 1, 0);
            modbus_set_device_output(2, 1, 0);
            modbus_set_device_output(1, 1, 0);
            modbus_set_device_output(4, 0, 0);
            modbus_read_device_state(3);
            break;

        case BALLAST_SEQUENCE_4:
            update_all_ballast(pmodel, 1);
            modbus_read_device_state(4);
            break;

        case BALLAST_SEQUENCE_DONE:
            break;
    }
}


static void update_all_ballast(model_t *pmodel, int value) {
    modbus_set_device_output(4, value > 0, 0);
    modbus_set_device_output(3, value > 0, 0);
    modbus_set_device_output(2, value > 0, 0);
    modbus_set_device_output(1, value > 0, 0);
}
