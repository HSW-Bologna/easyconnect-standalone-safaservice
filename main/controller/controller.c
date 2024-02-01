#include "controller.h"
#include "model/model.h"
#include "modbus.h"
#include "observer.h"
#include "model/updater.h"
#include "services/system_time.h"
#include "bsp/interface.h"
#include "esp_log.h"
#include "bsp/safety.h"


static const char *TAG = "Controller";


void controller_init(mut_model_t *pmodel) {
    (void)pmodel;

    modbus_init();
    observer_init(pmodel);
}


void controller_manage(mut_model_t *pmodel) {
    static unsigned long modbus_ts      = 0;
    static size_t        info_counter   = 0;
    static uint8_t       modbus_address = 1;

    if (is_expired(modbus_ts, get_millis(), 200)) {
        if ((info_counter % 50) == 0) {
            modbus_read_device_work_hours(modbus_address);
            modbus_read_device_info(modbus_address);
        }

        modbus_read_device_state(modbus_address);

        if (modbus_address == 4) {
            modbus_address = 1;
            info_counter++;
        } else {
            modbus_address++;
        }

        modbus_ts = get_millis();
    }

    if (interface_manage()) {
        ESP_LOGI(TAG, "Reset work hours");
        model_set_ballast_work_hours(pmodel, 1, 0);
        model_set_ballast_work_hours(pmodel, 2, 0);
        model_set_ballast_work_hours(pmodel, 3, 0);
        model_set_ballast_work_hours(pmodel, 4, 0);
        modbus_reset_device_work_hours(1);
        modbus_reset_device_work_hours(2);
        modbus_reset_device_work_hours(3);
        modbus_reset_device_work_hours(4);
    }

    pmodel->safety_ok = safety_ok();
    interface_set_safety(!model_is_safety_ok(pmodel));

    modbus_response_t response;
    if (modbus_get_response(&response)) {
        switch (response.code) {
            case MODBUS_RESPONSE_CODE_ERROR:
                model_set_ballast_communication_ok(pmodel, response.address, 0);
                break;

            case MODBUS_RESPONSE_CODE_INFO:
                ESP_LOGD(TAG, "Device %i has class 0x%02X", response.address, response.class);
                model_set_ballast_class(pmodel, response.address, response.class);
                break;

            case MODBUS_RESPONSE_CODE_STATE:
                ESP_LOGD(TAG, "Device %i state: 0x%02X; alarms: 0x%02X", response.address, response.state,
                         response.alarms);
                model_set_ballast_state(pmodel, response.address, response.state, response.alarms);
                break;

            case MODBUS_RESPONSE_CODE_WORK_HOURS:
                ESP_LOGD(TAG, "Device %i has worked for %0ih", response.address, response.work_hours);
                model_set_ballast_work_hours(pmodel, response.address, response.work_hours);
                break;

            default:
                break;
        }
    }


    observer_manage();
    model_updater_manage(pmodel);
}
