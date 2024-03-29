#include <assert.h>
#include "model.h"
#include "config/app_config.h"
#include "easyconnect_interface.h"
#include "esp_log.h"


static size_t ballast_from_address(uint8_t address);


static const char *TAG = "Model";


void model_init(mut_model_t *pmodel) {
    assert(pmodel != NULL);

    for (size_t i = 0; i < MODBUS_MAX_DEVICES; i++) {
        pmodel->ballast[i].comm_ok = 0;
        pmodel->ballast[i].alarms  = 0;
        pmodel->ballast[i].state   = 0;
        pmodel->ballast[i].class   = 0;
    }

    pmodel->sequence    = BALLAST_SEQUENCE_1;
    pmodel->sequence_ts = 0;
    pmodel->safety_ok   = 0;

    ESP_LOGI(TAG, "Initialized");
}


uint8_t model_are_all_ballast_working(model_t *pmodel) {
    assert(pmodel != NULL);

    for (size_t i = 0; i < MODBUS_MAX_DEVICES; i++) {
        if (pmodel->ballast[i].alarms > 0) {
            return 0;
        }
    }

    return 1;
}


uint8_t model_is_ballast_configured_correctly(model_t *pmodel, size_t ballast) {
    assert(pmodel != NULL);
    return pmodel->ballast[ballast].comm_ok && CLASS_GET_MODE(pmodel->ballast[ballast].class) == DEVICE_MODE_UVC;
}


void model_set_ballast_communication_ok(mut_model_t *pmodel, uint8_t address, uint8_t comm_ok) {
    assert(pmodel != NULL);
    pmodel->ballast[ballast_from_address(address)].comm_ok = comm_ok;
    if (pmodel->ballast[ballast_from_address(address)].present == BALLAST_PRESENCE_UNKNOWN) {
        pmodel->ballast[ballast_from_address(address)].present =
            comm_ok ? BALLAST_PRESENCE_FOUND : BALLAST_PRESENCE_MISSING;
    }
}


void model_set_ballast_class(mut_model_t *pmodel, uint8_t address, uint16_t class) {
    assert(pmodel != NULL);
    pmodel->ballast[ballast_from_address(address)].class = class;
}


void model_set_ballast_state(mut_model_t *pmodel, uint8_t address, uint16_t state, uint16_t alarms) {
    assert(pmodel != NULL);
    pmodel->ballast[ballast_from_address(address)].comm_ok = 1;
    pmodel->ballast[ballast_from_address(address)].state   = state;
    pmodel->ballast[ballast_from_address(address)].alarms  = alarms;
    pmodel->ballast[ballast_from_address(address)].present = BALLAST_PRESENCE_FOUND;
}


void model_set_ballast_work_hours(mut_model_t *pmodel, uint8_t address, uint16_t work_hours) {
    assert(pmodel != NULL);
    pmodel->ballast[ballast_from_address(address)].comm_ok    = 1;
    pmodel->ballast[ballast_from_address(address)].work_hours = work_hours;
    pmodel->ballast[ballast_from_address(address)].present    = BALLAST_PRESENCE_FOUND;
}


uint8_t model_is_safety_ok(model_t *pmodel) {
    assert(pmodel != NULL);
    for (size_t i = 0; i < MODBUS_MAX_DEVICES; i++) {
        if ((pmodel->ballast[i].alarms & EASYCONNECT_SAFETY_ALARM) > 0) {
            return 0;
        }
    }
    return pmodel->safety_ok;
}


uint8_t model_get_working_hours_warning(model_t *pmodel) {
    assert(pmodel != NULL);
    for (size_t i = 0; i < MODBUS_MAX_DEVICES; i++) {
        if (pmodel->ballast[i].work_hours >= APP_CONFIG_HOURS_WARNING) {
            return 1;
        }
    }
    return 0;
}


uint8_t model_get_working_hours_alarm(model_t *pmodel) {
    assert(pmodel != NULL);
    for (size_t i = 0; i < MODBUS_MAX_DEVICES; i++) {
        if (pmodel->ballast[i].work_hours >= APP_CONFIG_HOURS_ALARM) {
            return 1;
        }
    }
    return 0;
}


uint8_t model_ballast_should_be_on(model_t *pmodel, size_t ballast) {
    assert(pmodel != NULL);
    switch (pmodel->sequence) {
        case BALLAST_SEQUENCE_1:
            return ballast == 0;
        case BALLAST_SEQUENCE_2:
            return ballast <= 1;
        case BALLAST_SEQUENCE_3:
            return ballast <= 2;
        case BALLAST_SEQUENCE_4:
        case BALLAST_SEQUENCE_DONE:
            return ballast <= 3;
        default:
            return 0;
    }
}


uint8_t model_ballast_present(model_t *pmodel, size_t ballast) {
    assert(pmodel != NULL);
    return pmodel->ballast[ballast].present != BALLAST_PRESENCE_MISSING;
}


static size_t ballast_from_address(uint8_t address) {
    return address - 1;
}
