#ifndef MODBUS_H_INCLUDED
#define MODBUS_H_INCLUDED

#include <stdint.h>
#include <stdlib.h>
#include "model/model.h"

typedef enum {
    MODBUS_RESPONSE_CODE_INFO,
    MODBUS_RESPONSE_CODE_STATE,
    MODBUS_RESPONSE_CODE_SCAN_DONE,
    MODBUS_RESPONSE_CODE_DEVICE_OK,
    MODBUS_RESPONSE_CODE_ERROR,
    MODBUS_RESPONSE_CODE_ALARM,
    MODBUS_RESPONSE_CODE_EVENTS,
    MODBUS_RESPONSE_CODE_WORK_HOURS,
} modbus_response_code_t;

typedef struct {
    modbus_response_code_t code;
    uint8_t                address;
    int                    error;
    int                    scanning;
    int                    devices_number;
    union {
        struct {
            uint16_t class;
            uint16_t firmware_version;
            uint32_t serial_number;
        };
        struct {
            uint16_t alarms;
            uint16_t state;
        };
        struct {
            int16_t temperature;
            int16_t pressure;
            int16_t humidity;
        };
        uint16_t work_hours;
        struct {
            uint16_t event_count;
        };
        modbus_response_code_t success_code;
    };
} modbus_response_t;


void modbus_init(void);
void modbus_read_device_info(uint8_t address);
void modbus_read_device_messages(uint8_t address, uint8_t device_model);
void modbus_read_device_inputs(uint8_t address);
void modbus_set_device_output(uint8_t address, uint8_t value, uint8_t bypass);
void modbus_automatic_commissioning(uint16_t expected_devices);
int  modbus_automatic_commissioning_done(unsigned long millis);
int  modbus_get_response(modbus_response_t *response);
void modbus_set_class_output(uint16_t class, uint8_t value, uint8_t bypass);
void modbus_scan(void);
void modbus_stop_current_operation(void);
void modbus_set_fan_percentage(uint8_t address, uint8_t percentage);
void modbus_read_device_state(uint8_t address);
void modbus_read_device_pressure(uint8_t address);
void modbus_update_time(void);
void modbus_read_device_work_hours(uint8_t address);
void modbus_reset_device_work_hours(uint8_t address);

#endif
