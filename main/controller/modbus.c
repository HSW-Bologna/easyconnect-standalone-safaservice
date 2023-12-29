#include <inttypes.h>
#include <sys/time.h>
#include <string.h>
#include <assert.h>
#include <sys/types.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "config/app_config.h"
#include "lightmodbus/lightmodbus.h"
#include "lightmodbus/master_func.h"
#include "services/system_time.h"
#include "utils/utils.h"
#include "model/model.h"
#include "easyconnect_interface.h"
#include "modbus.h"
#include "bsp/rs485.h"
#include "config/app_config.h"


#define MODBUS_RESPONSE_03_LEN(data_len) (5 + data_len * 2)
#define MODBUS_RESPONSE_05_LEN           8

#define MODBUS_MESSAGE_QUEUE_SIZE     512
#define MODBUS_TIMEOUT                30
#define MODBUS_MAX_PACKET_SIZE        256
#define MODBUS_BROADCAST_ADDRESS      0
#define MODBUS_COMMUNICATION_ATTEMPTS 1

#define HOLDING_REGISTER_MOTOR_SPEED 256
#define HOLDING_REGISTER_PRESSURE    256
#define HOLDING_REGISTER_WORK_HOURS  256

#define MODBUS_AUTO_COMMISSIONING_DONE_BIT 0x01

typedef enum {
    TASK_MESSAGE_CODE_READ_DEVICE_INFO,
    TASK_MESSAGE_CODE_READ_DEVICE_STATE,
    TASK_MESSAGE_CODE_READ_DEVICE_INPUTS,
    TASK_MESSAGE_CODE_READ_DEVICE_WORK_HOURS,
    TASK_MESSAGE_CODE_RESET_DEVICE_WORK_HOURS,
    TASK_MESSAGE_CODE_SET_DEVICE_OUTPUT,
    TASK_MESSAGE_CODE_SET_CLASS_OUTPUT,
    TASK_MESSAGE_CODE_SET_FAN_PERCENTAGE,
    TASK_MESSAGE_CODE_UPDATE_TIME,
    TASK_MESSAGE_CODE_UPDATE_EVENTS,
    TASK_MESSAGE_CODE_SCAN,
} task_message_code_t;


struct __attribute__((packed)) task_message {
    task_message_code_t code;
    uint8_t             address;
    union {
        struct {
            uint16_t class;
            uint8_t value;
            uint8_t bypass;
        };
        uint16_t expected_devices;
        uint16_t num_messages;
        uint16_t event_count;
    };
};

typedef struct {
    uint32_t device_map[MODBUS_MAX_DEVICES];
} device_map_context_t;

typedef struct {
    uint16_t start;
    void    *pointer;
} master_context_t;


static inline __attribute__((always_inline)) size_t serialize_uint64_be(uint8_t *buf, uint64_t val) {
    buf[0] = (val >> 56) & 0xFF;
    buf[1] = (val >> 48) & 0xFF;
    buf[2] = (val >> 40) & 0xFF;
    buf[3] = (val >> 32) & 0xFF;
    buf[4] = (val >> 24) & 0xFF;
    buf[5] = (val >> 16) & 0xFF;
    buf[6] = (val >> 8) & 0xFF;
    buf[7] = val & 0xFF;
    return 8;
}

static LIGHTMODBUS_RET_ERROR build_custom_request(ModbusMaster *status, uint8_t function, uint8_t *data, size_t len);
static LIGHTMODBUS_RET_ERROR random_serial_number_response(ModbusMaster *status, uint8_t address, uint8_t function,
                                                           const uint8_t *requestPDU, uint8_t requestLength,
                                                           const uint8_t *responsePDU, uint8_t responseLength);

static void modbus_task(void *args);
static int  write_holding_register(ModbusMaster *master, uint8_t address, uint16_t index, uint16_t data);
static int  write_holding_registers(ModbusMaster *master, uint8_t address, uint16_t starting_address, uint16_t *data,
                                    size_t num);
static int  write_coils(ModbusMaster *master, uint8_t address, uint16_t index, size_t num_values, uint8_t *values);
static int  read_holding_registers(ModbusMaster *master, uint16_t *registers, uint8_t address, uint16_t start,
                                   uint16_t count);
static void send_custom_function(ModbusMaster *master, uint8_t address, uint8_t function, uint8_t *data, size_t len);

static const char   *TAG       = "Modbus";
static QueueHandle_t messageq  = NULL;
static QueueHandle_t responseq = NULL;
static TaskHandle_t  task      = NULL;


static ModbusMasterFunctionHandler custom_functions[] = {
#if defined(LIGHTMODBUS_F01M) || defined(LIGHTMODBUS_MASTER_FULL)
    {1, modbusParseResponse01020304},
#endif

#if defined(LIGHTMODBUS_F02M) || defined(LIGHTMODBUS_MASTER_FULL)
    {2, modbusParseResponse01020304},
#endif

#if defined(LIGHTMODBUS_F03M) || defined(LIGHTMODBUS_MASTER_FULL)
    {3, modbusParseResponse01020304},
#endif

#if defined(LIGHTMODBUS_F04M) || defined(LIGHTMODBUS_MASTER_FULL)
    {4, modbusParseResponse01020304},
#endif

#if defined(LIGHTMODBUS_F05M) || defined(LIGHTMODBUS_MASTER_FULL)
    {5, modbusParseResponse0506},
#endif

#if defined(LIGHTMODBUS_F06M) || defined(LIGHTMODBUS_MASTER_FULL)
    {6, modbusParseResponse0506},
#endif

#if defined(LIGHTMODBUS_F15M) || defined(LIGHTMODBUS_MASTER_FULL)
    {15, modbusParseResponse1516},
#endif

#if defined(LIGHTMODBUS_F16M) || defined(LIGHTMODBUS_MASTER_FULL)
    {16, modbusParseResponse1516},
#endif

#if defined(LIGHTMODBUS_F22M) || defined(LIGHTMODBUS_MASTER_FULL)
    {22, modbusParseResponse22},
#endif

    {EASYCONNECT_FUNCTION_CODE_RANDOM_SERIAL_NUMBER, random_serial_number_response},
    // Guard - prevents 0 size array
    {0, NULL},
};


void modbus_init(void) {
    static StaticQueue_t static_queue1;
    static uint8_t       queue_buffer1[MODBUS_MESSAGE_QUEUE_SIZE * sizeof(struct task_message)] = {0};
    messageq =
        xQueueCreateStatic(MODBUS_MESSAGE_QUEUE_SIZE, sizeof(struct task_message), queue_buffer1, &static_queue1);

    static StaticQueue_t static_queue2;
    static uint8_t       queue_buffer2[MODBUS_MESSAGE_QUEUE_SIZE * sizeof(modbus_response_t)] = {0};
    responseq = xQueueCreateStatic(MODBUS_MESSAGE_QUEUE_SIZE, sizeof(modbus_response_t), queue_buffer2, &static_queue2);

#ifdef PC_SIMULATOR
    xTaskCreate(modbus_task, TAG, APP_CONFIG_BASE_TASK_STACK_SIZE * 6, NULL, 5, &task);
#else
    static uint8_t      task_stack[APP_CONFIG_BASE_TASK_STACK_SIZE * 6] = {0};
    static StaticTask_t static_task;
    task = xTaskCreateStatic(modbus_task, TAG, sizeof(task_stack), NULL, 5, task_stack, &static_task);
#endif
}


int modbus_get_response(modbus_response_t *response) {
    return xQueueReceive(responseq, response, 0) == pdTRUE;
}


void modbus_update_events(uint8_t address, uint16_t previous_event_count) {
    struct task_message message = {
        .code = TASK_MESSAGE_CODE_UPDATE_EVENTS, .address = address, .event_count = previous_event_count};
    xQueueSend(messageq, &message, 0);
}


void modbus_update_time(void) {
    struct task_message message = {.code = TASK_MESSAGE_CODE_UPDATE_TIME};
    xQueueSend(messageq, &message, 0);
}


void modbus_set_class_output(uint16_t class, uint8_t value, uint8_t bypass) {
    struct task_message message = {
        .code   = TASK_MESSAGE_CODE_SET_CLASS_OUTPUT,
        .class  = class,
        .value  = value,
        .bypass = bypass,
    };
    xQueueSend(messageq, &message, 0);
}


void modbus_set_fan_percentage(uint8_t address, uint8_t percentage) {
    struct task_message message = {
        .code = TASK_MESSAGE_CODE_SET_FAN_PERCENTAGE, .address = address, .value = percentage};
    xQueueSend(messageq, &message, 0);
}


void modbus_scan(void) {
    struct task_message message = {.code = TASK_MESSAGE_CODE_SCAN};
    xQueueSend(messageq, &message, 0);
}


void modbus_set_device_output(uint8_t address, uint8_t value, uint8_t bypass) {
    struct task_message message = {
        .code    = TASK_MESSAGE_CODE_SET_DEVICE_OUTPUT,
        .address = address,
        .value   = value,
        .bypass  = bypass,
    };
    xQueueSend(messageq, &message, 0);
}


void modbus_read_device_info(uint8_t address) {
    struct task_message message = {.code = TASK_MESSAGE_CODE_READ_DEVICE_INFO, .address = address};
    xQueueSend(messageq, &message, 0);
}



void modbus_read_device_state(uint8_t address) {
    struct task_message message = {.code = TASK_MESSAGE_CODE_READ_DEVICE_STATE, .address = address};
    xQueueSend(messageq, &message, 0);
}


void modbus_read_device_work_hours(uint8_t address) {
    struct task_message message = {.code = TASK_MESSAGE_CODE_READ_DEVICE_WORK_HOURS, .address = address};
    xQueueSend(messageq, &message, 0);
}


void modbus_reset_device_work_hours(uint8_t address) {
    struct task_message message = {.code = TASK_MESSAGE_CODE_RESET_DEVICE_WORK_HOURS, .address = address};
    xQueueSend(messageq, &message, 0);
    modbus_read_device_work_hours(address);
}


void modbus_read_device_inputs(uint8_t address) {
    struct task_message message = {.code = TASK_MESSAGE_CODE_READ_DEVICE_INPUTS, .address = address};
    xQueueSend(messageq, &message, 0);
}


void modbus_stop_current_operation(void) {
    xTaskNotifyGive(task);
}


static ModbusError data_callback(const ModbusMaster *master, const ModbusDataCallbackArgs *args) {
    master_context_t *ctx = modbusMasterGetUserPointer(master);

    if (ctx != NULL) {
        switch (args->type) {
            case MODBUS_HOLDING_REGISTER: {
                uint16_t *buffer                 = ctx->pointer;
                buffer[args->index - ctx->start] = args->value;
                break;
            }

            case MODBUS_DISCRETE_INPUT: {
                uint8_t *buffer                  = ctx->pointer;
                buffer[args->index - ctx->start] = args->value;
                break;
            }

            case MODBUS_INPUT_REGISTER: {
                uint16_t *buffer                 = ctx->pointer;
                buffer[args->index - ctx->start] = args->value;
                break;
            }

            case MODBUS_COIL: {
                uint8_t *buffer                  = ctx->pointer;
                buffer[args->index - ctx->start] = args->value;
                break;
            }
        }
    }

    return MODBUS_OK;
}


static ModbusError exception_callback(const ModbusMaster *master, uint8_t address, uint8_t function,
                                      ModbusExceptionCode code) {
    modbus_response_t *response = modbusMasterGetUserPointer(master);
    // printf("Received exception (function %d) from slave %d code %d\n", function, address, code);

    if (response != NULL) {
        switch (code) {
            case MODBUS_EXCEP_SLAVE_FAILURE:
                response->address = address;
                response->code    = MODBUS_RESPONSE_CODE_ALARM;
                break;

            default:
                response->address = address;
                response->code    = MODBUS_RESPONSE_CODE_ERROR;
                break;
        }
    }

    return MODBUS_OK;
}


static void modbus_task(void *args) {
    (void)args;
    ModbusMaster    master;
    ModbusErrorInfo err = modbusMasterInit(&master,
                                           data_callback,              // Callback for handling incoming data
                                           exception_callback,         // Exception callback (optional)
                                           modbusDefaultAllocator,     // Memory allocator used to allocate request
                                           custom_functions,           // Set of supported functions
                                           modbusMasterDefaultFunctionCount + 1     // Number of supported functions
    );

    // Check for errors
    assert(modbusIsOk(err) && "modbusMasterInit() failed");
    struct task_message message                        = {0};
    uint8_t             buffer[MODBUS_MAX_PACKET_SIZE] = {0};
    modbus_response_t   error_resp                     = {.code = MODBUS_RESPONSE_CODE_ERROR};
    unsigned long       timestamp                      = 0;

    ESP_LOGI(TAG, "Task starting");
    send_custom_function(&master, MODBUS_BROADCAST_ADDRESS, EASYCONNECT_FUNCTION_CODE_NETWORK_INITIALIZATION, NULL, 0);

    for (;;) {
        xTaskNotifyStateClear(task);

        ESP_LOGD(TAG, "Items: %i", uxQueueMessagesWaiting(messageq));

        if (xQueueReceive(messageq, &message, pdMS_TO_TICKS(100))) {
            modbus_response_t response = {.address = message.address};
            error_resp.address         = message.address;

            switch (message.code) {
                case TASK_MESSAGE_CODE_READ_DEVICE_INFO: {
                    response.code    = MODBUS_RESPONSE_CODE_INFO;
                    response.address = message.address;

                    uint16_t registers[5];
                    if (read_holding_registers(&master, registers, message.address,
                                               EASYCONNECT_HOLDING_REGISTER_FIRMWARE_VERSION,
                                               sizeof(registers) / sizeof(registers[0]))) {
                        error_resp.success_code = MODBUS_RESPONSE_CODE_INFO;
                        xQueueSend(responseq, &error_resp, portMAX_DELAY);
                    } else {
                        response.firmware_version = registers[0];
                        response.class            = registers[1];
                        response.serial_number    = (registers[2] << 16) | registers[3];
                        xQueueSend(responseq, &response, portMAX_DELAY);
                    }
                    break;
                }

                case TASK_MESSAGE_CODE_READ_DEVICE_STATE: {
                    response.code    = MODBUS_RESPONSE_CODE_STATE;
                    response.address = message.address;

                    uint16_t registers[2];
                    if (read_holding_registers(&master, registers, message.address, EASYCONNECT_HOLDING_REGISTER_ALARMS,
                                               sizeof(registers) / sizeof(registers[0]))) {
                        xQueueSend(responseq, &error_resp, portMAX_DELAY);
                    } else {
                        response.alarms = registers[0];
                        response.state  = registers[1];
                        xQueueSend(responseq, &response, portMAX_DELAY);
                    }
                    break;
                }

                case TASK_MESSAGE_CODE_READ_DEVICE_INPUTS: {
                    ESP_LOGI(TAG, "Reading inputs from %i", message.address);
                    err = modbusBuildRequest02RTU(&master, message.address, 0, 2);
                    assert(modbusIsOk(err));
                    rs485_write(modbusMasterGetRequest(&master), modbusMasterGetRequestLength(&master));

                    int len = rs485_read(buffer, sizeof(buffer), pdMS_TO_TICKS(MODBUS_TIMEOUT));
                    err     = modbusParseResponseRTU(&master, modbusMasterGetRequest(&master),
                                                     modbusMasterGetRequestLength(&master), buffer, len);

                    if (!modbusIsOk(err)) {
                        ESP_LOG_BUFFER_HEX(TAG, buffer, len);
                        ESP_LOGW(TAG, "Cold not query device %i: %i %i", message.address, err.source, err.error);
                    }
                    break;
                }

                case TASK_MESSAGE_CODE_SET_DEVICE_OUTPUT: {
                    response.code    = MODBUS_RESPONSE_CODE_DEVICE_OK;
                    response.address = message.address;
                    uint8_t coils    = (message.value << 0) | (message.bypass << 1);
                    if (write_coils(&master, message.address, 0, 2, &coils)) {
                        xQueueSend(responseq, &error_resp, portMAX_DELAY);
                    } else {
                        xQueueSend(responseq, &response, portMAX_DELAY);
                    }
                    break;
                }

                case TASK_MESSAGE_CODE_UPDATE_EVENTS: {
                    uint16_t event_count = 0;

                    if (read_holding_registers(&master, &event_count, message.address,
                                               EASYCONNECT_HOLDING_REGISTER_LOGS_COUNTER, 1)) {
                        xQueueSend(responseq, &error_resp, portMAX_DELAY);
                        break;
                    }

                    if (event_count > message.event_count) {
                        uint16_t count      = event_count - message.event_count;
                        count               = count > 8 ? 8 : count;
                        uint16_t *registers = malloc(count * sizeof(uint16_t));
                        assert(registers != NULL);

                        if (read_holding_registers(&master, registers, message.address,
                                                   EASYCONNECT_HOLDING_REGISTER_LOGS, count)) {
                            xQueueSend(responseq, &error_resp, portMAX_DELAY);
                            break;
                        }

                        ESP_LOGI(TAG, "New events from %i: %i", message.address, count);
                    }
                    break;
                }

                case TASK_MESSAGE_CODE_SCAN: {
                    ESP_LOGI(TAG, "Scan start");
                    response.code     = MODBUS_RESPONSE_CODE_INFO;
                    response.scanning = 1;

                    for (size_t i = 1; i <= MODBUS_MAX_DEVICES; i++) {
                        if (ulTaskNotifyTake(pdTRUE, 0)) {
                            ESP_LOGI(TAG, "Interrupting scan");
                            break;
                        }

                        response.code     = MODBUS_RESPONSE_CODE_INFO;
                        response.address  = i;
                        response.scanning = 1;

                        uint16_t registers[5];
                        if (read_holding_registers(&master, registers, response.address,
                                                   EASYCONNECT_HOLDING_REGISTER_FIRMWARE_VERSION,
                                                   sizeof(registers) / sizeof(registers[0]))) {
                            // No response
                        } else {
                            response.firmware_version = registers[0];
                            response.class            = registers[1];
                            response.serial_number    = (registers[2] << 16) | registers[3];
                            xQueueSend(responseq, &response, portMAX_DELAY);
                        }

                        vTaskDelay(pdMS_TO_TICKS(MODBUS_TIMEOUT));
                    }

                    ESP_LOGI(TAG, "Scan done!");
                    response.code = MODBUS_RESPONSE_CODE_SCAN_DONE;
                    xQueueSend(responseq, &response, portMAX_DELAY);
                    break;
                }

                case TASK_MESSAGE_CODE_UPDATE_TIME: {
                    struct timeval timeval = {0};
                    gettimeofday(&timeval, NULL);
                    uint64_t timestamp = (uint64_t)timeval.tv_sec;

                    uint8_t buffer[8] = {0};
                    serialize_uint64_be(buffer, timestamp);

                    send_custom_function(&master, MODBUS_BROADCAST_ADDRESS, 67, buffer,
                                         sizeof(buffer));     // TODO: use macro
                    break;
                }

                case TASK_MESSAGE_CODE_SET_CLASS_OUTPUT: {
                    uint8_t data[] = {
                        (message.class >> 8) & 0xFF,
                        message.class & 0xFF,
                        message.value,
                        message.bypass,
                    };
                    send_custom_function(&master, MODBUS_BROADCAST_ADDRESS, EASYCONNECT_FUNCTION_CODE_SET_CLASS_OUTPUT,
                                         data, sizeof(data));
                    break;
                }

                case TASK_MESSAGE_CODE_SET_FAN_PERCENTAGE: {
                    ESP_LOGI(TAG, "Setting fan speed for device %i %i%%", message.address, message.value);
                    if (write_holding_register(&master, message.address, HOLDING_REGISTER_MOTOR_SPEED,
                                               (uint16_t)message.value)) {
                        xQueueSend(responseq, &error_resp, portMAX_DELAY);
                    }
                    break;
                }

                case TASK_MESSAGE_CODE_READ_DEVICE_WORK_HOURS: {
                    response.code    = MODBUS_RESPONSE_CODE_WORK_HOURS;
                    response.address = message.address;

                    if (read_holding_registers(&master, &response.work_hours, message.address,
                                               HOLDING_REGISTER_WORK_HOURS, 1)) {
                        xQueueSend(responseq, &error_resp, portMAX_DELAY);
                    } else {
                        xQueueSend(responseq, &response, portMAX_DELAY);
                    }
                    break;
                }

                case TASK_MESSAGE_CODE_RESET_DEVICE_WORK_HOURS: {
                    if (write_holding_register(&master, message.address, HOLDING_REGISTER_WORK_HOURS, 0)) {
                        xQueueSend(responseq, &error_resp, portMAX_DELAY);
                    }
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(MODBUS_TIMEOUT / 2));
        }

        if (is_expired(timestamp, get_millis(), 100)) {
            send_custom_function(&master, MODBUS_BROADCAST_ADDRESS, EASYCONNECT_FUNCTION_CODE_HEARTBEAT, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(MODBUS_TIMEOUT / 2));
            timestamp = get_millis();
        }
    }

    vTaskDelete(NULL);
}


static LIGHTMODBUS_RET_ERROR build_custom_request(ModbusMaster *status, uint8_t function, uint8_t *data, size_t len) {
    if (modbusMasterAllocateRequest(status, len + 1)) {
        return MODBUS_GENERAL_ERROR(ALLOC);
    }

    status->request.pdu[0] = function;
    if (data != NULL) {
        for (size_t i = 0; i < len; i++) {
            status->request.pdu[1 + i] = data[i];
        }
    }

    return MODBUS_NO_ERROR();
}


static LIGHTMODBUS_RET_ERROR random_serial_number_response(ModbusMaster *master, uint8_t address, uint8_t function,
                                                           const uint8_t *requestPDU, uint8_t requestLength,
                                                           const uint8_t *responsePDU, uint8_t responseLength) {
    // Check lengths
    if (requestLength != 3) {
        return MODBUS_REQUEST_ERROR(LENGTH);
    }
    if (responseLength != 3) {
        return MODBUS_RESPONSE_ERROR(LENGTH);
    }

    device_map_context_t *ctx = modbusMasterGetUserPointer(master);

    uint32_t serial_number   = (responsePDU[1] << 24) | (responsePDU[2] << 16) | (responsePDU[3] << 8) | responsePDU[4];
    ctx->device_map[address] = serial_number;

    ESP_LOGI(TAG, "Received serial number %" PRIdLEAST32 "from %" PRIdLEAST16, serial_number, address);

    return MODBUS_NO_ERROR();
}


static void send_custom_function(ModbusMaster *master, uint8_t address, uint8_t function, uint8_t *data, size_t len) {
    ModbusErrorInfo err = modbusBeginRequestRTU(master);
    assert(modbusIsOk(err));
    err = build_custom_request(master, function, data, len);
    assert(modbusIsOk(err));
    err = modbusEndRequestRTU(master, MODBUS_BROADCAST_ADDRESS);
    assert(modbusIsOk(err));
    /* Broadcast message, we expect no answer */
    rs485_write(modbusMasterGetRequest(master), modbusMasterGetRequestLength(master));
    vTaskDelay(pdMS_TO_TICKS(MODBUS_TIMEOUT));
}


static int write_holding_registers(ModbusMaster *master, uint8_t address, uint16_t starting_address, uint16_t *data,
                                   size_t num) {
    uint8_t buffer[MODBUS_MAX_PACKET_SIZE] = {0};
    int     res                            = 0;
    size_t  counter                        = 0;

    do {
        res                 = 0;
        ModbusErrorInfo err = modbusBuildRequest16RTU(master, address, starting_address, num, data);
        assert(modbusIsOk(err));
        rs485_flush();
        rs485_write(modbusMasterGetRequest(master), modbusMasterGetRequestLength(master));

        int len = rs485_read(buffer, sizeof(buffer), pdMS_TO_TICKS(MODBUS_TIMEOUT));
        err     = modbusParseResponseRTU(master, modbusMasterGetRequest(master), modbusMasterGetRequestLength(master),
                                         buffer, len);

        if (!modbusIsOk(err)) {
            ESP_LOGW(TAG, "Write holding registers for %i error: %i %i", address, err.source, err.error);
            res = 1;
            vTaskDelay(pdMS_TO_TICKS(MODBUS_TIMEOUT));
        }
    } while (res && counter++ < MODBUS_COMMUNICATION_ATTEMPTS);

    return res;
}


static int write_holding_register(ModbusMaster *master, uint8_t address, uint16_t index, uint16_t data) {
    return write_holding_registers(master, address, index, &data, 1);
}


static int write_coils(ModbusMaster *master, uint8_t address, uint16_t index, size_t num_values, uint8_t *values) {
    uint8_t buffer[MODBUS_RESPONSE_05_LEN] = {0};
    int     res                            = 0;
    size_t  counter                        = 0;

    do {
        ModbusErrorInfo err = modbusBuildRequest15RTU(master, address, index, num_values, values);
        assert(modbusIsOk(err));
        rs485_flush();
        rs485_write(modbusMasterGetRequest(master), modbusMasterGetRequestLength(master));

        int len = rs485_read(buffer, sizeof(buffer), pdMS_TO_TICKS(MODBUS_TIMEOUT));
        err     = modbusParseResponseRTU(master, modbusMasterGetRequest(master), modbusMasterGetRequestLength(master),
                                         buffer, len);

        if (!modbusIsOk(err)) {
            ESP_LOGW(TAG, "Write coil for %i error: %i %i", address, err.source, err.error);
            res = 1;
            vTaskDelay(pdMS_TO_TICKS(MODBUS_TIMEOUT));
        }
    } while (res && counter++ < MODBUS_COMMUNICATION_ATTEMPTS);

    return res;
}


static int read_holding_registers(ModbusMaster *master, uint16_t *registers, uint8_t address, uint16_t start,
                                  uint16_t count) {
    ModbusErrorInfo err;
    uint8_t         buffer[MODBUS_RESPONSE_03_LEN(count)];
    int             res     = 0;
    size_t          counter = 0;

    master_context_t ctx = {.pointer = registers, .start = start};
    if (registers == NULL) {
        modbusMasterSetUserPointer(master, NULL);
    } else {
        modbusMasterSetUserPointer(master, &ctx);
    }

    do {
        res = 0;
        err = modbusBuildRequest03RTU(master, address, start, count);
        assert(modbusIsOk(err));
        rs485_flush();
        rs485_write(modbusMasterGetRequest(master), modbusMasterGetRequestLength(master));

        int len = rs485_read(buffer, sizeof(buffer), pdMS_TO_TICKS(MODBUS_TIMEOUT));
        err     = modbusParseResponseRTU(master, modbusMasterGetRequest(master), modbusMasterGetRequestLength(master),
                                         buffer, len);

        if (!modbusIsOk(err)) {
            ESP_LOGW(TAG, "Read holding registers for %i error %zu: %i %i", address, counter, err.source, err.error);
            if (len == 0) {
                ESP_LOGW(TAG, "Empty packet!");
            } else {
                ESP_LOG_BUFFER_HEX(TAG, buffer, len);
            }
            res = 1;
            vTaskDelay(pdMS_TO_TICKS(MODBUS_TIMEOUT));
        }
    } while (res && counter++ < MODBUS_COMMUNICATION_ATTEMPTS);

    return res;
}
