#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "model/model.h"
#include "controller/controller.h"
#include "bsp/interface.h"
#include "bsp/safety.h"
#include "bsp/rs485.h"


static const char *TAG = "Main";


void app_main(void) {
    mut_model_t model;

    safety_init();
    interface_init();
    rs485_init();

    model_init(&model);
    controller_init(&model);

    ESP_LOGI(TAG, "Begin main loop");
    for (;;) {
        controller_manage(&model);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
