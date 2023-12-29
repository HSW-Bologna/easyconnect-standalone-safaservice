#include <assert.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define COMPATIBILITY_KEY     "COMPATIBILITY"
#define COMPATIBILITY_VERSION 1


static const char *TAG = "Storage";


void storage_init(void) {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &handle));
    uint8_t buf;
    err = nvs_get_u8(handle, COMPATIBILITY_KEY, &buf);

    if (err == ESP_OK) {
        if (buf != COMPATIBILITY_VERSION) {
            ESP_LOGI(TAG,
                     "The previously saved configuration is not compatibile with the new firmware version; erasing...");
            nvs_close(handle);
            ESP_ERROR_CHECK(nvs_erase_all(handle));
            ESP_ERROR_CHECK(nvs_set_u8(handle, COMPATIBILITY_KEY, COMPATIBILITY_VERSION));
            ESP_ERROR_CHECK(nvs_commit(handle));
        }
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_set_u8(handle, COMPATIBILITY_KEY, COMPATIBILITY_VERSION));
        ESP_ERROR_CHECK(nvs_commit(handle));
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Storage initialized!");
}


int storage_load_uint8(uint8_t *value, char *key) {
    nvs_handle_t handle;
    esp_err_t    err;
    assert(strlen(key) <= 15);

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &handle));
    err = nvs_get_u8(handle, key, value);
    nvs_close(handle);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS error (%s) while reading %s", esp_err_to_name(err), key);
        return -1;
    }

    return 0;
}


void storage_save_uint8(uint8_t *value, char *key) {
    nvs_handle_t handle;
    assert(strlen(key) <= 15);

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%i) opening NVS handle!\n", err);
        return;
    }

    err = nvs_set_u8(handle, key, *value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS error (%i) while writing %s", err, key);
    } else {
        ESP_ERROR_CHECK(nvs_commit(handle));
        nvs_close(handle);
    }
}


int storage_load_uint16(uint16_t *value, char *key) {
    nvs_handle_t handle;
    esp_err_t    err;
    assert(strlen(key) <= 15);

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &handle));
    err = nvs_get_u16(handle, key, value);
    nvs_close(handle);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS error (%s) while reading %s", esp_err_to_name(err), key);
        return -1;
    }

    return 0;
}


void storage_save_uint16(uint16_t *value, char *key) {
    nvs_handle_t handle;
    ESP_LOGI(TAG, "Trying to save key %s", key);
    assert(strlen(key) <= 15);

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%i) opening NVS handle!\n", err);
        return;
    }

    err = nvs_set_u16(handle, key, *value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS error (%i) while writing %s", err, key);
    } else {
        ESP_ERROR_CHECK(nvs_commit(handle));
        nvs_close(handle);
    }
}


int storage_load_uint32(uint32_t *value, char *key) {
    nvs_handle_t handle;
    esp_err_t    err;
    assert(strlen(key) <= 15);

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &handle));
    err = nvs_get_u32(handle, key, value);
    nvs_close(handle);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS error (%s) while reading %s", esp_err_to_name(err), key);
        return -1;
    }

    return 0;
}


void storage_save_uint32(uint32_t *value, char *key) {
    nvs_handle_t handle;
    ESP_LOGI(TAG, "Trying to save key %s", key);
    assert(strlen(key) <= 15);

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%i) opening NVS handle!\n", err);
        return;
    }

    err = nvs_set_u32(handle, key, *value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS error (%i) while writing %s", err, key);
    } else {
        ESP_ERROR_CHECK(nvs_commit(handle));
        nvs_close(handle);
    }
}


int storage_load_uint64(uint64_t *value, char *key) {
    nvs_handle_t handle;
    esp_err_t    err;
    assert(strlen(key) <= 15);

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &handle));
    err = nvs_get_u64(handle, key, value);
    nvs_close(handle);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS error (%s) while reading %s", esp_err_to_name(err), key);
        return -1;
    }

    return 0;
}


void storage_save_uint64(uint64_t *value, char *key) {
    nvs_handle_t handle;
    ESP_LOGI(TAG, "Trying to save key %s", key);
    assert(strlen(key) <= 15);

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%i) opening NVS handle!\n", err);
        return;
    }

    err = nvs_set_u64(handle, key, *value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS error (%i) while writing %s", err, key);
    } else {
        ESP_ERROR_CHECK(nvs_commit(handle));
        nvs_close(handle);
    }
}


int storage_load_blob(void *value, size_t len, char *key) {
    nvs_handle_t handle;
    esp_err_t    err;
    assert(strlen(key) <= 15);

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &handle));
    err = nvs_get_blob(handle, key, value, &len);
    nvs_close(handle);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS error (%s) while reading %s", esp_err_to_name(err), key);
        return -1;
    }

    return 0;
}


void storage_save_blob(void *value, size_t len, char *key) {
    nvs_handle_t handle;
    ESP_LOGI(TAG, "Trying to save key %s", key);
    assert(strlen(key) <= 15);

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%i) opening NVS handle!\n", err);
        return;
    }

    err = nvs_set_blob(handle, key, value, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS error (%i) while writing %s", err, key);
    } else {
        ESP_ERROR_CHECK(nvs_commit(handle));
        nvs_close(handle);
    }
}
