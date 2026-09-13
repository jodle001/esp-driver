#include "nvs_store.h"

#include <stdio.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "pb_decode.h"
#include "pb_encode.h"

static const char *TAG = "nvs_store";
#define NS "scorbot"

esp_err_t nvs_store_init(uint32_t *boot_count)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }
    nvs_handle_t h;
    if ((err = nvs_open(NS, NVS_READWRITE, &h)) != ESP_OK) {
        return err;
    }
    uint32_t count = 0;
    nvs_get_u32(h, "boot_count", &count); /* missing = 0 */
    ++count;
    err = nvs_set_u32(h, "boot_count", count);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    *boot_count = count;
    ESP_LOGI(TAG, "boot %lu", (unsigned long)count);
    return err;
}

bool nvs_store_load_robot_type(char *out, size_t cap)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = cap;
    const bool ok = nvs_get_str(h, "robot_type", out, &len) == ESP_OK;
    nvs_close(h);
    return ok;
}

esp_err_t nvs_store_save_robot_type(const char *robot_type)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, "robot_type", robot_type);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* Calibration is stored as its protobuf encoding, so a firmware update that adds
   fields keeps reading old entries. */
bool nvs_store_load_calibration(uint32_t joint, scorbot_v1_JointCalibration *out)
{
    char key[8];
    snprintf(key, sizeof(key), "cal%lu", (unsigned long)joint);
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t buf[scorbot_v1_JointCalibration_size + 8];
    size_t len = sizeof(buf);
    bool ok = nvs_get_blob(h, key, buf, &len) == ESP_OK;
    nvs_close(h);
    if (ok) {
        pb_istream_t in = pb_istream_from_buffer(buf, len);
        const scorbot_v1_JointCalibration zero = scorbot_v1_JointCalibration_init_zero;
        *out = zero;
        ok = pb_decode(&in, scorbot_v1_JointCalibration_fields, out) && out->index == joint;
    }
    return ok;
}

esp_err_t nvs_store_save_calibration(const scorbot_v1_JointCalibration *cal)
{
    uint8_t buf[scorbot_v1_JointCalibration_size + 8];
    pb_ostream_t out = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&out, scorbot_v1_JointCalibration_fields, cal)) {
        return ESP_FAIL;
    }
    char key[8];
    snprintf(key, sizeof(key), "cal%lu", (unsigned long)cal->index);
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, key, buf, out.bytes_written);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}
