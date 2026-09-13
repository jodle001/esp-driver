// What survives a reboot: the boot counter, the robot type and the per-joint
// calibration the Pi pushed. Nothing robot-specific is compiled in.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "scorbot_proto/envelope.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Mount NVS (erasing it if the layout changed) and increment + return the boot count.
esp_err_t nvs_store_init(uint32_t *boot_count);
bool nvs_store_load_robot_type(char *out, size_t cap);
esp_err_t nvs_store_save_robot_type(const char *robot_type);
bool nvs_store_load_calibration(uint32_t joint, scorbot_v1_JointCalibration *out);
esp_err_t nvs_store_save_calibration(const scorbot_v1_JointCalibration *cal);

#ifdef __cplusplus
}
#endif
