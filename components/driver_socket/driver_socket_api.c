#include "driver_socket_api.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "endian.h"
#include "esp_log.h"
#include "freertos//FreeRTOS.h"

#define TAG "driver_socket_api"

static driver_socket_api_motor_interface_t s_motor_interface;

static void driver_socket_api_header_swap_endianess(
    driver_socket_api_header_t* header) {
  header->msg_id = be32toh(header->msg_id);
  header->cmd_id = be16toh(header->cmd_id);
  header->len = be16toh(header->len);
}

static void driver_socket_api_polar_pan_payload_swap_endianess(
    driver_socket_api_polar_pan_payload_t* payload) {
  payload->delta_azimuth = be32toh(payload->delta_azimuth);
  payload->delta_altitude = be32toh(payload->delta_altitude);
  payload->delay_ms = be32toh(payload->delay_ms);
  payload->time_ms = be32toh(payload->time_ms);
}

static void driver_socket_api_home_payload_swap_endianess(
    driver_socket_api_home_payload_t* payload) {
  payload->delay_ms = be32toh(payload->delay_ms);
}

static uint8_t xor_checksum(uint8_t* data, size_t len) {
  uint8_t result = 0;
  for (size_t i = 0; i < len; i++) {
    result ^= data[i];
  }
  return result;
}

static esp_err_t validate_checksum(uint8_t* data, size_t len,
                                   uint8_t checksum) {
  uint8_t calculated_checksum = xor_checksum((uint8_t*)data, len);
  return (calculated_checksum == checksum) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

esp_err_t driver_socket_api_init(
    const driver_socket_api_motor_interface_t* motor_interface) {
  if (motor_interface == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  s_motor_interface = *motor_interface;
  return ESP_OK;
}

// Largest payload accepted.  POLAR_PAN is the biggest command at 16 bytes.
// A length above this means the stream has lost its place.
#define DRIVER_SOCKET_API_MAX_PAYLOAD 64
#define DRIVER_SOCKET_API_HEADER_SIZE sizeof(driver_socket_api_header_t)

// Bytes received but not yet processed. TCP is a byte stream, so one recv()
// can hold several frames (a START and STOP sent close together) or end
// partway through one. Frames are cut out using the header's length field.
static uint8_t s_pending[256];
static size_t s_pending_len = 0;

static bool payload_fits(const driver_socket_api_header_t* header,
                         size_t needed) {
  if (header->len < needed) {
    ESP_LOGE(TAG, "Command %d payload too short: %d of %d bytes",
             header->cmd_id, header->len, (int)needed);
    return false;
  }
  return true;
}

static esp_err_t driver_socket_api_process_frame(uint8_t* frame,
                                                 size_t frame_size) {
  esp_err_t err =
      validate_checksum(frame, frame_size - 1, frame[frame_size - 1]);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Checksum validation failed");
    return err;
  }

  driver_socket_api_wrapper_t* wrapper = (driver_socket_api_wrapper_t*)frame;

  driver_socket_api_header_swap_endianess(&wrapper->header);

  ESP_LOGI(TAG, "Parsed header: msg_id=%d, cmd_id=%d, len=%d",
           (int)wrapper->header.msg_id, wrapper->header.cmd_id,
           wrapper->header.len);

  switch (wrapper->header.cmd_id) {
    case DRIVER_SOCKET_API_CMD_ID_HANDSHAKE:
      ESP_LOGI(TAG, "Processing HANDSHAKE command");
      break;
    case DRIVER_SOCKET_API_CMD_ID_POLAR_PAN:
      if (!payload_fits(&wrapper->header,
                        sizeof(driver_socket_api_polar_pan_payload_t))) {
        return ESP_ERR_INVALID_SIZE;
      }
      driver_socket_api_polar_pan_payload_t* polar_pan_payload =
          (driver_socket_api_polar_pan_payload_t*)&wrapper->payload_head;
      driver_socket_api_polar_pan_payload_swap_endianess(polar_pan_payload);
      s_motor_interface.polar_pan(
          polar_pan_payload->delta_azimuth, polar_pan_payload->delta_altitude,
          polar_pan_payload->delay_ms, polar_pan_payload->time_ms);
      break;
    case DRIVER_SOCKET_API_CMD_ID_HOME:
      if (!payload_fits(&wrapper->header,
                        sizeof(driver_socket_api_home_payload_t))) {
        return ESP_ERR_INVALID_SIZE;
      }
      driver_socket_api_home_payload_t* home_payload =
          (driver_socket_api_home_payload_t*)&wrapper->payload_head;
      driver_socket_api_home_payload_swap_endianess(home_payload);
      s_motor_interface.home(home_payload->delay_ms);
      break;
    case DRIVER_SOCKET_API_CMD_ID_POLAR_PAN_START:
      if (!payload_fits(&wrapper->header,
                        sizeof(driver_socket_api_polar_pan_start_payload_t))) {
        return ESP_ERR_INVALID_SIZE;
      }
      driver_socket_api_polar_pan_start_payload_t* polar_pan_start_payload =
          (driver_socket_api_polar_pan_start_payload_t*)&wrapper->payload_head;
      // No endianess swap needed for int8_t fields
      s_motor_interface.polar_pan_start(
          polar_pan_start_payload->delta_azimuth,
          polar_pan_start_payload->delta_altitude);
      break;
    case DRIVER_SOCKET_API_CMD_ID_POLAR_PAN_STOP:
      s_motor_interface.polar_pan_stop();
      break;
    default:
      ESP_LOGW(TAG, "Unknown command ID: %d", wrapper->header.cmd_id);
      return ESP_ERR_INVALID_ARG;
  }

  return ESP_OK;
}

esp_err_t driver_socket_api_process(uint8_t* buffer, size_t buffer_size) {
  ESP_LOGI(TAG, "Received data size: %d", (int)buffer_size);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, buffer_size, ESP_LOG_INFO);

  if (buffer_size > sizeof(s_pending) - s_pending_len) {
    ESP_LOGE(TAG, "Receive buffer full, dropping %d pending bytes",
             (int)s_pending_len);
    s_pending_len = 0;
    if (buffer_size > sizeof(s_pending)) {
      return ESP_ERR_INVALID_SIZE;
    }
  }
  memcpy(&s_pending[s_pending_len], buffer, buffer_size);
  s_pending_len += buffer_size;

  esp_err_t result = ESP_OK;
  size_t offset = 0;
  while (s_pending_len - offset >= DRIVER_SOCKET_API_HEADER_SIZE + 1) {
    // The length field is big endian, read it without swapping in place
    const uint8_t* len_bytes =
        &s_pending[offset + offsetof(driver_socket_api_header_t, len)];
    size_t payload_len = ((size_t)len_bytes[0] << 8) | len_bytes[1];
    if (payload_len > DRIVER_SOCKET_API_MAX_PAYLOAD) {
      ESP_LOGE(TAG, "Invalid payload length %d, dropping %d bytes",
               (int)payload_len, (int)(s_pending_len - offset));
      s_pending_len = 0;
      return ESP_ERR_INVALID_SIZE;
    }

    size_t frame_size = DRIVER_SOCKET_API_HEADER_SIZE + payload_len + 1;
    if (s_pending_len - offset < frame_size) {
      break;  // The rest of this frame has not arrived yet
    }

    esp_err_t err =
        driver_socket_api_process_frame(&s_pending[offset], frame_size);
    if (err != ESP_OK) {
      result = err;
    }
    offset += frame_size;
  }

  // Keep any partial frame for next call
  memmove(s_pending, &s_pending[offset], s_pending_len - offset);
  s_pending_len -= offset;

  return result;
}

void driver_socket_api_client_closed(void) {
  ESP_LOGW(TAG, "Client disconnected, stopping motors");
  s_pending_len = 0;
  if (s_motor_interface.polar_pan_stop != NULL) {
    s_motor_interface.polar_pan_stop();
  }
}
