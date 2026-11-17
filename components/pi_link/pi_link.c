#include "pi_link.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/uart_pins.h"

#define TAG "pi_link"

#define PI_LINK_UART UART_NUM_0
#define PI_LINK_RX_BUFFER_SIZE 256  // Driver needs more than the 128 byte FIFO
#define PI_LINK_LINE_MAX 64         // ACL_CMD_SIZE in operator acl.hpp
#define PI_LINK_READ_TIMEOUT_MS 10  // erv sends manual keys every 10 ms
// Stop if erv's key repeat stops. A plain jog repeats every 10 ms, a tracking
// jog (TrackingJog in operator api.hpp) every interval_ms, a uint8_t, so 255 ms
// at most. erv stops with ~ itself, so this only catches a lost link
#define PI_LINK_KEY_TIMEOUT_MS 300
#define PI_LINK_RESTART_MS 250  // Repeat start below CONFIG_DRIVER_MAX_RUN_MS

#define ACL_MANUAL_TOGGLE '~'
#define ACL_XON 0x11   // The Pi's tty uses XON/XOFF flow control (ixon ixoff)
#define ACL_XOFF 0x13
#define ACL_PROMPT '>'
#define ACL_CTRL_C 0x03  // erv aborts a LISTPV reply with Ctrl-C

static pi_link_config_t s_config;
static bool s_manual_mode = false;
static char s_line[PI_LINK_LINE_MAX];
static size_t s_line_len = 0;
static bool s_line_bad = false;
static uint8_t s_active_key = 0;  // Key driving the motors, 0 when stopped
static TickType_t s_last_key_tick;
static TickType_t s_last_start_tick;
static volatile bool s_homing = false;  // Cleared by home_task when done

static void reply(const char* text) {
  if (!s_config.debug_reply) return;
  uart_write_bytes(PI_LINK_UART, text, strlen(text));
}

// erv sends its next queued command, and enters manual mode, only after the
// controller's > prompt (Scorbot::poll in operator erv.cpp). Like the ER-V
// controller, prompt after every command line, after leaving manual mode,
// after Ctrl-C and at boot
static void prompt(void) {
  const char p = ACL_PROMPT;
  uart_write_bytes(PI_LINK_UART, &p, 1);
}

// Manual mode keys erv can send (ACL_Manual_Axis in operator acl_private.hpp).
// Which key of each pair is + differs between operator versions; the build on
// the Pi image is the reference, so check directions on the bench
static bool is_manual_key(uint8_t byte) {
  switch (byte) {
    case '1':
    case 'q':  // base
    case '2':
    case 'w':  // shoulder
    case '3':
    case 'e':  // elbow
    case '4':
    case 'r':  // wrist pitch
    case '5':
    case 't':  // wrist roll
      return true;
    default:
      return false;
  }
}

// Checked on the bench against the Pi image's erv:
// RIGHT 'q', LEFT '1', UP '4', DOWN 'r'
static bool key_to_deltas(uint8_t key, int8_t* azimuth, int8_t* altitude) {
  *azimuth = 0;
  *altitude = 0;
  switch (key) {
    case 'q':
      *azimuth = 1;
      return true;
    case '1':
      *azimuth = -1;
      return true;
    case '4':
      *altitude = 1;
      return true;
    case 'r':
      *altitude = -1;
      return true;
    default:
      return false;  // Shoulder, elbow and roll are not driven
  }
}

static void stop_motion(void) {
  s_active_key = 0;
  // Homing stops the motors itself, and a stop landing as it starts could
  // brake the axis it is driving
  if (s_homing) return;
  s_config.motor.polar_pan_stop();
}

static void handle_manual_toggle(void) {
  // Nothing should be moving on entering or leaving manual mode
  stop_motion();
  reply(s_manual_mode ? "manual on\r\n" : "manual off\r\n");
  if (!s_manual_mode) prompt();
}

static void handle_manual_key(uint8_t key) {
  char text[16];
  snprintf(text, sizeof(text), "key %c\r\n", key);
  reply(text);

  int8_t azimuth, altitude;
  if (!key_to_deltas(key, &azimuth, &altitude)) {
    if (s_active_key) stop_motion();
    return;
  }
  if (s_homing) return;

  TickType_t now = xTaskGetTickCount();
  s_last_key_tick = now;

  // Start on a new key, and repeat it while the key is held: each start
  // renews the max run check and re-checks the soft limits
  if (key != s_active_key ||
      now - s_last_start_tick >= pdMS_TO_TICKS(PI_LINK_RESTART_MS)) {
    s_last_start_tick = now;
    if (s_config.motor.polar_pan_start(azimuth, altitude) == ESP_OK) {
      s_active_key = key;
    } 
    else {
      s_active_key = 0;  // Retried on the next key
    }
  }
}

// Called every read, so a key stream that stops also stops the motors
static void check_key_timeout(void) {
  if (s_active_key && xTaskGetTickCount() - s_last_key_tick > pdMS_TO_TICKS(PI_LINK_KEY_TIMEOUT_MS)) {
    stop_motion();
    reply("key timeout\r\n");
  }
}

// motorhat_home blocks until done, so it runs here to keep the link read
static void home_task(void* arg) {
  s_config.motor.home(0);
  s_homing = false;
  prompt();  // HOME's prompt comes when homing is over, as on the ER-V
  vTaskDelete(NULL);
}

// Returns true when home_task will send the prompt
static bool start_homing(void) {
  if (s_homing) return true;
  stop_motion();
  s_homing = true;
  if (xTaskCreate(home_task, "pi_link_home", 4096, NULL, 5, NULL) != pdPASS) {
    s_homing = false;
  }
  return s_homing;
}

// Returns true when the prompt should be sent now
static bool handle_line(const char* line) {
  char text[PI_LINK_LINE_MAX + 16];
  snprintf(text, sizeof(text), "line [%s]\r\n", line);
  reply(text);

  // Exact matches only. Anything else (HERE, DEFP, SHIFT, MOVE, LISTPV, CON,
  // SPEED) only gets a prompt until relative moves exist (motorhat_polar_pan is a TODO)
  if (strcmp(line, "HOME") == 0) {
    return !start_homing();
  } else if (strcmp(line, "clrbuf") == 0) {
    stop_motion();
  }
  return true;
}

static void reset_line(void) {
  s_line_len = 0;
  s_line_bad = false;
}

static void handle_byte(uint8_t byte) {
  // Flow control from the Pi, not part of any command
  if (byte == ACL_XON || byte == ACL_XOFF) return;

  if (byte == ACL_CTRL_C) {
    reset_line();
    if (!s_manual_mode) prompt();
    return;
  }

  if (byte == ACL_MANUAL_TOGGLE) {
    reset_line();  // erv only sends ~ between whole commands
    s_manual_mode = !s_manual_mode;
    handle_manual_toggle();
    return;
  }

  if (s_manual_mode) {
    if (is_manual_key(byte)) {
      handle_manual_key(byte);
      return;
    }
    // In manual mode erv only sends keys and ~. Anything else means a ~ was
    // missed (e.g. this board rebooted mid pan), so leave manual mode and
    // read the byte as the start of a command line
    s_manual_mode = false;
    handle_manual_toggle();
  }

  if (byte == '\r' || byte == '\n') {
    // A bare CR gets a prompt (erv sends one at start up); a bare LF doesn't,
    // so a CR LF from a terminal is one prompt
    bool prompt_now = byte == '\r' || s_line_len > 0;
    if (s_line_len > 0 && !s_line_bad) {
      s_line[s_line_len] = '\0';
      prompt_now = handle_line(s_line);
    }
    reset_line();
    if (prompt_now) prompt();
    return;
  }

  if (s_line_bad) return;
  // Drop the whole line rather than run part of a corrupted or long one
  if (!isprint(byte) || s_line_len >= PI_LINK_LINE_MAX - 1) {
    s_line_bad = true;
    return;
  }
  s_line[s_line_len++] = (char)byte;
}

static void pi_link_task(void* arg) {
  uint8_t rx[32];

  while (1) {
    int len = uart_read_bytes(PI_LINK_UART, rx, sizeof(rx), pdMS_TO_TICKS(PI_LINK_READ_TIMEOUT_MS));
    if (len < 0) {
      ESP_LOGE(TAG, "UART read failed");
      len = 0;  // Still run the key timeout below
    }

    for (int i = 0; i < len; i++) {
      handle_byte(rx[i]);
    }
    check_key_timeout();
  }
}

esp_err_t pi_link_init(const pi_link_config_t* config) {
  if (!config || !config->motor.polar_pan_start ||
      !config->motor.polar_pan_stop || !config->motor.home) {
    return ESP_ERR_INVALID_ARG;
  }
  s_config = *config;

  uart_config_t uart_config = {
      .baud_rate = config->baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t err = uart_driver_install(PI_LINK_UART, PI_LINK_RX_BUFFER_SIZE, 0, 0, NULL, 0);
  if (err != ESP_OK) return err;

  err = uart_param_config(PI_LINK_UART, &uart_config);
  if (err != ESP_OK) return err;

  // With console output set to None the bootloader detaches UART0 from its
  // pins, so connect it back to GPIO1/3, which the dev board wires to USB
  err = uart_set_pin(PI_LINK_UART, U0TXD_GPIO_NUM, U0RXD_GPIO_NUM,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) return err;

  // Drop anything that arrived while booting
  err = uart_flush_input(PI_LINK_UART);
  if (err != ESP_OK) return err;

  // The ROM boot message goes out at 115200 and reads as random bytes at
  // 9600. One of them can look like XOFF and pause everything the Pi sends,
  // stops included, so tell the Pi to resume
  const char xon = ACL_XON;
  if (uart_write_bytes(PI_LINK_UART, &xon, 1) < 0) return ESP_FAIL;

  // An erv that started before this board is waiting for a prompt
  prompt();

  if (xTaskCreate(pi_link_task, "pi_link_task", 4096, NULL, 5, NULL) !=
      pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}
