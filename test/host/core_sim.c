// core_sim: the real firmware core driving the host plant model behind a pseudo
// terminal, so scorbot_protocol_cli and the ROS hardware interface can talk to it the
// way they talk to a real controller (and to scorbot_esp_sim).
//
//   core_sim                          # prints the pty path
//   core_sim --link /tmp/scorbot_core # also creates a stable symlink to it
//   core_sim --robot er_v --speed 2.0 --no-calibration
//
// Then: scorbot_protocol_cli --port /tmp/scorbot_core ping | info | home | monitor 3
// or:   ros2 launch scorbot_bringup robot.launch.py mock:=false serial_port:=/tmp/scorbot_core
//
// Commands on stdin while running:
//   status | power on|off (the e-stop) | fault <driver|encoder|...> [joint] | reboot | quit
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "plant.h"
#include "scorbot_core/core.h"
#include "scorbot_proto/envelope.h"
#include "scorbot_proto/framing.h"

#define N 5

typedef struct {
    sc_core_t core;
    plant_t plant;
    sc_core_config_t cfg;
    char robot_type[SC_CORE_ROBOT_TYPE_LEN];
    bool has_cal[SC_CORE_MAX_JOINTS];
    scorbot_v1_JointCalibration cal[SC_CORE_MAX_JOINTS];
    uint32_t boot_count;
    bool require_calibration;
    int master;
    unsigned long rx_frames, tx_frames, tx_errors;
} app_t;

static const char *kNames[N] = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"};

static void persist_cal(void *ctx, const scorbot_v1_JointCalibration *cal)
{
    app_t *a = (app_t *)ctx;
    a->cal[cal->index] = *cal;
    a->has_cal[cal->index] = true;
}
static void persist_type(void *ctx, const char *t) { strncpy(((app_t *)ctx)->robot_type, t, SC_CORE_ROBOT_TYPE_LEN - 1); }

static void boot_core(app_t *a)
{
    sc_core_config_default(&a->cfg);
    a->cfg.joint_count = N;
    for (int i = 0; i < N; ++i) {
        a->cfg.joint_names[i] = kNames[i];
        a->cfg.has_calibration[i] = a->has_cal[i];
        a->cfg.calibration[i] = a->cal[i];
    }
    strncpy(a->cfg.robot_type, a->robot_type, sizeof(a->cfg.robot_type) - 1);
    a->cfg.boot_count = a->boot_count;
    a->cfg.persist.ctx = a;
    a->cfg.persist.calibration = persist_cal;
    a->cfg.persist.robot_type = persist_type;
    a->cfg.firmware_version = "core_sim-0.1.0";
    a->cfg.require_calibration = a->require_calibration;
    sc_core_init(&a->core, &a->cfg);
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void on_payload(void *ctx, const uint8_t *payload, size_t n)
{
    app_t *a = (app_t *)ctx;
    scorbot_v1_Envelope e;
    if (sc_envelope_decode(payload, n, &e)) {
        ++a->rx_frames;
        sc_core_handle(&a->core, &e);
    }
}

static scorbot_v1_FaultCode parse_fault(const char *s)
{
    static const struct { const char *name; scorbot_v1_FaultCode code; } table[] = {
        {"watchdog", scorbot_v1_FaultCode_FAULT_WATCHDOG}, {"overcurrent", scorbot_v1_FaultCode_FAULT_OVERCURRENT},
        {"limit_switch", scorbot_v1_FaultCode_FAULT_LIMIT_SWITCH}, {"soft_limit", scorbot_v1_FaultCode_FAULT_SOFT_LIMIT},
        {"stall", scorbot_v1_FaultCode_FAULT_STALL}, {"homing_timeout", scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT},
        {"encoder", scorbot_v1_FaultCode_FAULT_ENCODER}, {"driver", scorbot_v1_FaultCode_FAULT_DRIVER},
        {"internal", scorbot_v1_FaultCode_FAULT_INTERNAL},
    };
    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); ++i) {
        if (strcmp(s, table[i].name) == 0) {
            return table[i].code;
        }
    }
    return scorbot_v1_FaultCode_FAULT_NONE;
}

static void print_status(const app_t *a)
{
    printf("state=%s fault=%s boot=%u wd=%ums rx_cmd=%u ign=%u | rx=%lu tx=%lu tx_err=%lu power=%s\n",
           sc_state_name(a->core.state), sc_fault_name(a->core.fault), a->cfg.boot_count, a->core.watchdog_ms,
           a->core.commands_received, a->core.commands_ignored, a->rx_frames, a->tx_frames, a->tx_errors,
           a->plant.motor_power ? "on" : "OFF");
    for (int i = 0; i < N; ++i) {
        const plant_joint_t *p = &a->plant.joint[i];
        const sc_joint_t *j = &a->core.joint[i];
        printf("  j%d true=%+.3f rep=%+.3f vel=%+.3f duty=%+.2f cur=%.2f%s%s%s\n", i, p->true_position, j->position,
               p->velocity, j->duty, p->current, j->homed ? " homed" : "", p->limit_switch ? " SWITCH" : "",
               j->fault ? " FAULT" : "");
    }
    fflush(stdout);
}

static bool handle_command(app_t *a, char *line)
{
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) {
        return true;
    }
    if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
        return false;
    }
    if (strcmp(cmd, "status") == 0) {
        print_status(a);
    } else if (strcmp(cmd, "reboot") == 0) {
        ++a->boot_count;
        plant_reset_counters(&a->plant);
        boot_core(a);
        printf("rebooted (boot_count=%u)\n", a->boot_count);
    } else if (strcmp(cmd, "power") == 0) {
        const char *arg = strtok(NULL, " \t\r\n");
        a->plant.motor_power = !(arg && strcmp(arg, "off") == 0);
        printf("motor power %s\n", a->plant.motor_power ? "on" : "off");
    } else if (strcmp(cmd, "fault") == 0) {
        const char *name = strtok(NULL, " \t\r\n");
        const char *joint = strtok(NULL, " \t\r\n");
        const scorbot_v1_FaultCode code = name ? parse_fault(name) : scorbot_v1_FaultCode_FAULT_NONE;
        if (code == scorbot_v1_FaultCode_FAULT_NONE) {
            printf("unknown fault '%s'\n", name ? name : "");
        } else {
            sc_core_raise_fault(&a->core, code, joint ? atoi(joint) : -1, false);
            printf("injected %s\n", sc_fault_name(code));
        }
    } else {
        printf("commands: status | power on|off | fault <name> [joint] | reboot | quit\n");
    }
    fflush(stdout);
    return true;
}

int main(int argc, char **argv)
{
    app_t a;
    memset(&a, 0, sizeof(a));
    const char *link_path = NULL;
    double speed = 1.0;
    bool quiet = false;
    strcpy(a.robot_type, "er_4pc");
    a.boot_count = 1;
    for (int i = 0; i < N; ++i) {
        a.cal[i] = plant_default_calibration((uint32_t)i);
        a.has_cal[i] = true;
    }
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--link") == 0 && i + 1 < argc) {
            link_path = argv[++i];
        } else if (strcmp(argv[i], "--robot") == 0 && i + 1 < argc) {
            strncpy(a.robot_type, argv[++i], sizeof(a.robot_type) - 1);
        } else if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc) {
            speed = atof(argv[++i]);
        } else if (strcmp(argv[i], "--quiet") == 0) {
            quiet = true;
        } else if (strcmp(argv[i], "--no-calibration") == 0) {
            /* Like a fresh device: nothing in NVS, the Pi must push calibration first. */
            for (int k = 0; k < N; ++k) {
                a.has_cal[k] = false;
            }
            a.require_calibration = true;
        } else {
            fprintf(stderr, "usage: core_sim [--link PATH] [--robot er_4pc|er_v] [--speed X] [--no-calibration] [--quiet]\n");
            return strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0 ? 0 : 64;
        }
    }
    plant_init(&a.plant, N, a.cal, NULL, NULL);
    boot_core(&a);

    int master = -1, slave = -1;
    char name[128] = {0};
    if (openpty(&master, &slave, name, NULL, NULL) != 0) {
        perror("openpty");
        return 2;
    }
    struct termios tio;
    if (tcgetattr(slave, &tio) == 0) {
        cfmakeraw(&tio);
        tcsetattr(slave, TCSANOW, &tio);
    }
    fcntl(master, F_SETFL, fcntl(master, F_GETFL, 0) | O_NONBLOCK);
    if (link_path) {
        unlink(link_path);
        if (symlink(name, link_path) != 0) {
            perror("symlink");
        }
    }
    a.master = master;
    printf("core_sim: robot=%s serial=%s%s%s speed=%.2f%s\n", a.robot_type, name, link_path ? " link=" : "",
           link_path ? link_path : "", speed, a.require_calibration ? " (no calibration until pushed)" : "");
    printf("type 'status', 'power off', 'fault overcurrent 2', 'reboot' or 'quit'\n");
    fflush(stdout);

    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
    char stdin_buf[512];
    size_t stdin_len = 0;
    sc_stream_decoder_t decoder;
    sc_stream_decoder_init(&decoder);
    scorbot_v1_SystemState last_state = a.core.state;
    double last = now_s();
    double accumulated = 0.0;
    bool running = true;
    while (running) {
        struct pollfd fds[2] = {{master, POLLIN, 0}, {STDIN_FILENO, POLLIN, 0}};
        poll(fds, 2, 1);

        uint8_t buf[512];
        ssize_t n;
        while ((n = read(master, buf, sizeof(buf))) > 0) {
            sc_stream_decoder_feed(&decoder, buf, (size_t)n, on_payload, &a);
        }

        /* Fixed 1 ms core ticks, as many as wall-clock time (times speed) asks for. */
        const double t = now_s();
        accumulated += (t - last) * speed;
        last = t;
        if (accumulated > 0.05) {
            accumulated = 0.05;
        }
        while (accumulated >= 0.001) {
            sc_input_t in;
            sc_output_t out;
            plant_read(&a.plant, &in);
            sc_core_tick(&a.core, 0.001f, &in, &out);
            plant_step(&a.plant, &out, 0.001);
            accumulated -= 0.001;
        }

        scorbot_v1_Envelope e;
        while (sc_core_pop(&a.core, &e)) {
            uint8_t frame[SC_MAX_FRAME];
            const size_t len = sc_envelope_to_frame(&e, frame, sizeof(frame));
            if (len == 0 || write(master, frame, len) != (ssize_t)len) {
                ++a.tx_errors;
                tcflush(slave, TCIFLUSH); /* nobody reading: drop stale output like a UART would */
            } else {
                ++a.tx_frames;
            }
        }

        if (!quiet && a.core.state != last_state) {
            printf("state -> %s%s%s\n", sc_state_name(a.core.state),
                   a.core.fault != scorbot_v1_FaultCode_FAULT_NONE ? " fault=" : "",
                   a.core.fault != scorbot_v1_FaultCode_FAULT_NONE ? sc_fault_name(a.core.fault) : "");
            fflush(stdout);
            last_state = a.core.state;
        }

        if (fds[1].revents & POLLIN) {
            const ssize_t r = read(STDIN_FILENO, stdin_buf + stdin_len, sizeof(stdin_buf) - 1 - stdin_len);
            if (r > 0) {
                stdin_len += (size_t)r;
            }
            stdin_buf[stdin_len] = '\0';
            char *nl;
            while ((nl = strchr(stdin_buf, '\n')) != NULL) {
                *nl = '\0';
                running = handle_command(&a, stdin_buf) && running;
                const size_t rest = stdin_len - (size_t)(nl + 1 - stdin_buf);
                memmove(stdin_buf, nl + 1, rest);
                stdin_len = rest;
                stdin_buf[stdin_len] = '\0';
            }
        }
    }
    if (link_path) {
        unlink(link_path);
    }
    close(slave);
    return 0;
}
