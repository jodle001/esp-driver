# esp-driver

Firmware for the custom ESP32 controller that drives a Scorbot (Bluey first, Bingo
next) for the ROS 2 stack in `scorbot_ros2`. It speaks the protocol in
`scorbot_ros2/scorbot_protocol/docs/protocol.md` over USB serial to the Raspberry Pi's
`ros2_control` hardware interface. The old WiFi/ICD driver for Commander is not built
any more; its components (`driver_socket`, `driver_wifi`, `motorhat`, `ads1015`) are
kept only for reference.

## Layout

| Component | What it is | Builds where |
| --- | --- | --- |
| `components/scorbot_core` | the controller: state machine, homing, watchdog, soft limits, faults, calibration, the per-joint control law and every message. Portable C, no ESP-IDF calls | device and host |
| `components/scorbot_proto` | the wire contract: nanopb runtime and generated messages copied from `scorbot_ros2/scorbot_protocol` (`scripts/sync_protocol.sh --check` keeps them identical), COBS + CRC-16 framing in C | device and host |
| `components/link_uart` | the protocol UART at 921600 (UART1, GPIO 5/13, to the Pi's ribbon header on the rev 0.3 board): RX task into the frame decoder, non-blocking TX | device |
| `components/expander` | MCP23017: DIR and SLP out, FLT and limit switches in, INTA on change | device |
| `components/motor` | five LEDC PWM channels + DIR/SLP through the expander for the Pololu G2 18v17 modules | device |
| `components/encoder` | quadrature on the pulse counters (unchanged from the previous firmware) | device |
| `components/limit_switch` | INTA interrupt latches all five counts, then the task reads which switch moved | device |
| `components/current` | two ADS1015 boards: five motor currents and the motor-rail voltage (e-stop detection) | device |
| `components/nvs_store` | boot count, robot type and per-joint calibration in NVS | device |
| `main/` | task wiring and the Kconfig pin map of the rev 0.2 board | device |
| `test/host/` | host build: plant model, the simulator's behaviour tests run against the real core, and `core_sim`, the core behind a pty | host |

## Host build (no ESP-IDF, no hardware)

```bash
cd esp-driver/test/host
make            # builds and runs test_proto (framing vectors) and test_core (behaviour spec)
make core_sim   # then: ./build/core_sim --link /tmp/scorbot_core
```

`test_core` is the port of `scorbot_esp_sim/test/test_controller_sim.cpp`: the same 19
scenarios (boot, ping/info, enable rules, watchdog, homing from both sides of the switch
and again from on the switch, timeouts, soft limits, overcurrent, ClearFault semantics,
reboot, calibration validation, invert and encoder_invert) plus the hardware-only ones
(driver FLT line, motor power loss = e-stop, calibration required before Enable, events
before responses with no seq gaps). `core_sim` puts that core behind a pseudo terminal so
`scorbot_protocol_cli --port /tmp/scorbot_core ping | info | home | monitor 3` and
`robot.launch.py mock:=false serial_port:=/tmp/scorbot_core` talk to the real firmware
logic. Type `power off` on its stdin to press the e-stop, `fault driver 2` to inject.

## Device build (ESP-IDF v5.5, target esp32)

```bash
cd esp-driver
idf.py set-target esp32
idf.py menuconfig          # "Talos ESP Driver Configuration": only if the board differs from rev 0.2
idf.py build flash
idf.py monitor                   # the console is on UART0, the DevKit's USB
```

The protocol link is UART1 (GPIO 5 TX, 13 RX at 921600): on the rev 0.3 board the Pi sits
on the board and its own UART reaches those pins through the 40-pin ribbon header J6; the
3-pin header J3 carries the same two lines for a 3.3 V USB-serial dongle when no Pi is
fitted (one or the other). The DevKit's USB is the console and the flashing port, from a
laptop or from the Pi's USB. `SC_LINK_UART_NUM=0` in menuconfig moves the link onto the
DevKit's USB for a bench laptop instead (then move the console). The status LED: short blip = UNHOMED, 5 Hz = HOMING, solid = READY,
2.5 Hz = ACTIVE, double blink = FAULT.

Bench acceptance, in the order of the design doc
(`project_documentation/technical/ros2/esp_controller_design.md` section 4.4):

1. DevKit alone, nothing on the board (link on the DevKit's USB, `SC_LINK_UART_NUM=0`): `scorbot_protocol_cli --port /dev/ttyUSB0 ping`,
   `info`, `monitor 3`. Without the board the self-test finds no expander or ADC and
   boots into FAULT (`fault=driver`); the link and the stream still work.
2. Full board, no robot: the same, now UNHOMED; `enable --allow-unhomed` then silence
   gives the watchdog fault after 200 ms.
3. One motor, encoder and switch: velocity and position mode, homing from both sides,
   current against a clamp meter.
4. The arm: `robot.launch.py mock:=false`, home, enable, `demo_motion`.

Nothing here has been compiled against ESP-IDF yet (the host build is what has run);
the first `idf.py build` on a machine with the toolchain is the next step.

## Sync with the ROS side

The protocol files are copied, not linked. After changing `scorbot.proto` or the nanopb
options in `scorbot_ros2/scorbot_protocol` and regenerating there:

```bash
components/scorbot_proto/scripts/sync_protocol.sh          # copy
components/scorbot_proto/scripts/sync_protocol.sh --check  # what CI runs
```

## Testing on the device

`test/` is the previous firmware's Unity menu project for the low-level components; the
behaviour tests of this firmware live in `test/host` and run on any machine.
