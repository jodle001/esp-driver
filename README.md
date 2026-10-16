# esp-driver

Firmware for the custom ESP32 controller that drives a Scorbot (Bluey first, Bingo
next). Over the DevKit's USB it speaks the Eshed box's own serial dialect, the ACL
subset the operator sends (`components/scorbot_core/acl.h`, design doc section 7), so a
Raspberry Pi running the unchanged operator cannot tell this board from an Eshed
controller (decided Oct 16 2026). The protobuf protocol of the ROS 2 stack
(`scorbot_ros2/scorbot_protocol/docs/protocol.md`) remains as a second mode on the same
port, chosen in menuconfig or from the first bytes. The old WiFi/ICD driver for Commander
is not built any more; its components (`driver_socket`, `driver_wifi`, `motorhat`,
`ads1015`) are kept only for reference.

## Layout

| Component | What it is | Builds where |
| --- | --- | --- |
| `components/scorbot_core` | the controller: state machine, homing, watchdog, soft limits, faults, calibration, the per-joint control law and every message; `acl.c`, the Eshed dialect on top of it; `link_mode.c`, which protocol the first bytes are. Portable C, no ESP-IDF calls | device and host |
| `components/scorbot_proto` | the wire contract: nanopb runtime and generated messages copied from `scorbot_ros2/scorbot_protocol` (`scripts/sync_protocol.sh --check` keeps them identical), COBS + CRC-16 framing in C | device and host |
| `components/link_uart` | the link UART (UART0, the DevKit's USB): RX task into a stream buffer the control loop reads, non-blocking TX, framing-error count, baud switch | device |
| `components/expander` | MCP23017: DIR and SLP out, FLT and limit switches in, INTA on change | device |
| `components/motor` | five LEDC PWM channels + DIR/SLP through the expander for the Pololu G2 18v17 modules | device |
| `components/encoder` | quadrature on the pulse counters (unchanged from the previous firmware) | device |
| `components/limit_switch` | INTA interrupt latches all five counts, then the task reads which switch moved | device |
| `components/current` | two ADS1015 boards: five motor currents and the motor-rail voltage (e-stop detection) | device |
| `components/nvs_store` | boot count, robot type and per-joint calibration in NVS | device |
| `main/` | task wiring, the link mode, and the Kconfig pin map of the rev 0.3 board (the expander bit map is still rev 0.2's) | device |
| `test/host/` | host build: plant model, the simulator's behaviour tests run against the real core, the dialect's tests, `core_sim` (the core behind a pty) and the end-to-end run with the real operator | host |

## Host build (no ESP-IDF, no hardware)

```bash
cd esp-driver/test/host
make            # builds and runs test_proto (framing vectors), test_core (behaviour spec), test_acl (the dialect)
make core_sim   # then: ./build/core_sim --link /tmp/scorbot_core   (--acl or --protobuf to force the mode)
make e2e OPERATOR=/tmp/opbuild/bin/erv   # the real operator against core_sim, see below
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

`test_acl` drives `scorbot_core/acl` the way the operator does, bytes in and text out:
the prompt rules, the start-up DEFP/HOME/HERE (the prompt only after the homing cycle),
SHIFT/MOVE/MOVED/clrbuf/SPEED in virtual ER-V counts, manual-mode jogs from the pendant
keys with the 10 ms key spam, CON refused with `CONTROL DISABLED` while the motor rail is
off and accepted after MOTORS ON, `IMPACT` for a stall, unknown lines, Ctrl-C, the slide as
axis 7, and that the output never carries XON/XOFF. `core_sim` sniffs the protocol from
the first bytes (a 0x00 is a COBS delimiter, a CR ends an ACL line).

`make e2e` goes the whole way: the real operator (`reference_only/operator`, built out
of tree with `cmake -S reference_only/operator -B /tmp/opbuild -DBUILD_TESTING=OFF
-DCMAKE_BUILD_TYPE=Release && make -C /tmp/opbuild erv`) opens `core_sim`'s pty as its
`/dev/ttyUSB0`, and `e2e_operator.py` plays commander on the operator's ICD port: the
handshake, the start-up home, a TrackingJog, a JointMoveRelative, SetSpeedPercent,
EnableControl, the e-stop seen as a `FAULT CONTROL DISABLED` line, CON refused and then
accepted, a jog ended by PolarPanStop, and a Home. `ptyshim.c` is preloaded into the
operator so its DTR/RTS ioctls succeed on a pty. The operator's own log (`/tmp/talos-e2e/op.log`,
`ACL TX` / `ACL RX` lines) is the byte-level record of the exchange.

## Device build (ESP-IDF v5.5, target esp32)

```bash
cd esp-driver
idf.py set-target esp32
idf.py menuconfig          # "Talos ESP Driver Configuration": link mode, pins, the rev 0.3 defaults
idf.py build flash         # the same USB is the link: unplug the Pi, plug the laptop
```

The link is UART0, the DevKit's USB, and there is no console (the Eshed box has none,
and UART1's pins carry the slide encoder on the rev 0.3 board). "Link to the Pi" in
menuconfig picks the protocol: the Eshed dialect at 9600 (default: the operator), the
protobuf framing at 921600 (the ROS 2 hardware interface), or auto, which starts at 9600
and switches when the first bytes are not text. The dialect's per-joint sign is
`SC_ACL_SIGN_MASK`; its scale is the ER-V's count factors in `scorbot_core/acl.c`, applied
to the NVS calibration, which is pushed once over the protobuf link
(`scorbot_protocol_cli`, in protobuf or auto mode) before the operator is used. A bench
build that wants ESP_LOG on the USB sets the console back to UART0 in sdkconfig and moves
the link to another UART. The status pixels (rev 0.3) are not driven yet; the old status
LED went with its pin.

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
the first `idf.py build` on a machine with the toolchain is the next step. The slide
channel (`SC_JOINT_COUNT=6`) is provided for in Kconfig and handled by the core and the
dialect, but its device glue (the second expander, PWM6, encoder 6, CS6 in the motor,
limit_switch and current components) is not written: the build refuses it until then.

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
