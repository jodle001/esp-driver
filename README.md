# esp-driver

Firmware for the custom controller that drives a Scorbot (Bluey first, Bingo next): an
ESP32-S3-DevKitC-1 on the rev 0.4 board with seven channels, the arm, the gripper and the
rail, each encoder counted by its own LS7366R chip on SPI (design doc section 8, decided
Oct 20 2026). Over the DevKit's USB-UART bridge it speaks the Eshed box's own serial dialect, the ACL
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
| `components/expander` | two MCP23017: the DIR chip (DIR1-7 out, the MOTORS ON chain sense in) and the sense chip (FLT1-7 and SW1-7 in), one shared INT line | device |
| `components/motor` | up to seven LEDC PWM channels + DIR through the DIR chip, the shared SLP line, for the Pololu G2 18v17 modules | device |
| `components/encoder_ls7366r` | the seven LS7366R quadrature counters on SPI: setup with read-back (self-test), the per-tick read of all counts, the filter clock from LEDC | device |
| `components/encoder` | the rev 0.3 pulse-counter encoder, kept for reference, not built | reference |
| `components/limit_switch` | the INT line wakes a task that reads every counter (the latch), then which switch, fault line or chain sense moved | device |
| `components/current` | two ADS1015 boards: seven motor currents and the motor-rail voltage (e-stop detection) | device |
| `components/nvs_store` | boot count, robot type and per-joint calibration in NVS | device |
| `components/pixel_strip` | the ten status pixels on one GPIO (RMT): a task sends the frames the control loop hands over; what each pixel shows is decided in `scorbot_core/pixels.c` | device |
| `main/` | task wiring, the link mode, and the Kconfig pin map of the rev 0.4 board | device |
| `test/host/` | host build: plant model, the simulator's behaviour tests run against the real core, the dialect's tests, `core_sim` (the core behind a pty) and the end-to-end run with the real operator | host |

## Host build (no ESP-IDF, no hardware)

```bash
cd esp-driver/test/host
make            # builds and runs test_proto (framing vectors), test_core (behaviour spec), test_acl (the dialect), test_pixels (the status pixels' table)
make core_sim   # then: ./build/core_sim --link /tmp/scorbot_core   (--acl or --protobuf to force the mode)
make e2e OPERATOR=/tmp/opbuild/bin/erv   # the real operator against core_sim, see below
```

`test_core` is the port of `scorbot_esp_sim/test/test_controller_sim.cpp`: the same 19
scenarios (boot, ping/info, enable rules, watchdog, homing from both sides of the switch
and again from on the switch, timeouts, soft limits, overcurrent, ClearFault semantics,
reboot, calibration validation, invert and encoder_invert) plus the hardware-only ones
(driver FLT line, motor power loss = e-stop, calibration required before Enable, events
before responses with no seq gaps, and the encoder 5 V sense: its debounce, a loss while
ready, active, homing and during an e-stop fault, and the three advisory flags). `core_sim` puts that core behind a pseudo terminal so
`scorbot_protocol_cli --port /tmp/scorbot_core ping | info | home | monitor 3` and
`robot.launch.py mock:=false serial_port:=/tmp/scorbot_core` talk to the real firmware
logic. Type `power off` on its stdin to press the e-stop, `enc5v off` to take the
encoders' supply away (`enc5v on` turns the sense on with the supply present), `fault
driver 2` to inject.

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

## Device build (ESP-IDF v5.5, target esp32s3)

```bash
cd esp-driver
idf.py set-target esp32s3
idf.py menuconfig          # "Talos ESP Driver Configuration": link mode, joints (7), pins, the rev 0.4 defaults
idf.py build flash         # flashing goes through the "UART" micro-USB socket, the same one the Pi uses
idf.py monitor -p /dev/ttyACM0   # the console is the other socket, "USB" (USB-Serial-JTAG)
```

The link is UART0, the DevKit's USB-UART bridge (the first micro-USB socket); the console,
logs and JTAG are the S3's USB-Serial-JTAG peripheral on the second socket, so no UART
pin is spent and nothing but the protocol reaches the Pi after the ROM's own boot line
(silenced for good by burning the `UART_PRINT_CONTROL` eFuse to 3 with `espefuse.py`, a
once-per-DevKit commissioning step). "Link to the Pi" in
menuconfig picks the protocol: the Eshed dialect at 9600 (default: the operator), the
protobuf framing at 921600 (the ROS 2 hardware interface), or auto, which starts at 9600
and switches when the first bytes are not text. The dialect's per-joint sign is
`SC_ACL_SIGN_MASK`; its scale is the ER-V's count factors in `scorbot_core/acl.c`, applied
to the NVS calibration, which is pushed once over the protobuf link
(`scorbot_protocol_cli`, in protobuf or auto mode) before the operator is used.

The ten status pixels ("Status pixels" in menuconfig, IO21 through the level shifter)
follow the table of the design doc, section 4.6: one per motor channel (amber unhomed,
blue blinking while homing, green homed, breathing while moving, red blinking on a
fault), then SYS (the controller's state), LINK (the Pi link: green for the Eshed
dialect, cyan for protobuf, blinking when idle) and E-STOP (red open, amber closed but
MOTORS ON not pressed, green latched). `test_pixels` checks that table on the host.
Every colour is scaled by `SC_PIXEL_MAX_LEVEL` (32 of 255) and the first frame after boot
is dark. Two things are to be confirmed at first power, both in menuconfig: the colour
order on the wire (RGB by default) and the bit timing (the WS2812 timing by default).

The encoder 5 V sense ("Encoder 5 V sense" in menuconfig; IO48 and IO38 on the rev 0.5
board) is read every control tick. With the PC supply off and the Pi still plugged in, the
DevKit runs on USB power while the encoders are dark, so the counts stop and the arm can
be moved unseen. Positions are trusted only while the two pins read (1, 0): three samples
of anything else clear every homed flag, fault a running or homing arm with `ENCODER`,
and refuse Enable and Home until the pins have read (1, 0) for 100 ms; Enable then stays
refused until Home has completed. The verdict and three advisories (both pins low, the
channels disagreeing for 5 s, a rail hovering at the threshold) are logged on the console.
Over the Eshed dialect a refusal reads `CONTROL DISABLED (encoder 5 V supply ...)`.

Bench acceptance, in the order of the design doc
(`project_documentation/technical/ros2/esp_controller_design.md` section 4.4):

1. DevKit alone, nothing on the board (link on the DevKit's USB, `SC_LINK_UART_NUM=0`): `scorbot_protocol_cli --port /dev/ttyUSB0 ping`,
   `info`, `monitor 3`. Without the board the self-test finds no expander, counter or ADC
   and boots into FAULT (`fault=driver`); the link and the stream still work.
2. Full board, no robot: the same, now UNHOMED; `enable --allow-unhomed` then silence
   gives the watchdog fault after 200 ms.
3. One motor, encoder and switch: velocity and position mode, homing from both sides,
   current against a clamp meter.
4. The arm: `robot.launch.py mock:=false`, home, enable, `demo_motion`.

Nothing here has been compiled against ESP-IDF yet (the host build is what has run);
the first `idf.py build` on a machine with the toolchain is the next step. The LS7366R
register map in `encoder_ls7366r.h` is from the part's common usage, not read off the
datasheet in this sandbox: the init reads MDR0 back and fails loudly if a chip disagrees.
The limit-switch latch is the software one (counters read about 100 us after the
interrupt); the chips' INDEX inputs are wired to the switch lines on the board so the
hardware latch (`SC_CNT_INDEX_LATCH`, the chip's index mode "load OTR") can take over once the bench shows the OTR holds the count at the switch edge rather than following CNTR while the switch is held; either way a periodic CNTR read copies CNTR into OTR, so the latch is read first.

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
