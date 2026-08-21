# ak_v3_driver

A ROS 2 driver for [CubeMars](https://www.cubemars.com/) AK-V3 series servo motors over CAN (SocketCAN), based on the CubeMars AK Series Module Product Manual V3.2.0.

**One node instance controls exactly one motor.** For multiple motors — even on the same physical CAN bus — you run multiple node instances, each filtering to its own `can_id` in software. See [Multiple Motors](#multiple-motors--launch-file) below.

Supported motor models (from the manual's parameter table, manual §4.2): `AK10-9`, `AK60-6`, `AK60-39`, `AK70-9`, `AK80-8`, `AK80-9`. Additional CubeMars AK-V3 actuators exists to add — see [Adding a Motor Model](#adding-a-motor-model).

> **Before first use:** each motor must be calibrated once (motor + encoder parameter identification) using CubeMars's own **CubeMarsTool** upper-computer application over a serial **R-Link** adapter — this ROS package does not perform calibration. See [Motor Calibration (Prerequisite)](#motor-calibration-prerequisite).

>[!Note]
> This driver uses a fire-and-forget model of communication not fire-and-hold. Its behavior is as follows:
>
> 1.The ROS client publishes a MotorCommand message.
> 2.The ROS driver node fires once, packs it into a CAN frame, sends it.
> 3.Nothing happens again until the next message arrives on ~/cmd.

---

## Table of Contents

- [Features](#features)
- [Package Layout](#package-layout)
- [Dependencies](#dependencies)
- [Building](#building)
- [Hardware / CAN Bus Setup](#hardware--can-bus-setup)
- [Motor Calibration (Prerequisite)](#motor-calibration-prerequisite)
- [Quick Start](#quick-start)
- [Nodes](#nodes)
  - [`motor_driver_node`](#motor_driver_node)
- [Parameters](#parameters)
- [Topics](#topics)
  - [`~/cmd` (subscribed)](#cmd-subscribed)
  - [`~/state` (published)](#state-published)
  - [`~/diagnostics` (published)](#diagnostics-published)
- [Services](#services)
  - [`~/set_origin`](#set_origin)
  - [`~/set_feedback_config`](#set_feedback_config)
  - [`~/disable`](#disable)
- [Control Modes Reference](#control-modes-reference)
- [Units and Sign Conventions](#units-and-sign-conventions)
- [Multiple Motors / Launch File](#multiple-motors--launch-file)
- [Adding a Motor Model](#adding-a-motor-model)
- [Architecture Notes](#architecture-notes)
- [Driver Board Hardware Reference](#driver-board-hardware-reference)
- [Troubleshooting](#troubleshooting)
- [Safety Notes](#safety-notes)

---

## Features

- Single generic `~/cmd` topic covering every AK-V3 servo mode (duty, current, current-brake, velocity, position, position+velocity, MIT/force control) via a mode discriminator field, so `ros2 topic echo` shows exactly what's being commanded.
- Decoded motor feedback (`~/state`) in ROS-standard SI units (rad, rad/s, A, N·m), with optional high-resolution multi-turn position support.
- Standard `diagnostic_msgs/DiagnosticStatus` publishing for integration with `diagnostic_aggregator`.
- Services to zero the motor's position sensor (`~/set_origin`), configure optional feedback frames (`~/set_feedback_config`), and coast the motor (`~/disable`).
- Dedicated background CAN RX thread, decoupled from the ROS executor, so feedback ingestion latency isn't affected by other node work.
- `invert_direction` parameter to normalize sign convention regardless of how a motor is mechanically mounted.
- Protocol encode/decode logic (`protocol.hpp`/`.cpp`) has **no ROS dependency**, so it's unit-testable in isolation and reusable from other transports.
- Fails fast at startup on misconfiguration (bad `can_id`, empty `joint_name`, unknown `motor_type`, CAN interface not up) rather than running with ambiguous state.

## Package Layout

```
ak_v3_driver/
├── include/ak_v3_driver/
│   ├── can_socket.hpp          # RAII SocketCAN raw-socket wrapper
│   ├── protocol.hpp             # CAN frame encode/decode (no ROS dependency)
│   ├── motor_limits.hpp         # Per-motor-model spec lookup
│   └── motor_driver_node.hpp    # The ROS 2 node
├── src/
│   ├── can_socket.cpp
│   ├── protocol.cpp
│   ├── motor_limits.cpp         # <-- edit this with your real pole_pairs/gear_ratio
│   ├── motor_driver_node.cpp
│   └── motor_driver_node_main.cpp
├── msg/
│   ├── MotorCommand.msg
│   └── MotorState.msg
├── srv/
│   ├── SetOrigin.srv
│   └── SetFeedbackConfig.srv
├── launch/
│   └── ak_v3_driver.launch.py
├── config/
│   └── motor.yaml               # example single-motor parameter file
└── package.xml
```

## Dependencies

From `package.xml`:

| Type | Package |
|---|---|
| buildtool | `ament_cmake`, `rosidl_default_generators` |
| build/exec | `rclcpp`, `std_srvs`, `diagnostic_msgs`, `sensor_msgs` |
| exec | `rosidl_default_runtime`, `launch`, `launch_ros` |
| test | `ament_lint_auto`, `ament_lint_common`, `ament_cmake_gtest` |

This package also links against Linux SocketCAN headers (`linux/can.h`, `linux/can/raw.h`) — no external CAN library is required, since it talks directly to the kernel's native CAN API.

Tested/intended for ROS 2 (rclcpp-based), `ament_cmake` build type. No specific distro is pinned in the manifest; use whatever ROS 2 distro provides the packages above.

## Building

```bash
# from the root of your colcon workspace
colcon build --packages-select ak_v3_driver
source install/setup.bash
```

## Hardware / CAN Bus Setup

The driver does **not** configure the CAN interface itself — bitrate and bringing the link up are treated as a system/udev concern shared across every process using the bus, not something one motor's ROS node should own. Bring the interface up externally before starting the node:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

The `bitrate` parameter in `config/motor.yaml` is **informational only** — it documents the expected bus speed but does not configure anything. Note the AK-V3 driver boards' CAN bus is fixed at **1 Mbps** (manual §1.1), matching the 1000000 default.

Wire each AK-V3 motor controller to the CAN bus per the CubeMars manual, and set each motor's driver ID (`can_id`, 0–255) via CubeMarsTool (manual §3.1.1.1, "CAN ID") — this driver only *filters* to a `can_id`, it doesn't assign one.

## Motor Calibration (Prerequisite)

Before a motor is usable with this driver, it must be calibrated once — this is a one-time hardware/firmware step done through CubeMars's own tooling, **not** through this ROS package:

1. Connect the driver board to a PC via CubeMars's **R-Link** USB-to-serial adapter (USB → PC, 8-pin port → R-Link, 3-pin UART terminal → motor's serial port) and open **CubeMarsTool** (manual §2.3, §3.1).
2. In the "Basic Settings" tab, click **Read**, then **Motor Identification** (~10 s, motor spins under no load), then **Encoder Identification** (~45 s, motor spins slowly), then **Write** (manual §3.2.1).
3. Recalibration is needed only if the driver board is reinstalled on the motor, the three-phase wiring is changed, or firmware is updated — motors ship pre-calibrated and don't need this on first use.

⚠️ Both identification steps must be run under no-load conditions (motor disconnected from any mechanical load) or the identified parameters will be wrong and the motor may be damaged. Repeated encoder identification runs can heat the motor significantly.

This is unrelated to, and does not fill in, the `pole_pairs`/`gear_ratio` placeholders described [below](#important-pole_pairs-and-gear_ratio-placeholders) — those aren't part of CubeMarsTool's calibration and must still be set by hand in `motor_limits.cpp`.

## Quick Start

1. Bring up the CAN interface (see above).
2. Copy/edit `config/motor.yaml` for your motor (`can_id`, `joint_name`, `motor_type`, etc.).
3. **Fill in real `pole_pairs` / `gear_ratio` values** in `src/motor_limits.cpp` for your motor_type (see [warning below](#important-pole_pairs-and-gear_ratio-placeholders)) and rebuild.
4. Run the node:
   ```bash
   ros2 run ak_v3_driver motor_driver_node --ros-args --params-file config/motor.yaml
   ```
5. Watch feedback:
   ```bash
   ros2 topic echo /ak_v3_driver_node/state
   ```
6. Send a command, e.g. a small current command:
   ```bash
   ros2 topic pub --once /ak_v3_driver_node/cmd ak_v3_driver/msg/MotorCommand "{mode: 1, current: 0.5}"
   ```

For multiple motors, use the launch file instead — see [Multiple Motors](#multiple-motors--launch-file).

## Nodes

### `motor_driver_node`

One instance == one physical motor. On construction it:

1. Declares and validates parameters (fails fast on bad `can_id`, empty `joint_name`/`motor_type`).
2. Looks up `motor_type` in the model spec table (`motor_limits.cpp`); throws if unrecognized.
3. Warns (but does not fail) if the resolved spec still has placeholder `pole_pairs`/`gear_ratio` (1.0/1.0).
4. Opens and binds the SocketCAN socket for `can_interface`; throws `CanSocketError` if that fails.
5. Creates the `~/cmd` subscription, `~/state`/`~/diagnostics` publishers, and the three services.
6. Starts a dedicated CAN RX thread (polls with a 50 ms timeout) and a 200 Hz wall timer that drains the RX thread's queues and publishes on the executor thread.

If construction throws (bad params, unreachable CAN interface, unknown motor type), `motor_driver_node_main.cpp` logs the error at `FATAL` and exits with status 1 rather than spinning a half-initialized node.

The destructor stops the RX thread and joins it before any other teardown, so no background thread is touching node state during shutdown.

## Parameters

All declared in `loadParameters()`, run once at construction. **None are reconfigurable at runtime** — changing `can_id` or `motor_type` while running would change which physical motor the node owns, so that's deliberately unsupported; restart the node instead.

| Parameter | Type | Default | Required | Notes |
|---|---|---|---|---|
| `can_interface` | string | `"can0"` | no | SocketCAN interface name. Must already be up (see [Hardware Setup](#hardware--can-bus-setup)). |
| `can_id` | int | `-1` (invalid) | **yes** | This motor's driver ID on the bus, `0`–`255`. Throws at startup if unset or out of range. |
| `joint_name` | string | `""` | **yes** | Used as the joint/frame identifier in `MotorState`/`DiagnosticStatus`. Throws at startup if empty. |
| `motor_type` | string | `""` | **yes** | Must match a key in the model table in `motor_limits.cpp` (e.g. `"AK80-9"`). Throws at startup if empty or unrecognized. |
| `invert_direction` | bool | `false` | no | Flips the sign of position/velocity/torque on both outgoing commands and incoming state. |
| `bitrate` | int | `1000000` | no | **Informational only** — does not configure the CAN interface. |
| `feedback_hz` | int | `100` | no | Requested rate at which the motor itself pushes status frames onto the bus. Not directly enforced by this node; consult the manual for how the motor applies this. |

Example (`config/motor.yaml`):

```yaml
ak_v3_driver_node:
  ros__parameters:
    can_interface: "can0"
    bitrate: 1000000
    can_id: 1
    joint_name: "joint1"
    motor_type: "AK80-9"
    invert_direction: false
    feedback_hz: 100
```

## Topics

All topics/services are node-relative (`~/...`), so under the launch file's per-joint namespacing they come out as e.g. `/shoulder/cmd`, `/shoulder/state`, `/shoulder/set_origin`.

### `~/cmd` (subscribed)

- Type: `ak_v3_driver/msg/MotorCommand`
- QoS: `SensorDataQoS` (best-effort, shallow depth) — a stale queued command is worse than a dropped one in a high-rate control loop, so this favors latest-command-wins over guaranteed delivery.

A single generic message covers every control mode via a `mode` discriminator, rather than one topic per mode — this way `ros2 topic echo` shows exactly what's being commanded, including which mode.

```
uint8 MODE_DUTY              = 0
uint8 MODE_CURRENT           = 1
uint8 MODE_CURRENT_BRAKE     = 2
uint8 MODE_VELOCITY          = 3
uint8 MODE_POSITION          = 4
uint8 MODE_POSITION_VELOCITY = 6
uint8 MODE_MIT               = 8

uint8 mode

# --- servo-mode fields ---
float64 duty                 # MODE_DUTY: -1.0 .. 1.0
float64 current               # MODE_CURRENT / MODE_CURRENT_BRAKE: amps
float64 velocity              # MODE_VELOCITY / MODE_POSITION_VELOCITY: rad/s
float64 position              # MODE_POSITION / MODE_POSITION_VELOCITY / MODE_MIT: rad
float64 acceleration          # MODE_POSITION_VELOCITY: rad/s^2

# --- MIT-mode-only fields (all sent together in one frame) ---
float64 mit_velocity          # MODE_MIT: rad/s
float64 mit_torque            # MODE_MIT: N*m, feed-forward torque
float64 kp                    # MODE_MIT: position gain
float64 kd                    # MODE_MIT: velocity gain
```

Only the fields relevant to `mode` are read; the rest are ignored. An unrecognized `mode` value is logged (throttled to 2 s) and the message is dropped rather than sending a garbage frame.

> **Note:** `MODE_DISABLE` (protocol mode 15) exists in the CAN protocol layer but is **not currently wired up** as a `MotorCommand` mode — use the `~/disable` service instead (see [Services](#services)).

### `~/state` (published)

- Type: `ak_v3_driver/msg/MotorState`
- QoS: `SensorDataQoS`
- Rate: driven by the 200 Hz drain timer, publishing once per queued status frame received since the last drain (so effectively at the rate the motor sends 0x29 status frames — see `feedback_hz`).

Decoded feedback from the motor's periodic status frame (and the optional high-resolution position frame, if enabled). Units are ROS-standard (rad, rad/s, A, N·m); `invert_direction` has already been applied.

```
std_msgs/Header header
string joint_name

float64 position              # rad
float64 velocity               # rad/s
float64 current                # A
float64 estimated_torque       # N*m, current * motor's torque coefficient

int8 temperature_c             # driver board temperature, degrees C
uint8 error_code                # raw error code from the motor, 0 = no fault
string error_text               # human-readable decode of error_code

bool extended_position_valid    # true if `position` came from the optional
                                 # 0x2A high-resolution frame rather than the
                                 # lower-resolution 0x29 status frame
```

`estimated_torque` is **derived** (`current × torque coefficient`), not a direct sensor measurement.

If the optional extended (multi-turn) position frame is enabled, its value takes priority over the lower-resolution status-frame position once received, and `extended_position_valid` reflects that. A `~/set_feedback_config` call that *disables* the extended frame clears the cached value, so publishing falls back to the status-frame position again.

### `~/diagnostics` (published)

- Type: `diagnostic_msgs/msg/DiagnosticStatus`
- QoS: `SystemDefaultsQoS` (reliable — diagnostics shouldn't be silently dropped)
- Published once per `~/state` message, mirroring the same fault info.

`level` is `OK` when `error_code == 0`, otherwise `ERROR` with `message` set to the decoded fault text. `hardware_id` is the motor's `can_id` as a string.

## Services

### `~/set_origin`

- Type: `ak_v3_driver/srv/SetOrigin`
- Zeros the motor's position sensor at its current physical position (manual §4.1.6).

```
bool permanent   # false = temporary (erased on power loss)
                  # true  = permanent (written to flash -- don't call often)
---
bool success
string message
```

### `~/set_feedback_config`

- Type: `ak_v3_driver/srv/SetFeedbackConfig`
- Toggles optional status-reporting features on the motor controller (manual §4.1.9).

```
bool add_extended_position_frame   # enable the high-resolution 0x2A frame
bool single_turn_mode                # report position as 0-360 deg instead
                                      # of multi-turn
---
bool success
string message
```

> **⚠️ Writes to motor flash memory.** Call this once at startup configuration time, not repeatedly / in a hot loop.

### `~/disable`

- Type: `std_srvs/srv/Trigger`
- Sends an immediate disable ("coast the motor") command. No request fields.

```
---
bool success
string message
```

All three services return `success = false` with `message = "CAN send failed"` if the underlying CAN write failed (a transient send error, not an exception).

## Control Modes Reference

| `mode` value | Constant | CubeMars protocol mode ID | Wire units | Notes |
|---|---|---|---|---|
| 0 | `MODE_DUTY` | 0 | duty × 100000 (int32) | Open-loop PWM, range −1.0 .. 1.0 |
| 1 | `MODE_CURRENT` | 1 | current_a × 1000 (int32, mA) | Closed-loop Iq current (torque = Iq × Kt); direction inverted by `invert_direction` |
| 2 | `MODE_CURRENT_BRAKE` | 2 | current_a × 1000 (int32, mA) | Braking current holds position; magnitude only — **not** affected by `invert_direction` |
| 3 | `MODE_VELOCITY` | 3 | ERPM, int32, no scaling | rad/s converted to ERPM via pole_pairs × gear_ratio |
| 4 | `MODE_POSITION` | 4 | position_deg × 10000 (int32) | rad converted to degrees |
| 6 | `MODE_POSITION_VELOCITY` | 6 | position ×10000; speed/accel ERPM ÷10 (int16 each) | Combined position + velocity + acceleration feed-forward |
| 8 | `MODE_MIT` | 8 | bit-packed kp(12)\|kd(12)\|pos(16)\|vel(12)\|torque(12) across 8 bytes | Impedance/force control; see below |
| — | `MODE_SET_ORIGIN` | 5 | 1 byte: 0/1 | Via `~/set_origin` service, not `~/cmd` |
| — | `MODE_DISABLE` | 15 | no payload | Via `~/disable` service, not `~/cmd` |
| — | `MODE_FEEDBACK_CONFIG` | 16 | uint16 flags at byte offset 6 | Via `~/set_feedback_config` service, not `~/cmd` |

**MIT mode** ranges (shared across all AK-V3 models, manual §4.2):

| Field | Min | Max | Resolution |
|---|---|---|---|
| Position | −12.56 rad | 12.56 rad | 16-bit |
| Kp | 0.0 | 500.0 | 12-bit |
| Kd | 0.0 | 5.0 | 12-bit |
| Velocity | model-specific (`motor_limits.cpp`) | | 12-bit |
| Torque | model-specific (`motor_limits.cpp`) | | 12-bit |

All MIT-mode inputs are clamped to range before packing — out-of-range values are silently saturated (the motor would clamp them too; clamping client-side just means you find out via ROS logs instead of silently on the bus).

**Feedback frames the driver decodes** (manual §4.3.1):

| Function ID | Name | Contents |
|---|---|---|
| `0x29` | Status frame | position (0.1°), speed (10 ERPM), current (0.01 A), temperature, fault code — periodic |
| `0x2A` | Extended position frame | position (0.0001°, multi-turn) — optional, enabled via `~/set_feedback_config` |
| `0x2C` | Startup frame | one-shot "servo mode entered" marker (`0xFA 0xFB 0xFC 0xFD`) — detected but not published anywhere |

**Fault codes** (status frame byte 7 — note this is a shorter 0–7 set than the manual's full `mc_fault_code` enum used elsewhere in the protocol, e.g. FAULT_CODE_DRV, FAULT_CODE_ENCODER_SPI, FAULT_CODE_UNBALANCED_CURRENTS, etc. — the 0x29 status frame only ever reports codes 0–7):

| Code | Meaning |
|---|---|
| 0 | none |
| 1 | motor_over_temperature |
| 2 | over_current |
| 3 | over_voltage |
| 4 | under_voltage |
| 5 | encoder_fault |
| 6 | mosfet_over_temperature |
| 7 | motor_lockup |
| other | unknown |

**Torque estimation:** the manual defines the motor's output torque as `T = Kt × Iq` (§4.2), where `Kt` is the model's torque coefficient and `Iq` is the measured phase (Q-axis) current — this is exactly the calculation `MotorDriverNode` uses to populate `MotorState.estimated_torque` from the status frame's current reading, so it's a computed value, not an independent sensor measurement.

## Units and Sign Conventions

- Public API (`MotorCommand`, `MotorState`) is entirely SI: radians, rad/s, N·m, amps.
- The underlying VESC-style wire protocol uses **degrees** for position and **ERPM** (electrical RPM) for velocity; conversion happens at the CAN boundary inside the node.
- `rad/s ↔ ERPM` conversion depends on `pole_pairs` and `gear_ratio` (see [placeholder warning](#important-pole_pairs-and-gear_ratio-placeholders)):

  ```
  output_rpm = rad_s * 60 / (2π)
  erpm       = output_rpm * pole_pairs * gear_ratio
  ```

- `invert_direction: true` flips the sign of position/velocity/torque on **both** outgoing commands and incoming state, so callers get a consistent sign convention independent of how the motor happens to be mechanically mounted or wired. It does **not** apply to braking current (`MODE_CURRENT_BRAKE`), which is a direction-less magnitude in the protocol.

## Multiple Motors / Launch File

Each motor gets its own node instance and its own ROS namespace, taken from `joint_name`:

```
/shoulder/cmd, /shoulder/state, /shoulder/set_origin, ...
/elbow/cmd,    /elbow/state,    /elbow/set_origin,    ...
```

Multiple motors on the same physical CAN bus simply repeat `can_interface` — each node filters to its own `can_id` in software and ignores every other frame on the wire, so nodes don't interfere with each other even sharing a bus.

Edit the `MOTORS` list in `launch/ak_v3_driver.launch.py` (or replace it with a YAML-driven loop) to match your robot:

```python
MOTORS = [
    {
        "joint_name": "wheel1",
        "can_interface": "can0",
        "can_id": 2,
        "motor_type": "AK60-6",
        "invert_direction": False,
    },
    {
        "joint_name": "wheel2",
        "can_interface": "can0",
        "can_id": 3,
        "motor_type": "AK60-6",
        "invert_direction": True,
    },
]
```

```bash
ros2 launch ak_v3_driver ak_v3_driver.launch.py
```



## Adding a Motor Model

The table in `src/motor_limits.cpp` currently only has a few of the existing motor models. To add a model not in the table at all, add a new entry following the same `{kv, kt, v_min, v_max, t_min, t_max, pole_pairs, gear_ratio}` shape, sourced from manual §4.2. An unrecognized `motor_type` string is treated as a configuration error at startup — the node throws rather than silently falling back to a default spec.

Warning the Kt values listed in the specifications on the website are not accurate contact CubeMars or see AK-V3 documentation for accurate Kt values.

## Architecture Notes

- **`protocol.hpp`/`protocol.cpp`** has no `rclcpp` or ROS-type dependency by design, so the encode/decode logic is unit-testable in isolation (`test/test_protocol.cpp`) and reusable from a transport other than SocketCAN if ever needed.
- **`can_socket.hpp`/`can_socket.cpp`** is a minimal RAII wrapper directly over the kernel's native `AF_CAN`/`SOCK_RAW` API — no external CAN library dependency. It does not configure bitrate or bring the interface up; that's left to the system.
- **RX threading model:** a dedicated background thread polls the CAN socket (50 ms timeout per iteration, so shutdown is noticed promptly) and pushes decoded frames onto mutex-guarded queues. A 200 Hz wall timer on the ROS executor thread swaps out those queues (holding the lock only briefly) and does all ROS-side work (publishing, logging) — this keeps CAN ingestion latency independent of whatever else the executor is doing, while keeping all rclcpp interaction on the executor thread.
- **CAN ID structure:** the 29-bit extended CAN identifier packs `(mode_id << 8) | can_id`, so a single arbitration ID encodes both "what kind of message" and "which motor" (`buildCanId()` / inverse filtering in the decoders).
- **Error frames / remote frames / non-extended frames** are silently filtered out in `CanSocket::receive()` — the AK-V3 protocol only uses extended (29-bit) data frames, and motor faults are reported via the status frame's error byte rather than CAN-level error frames.

## Driver Board Hardware Reference

This driver talks to any AK-V3 driver board over its CAN interface. For reference, the manual (§1.1) lists three driver board variants, distinguished by current rating and compatible motor models:

| Driver board | Compatible motors | Rated current (rms) | Max current (peak) | Size |
|---|---|---|---|---|
| AK54-4810-1C-A2 | `AK60-6`, `AKE60-8` | 10 A | 30 A | 54×54 mm |
| AK60-4820-1C-A2 | `AK70-9`, `AK80-9`, `AK10-9`, `AKE80-8` | 20 A | 60 A | 63×57 mm |
| AK80-4830-1C-A4 | `AKE90-8` | 30 A | 90 A | 76.33×85 mm |

All three share: 18–52 V allowable working voltage, 1 Mbps CAN bitrate, −20 °C to 65 °C operating range, 21-bit inner-loop encoder resolution.

**CAN wiring:** the manual defines `CAN_H`/`CAN_L` signal lines on each board's power+CAN connector (white/blue wires respectively on the smaller boards). Multiple driver boards share the same two-wire CAN bus — this is exactly what makes the multi-node/shared-`can_interface` setup in this package's [launch file](#multiple-motors--launch-file) possible.

**Indicator lights** (manual §1.5) — useful when debugging a node that isn't seeing `~/state` messages:

| Light | Color | Meaning |
|---|---|---|
| Power | Blue | On = driver board powered; should stay on continuously |
| Operation | Green | On = motor actively driving; should pulse ~2 s at startup then track motor activity |
| Drive fault | Red | On = driver board fault; should be off during normal operation |

If the blue power light doesn't come on when the board is powered, the manual recommends removing power immediately rather than retrying — this points to a hardware fault upstream of anything this ROS driver can diagnose.

**CubeMarsTool / R-Link** (manual §2, §3) is CubeMars's own Windows configuration utility, used over a serial connection (not CAN) for calibration, firmware updates, advanced parameter tuning, and one-off manual jogging in servo/MIT mode. It's a separate tool from this ROS package — this driver only ever talks over the CAN interface, and doesn't replace or wrap CubeMarsTool's serial workflow.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Node fails to start: `Unknown motor_type '...'` | `motor_type` param doesn't match a key in `motor_limits.cpp`'s table (check spelling/case against manual model names) |
| Node fails to start: `Failed to open CAN interface` | Interface not brought up (`sudo ip link set can0 up type can bitrate ...`), wrong interface name, or permissions |
| Node fails to start: `'can_id' must be set to a value in [0, 255]` | `can_id` param missing or out of range |
| No `~/state` messages | Motor not powered/wired, wrong `can_id` (must match the motor's actual configured driver ID, not just this param), or `feedback_hz` misconfigured on the motor itself |
| Velocity numbers look scaled wrong | `pole_pairs`/`gear_ratio` still at 1.0/1.0 placeholders for this `motor_type` — see [above](#important-pole_pairs-and-gear_ratio-placeholders) |
| `CAN send failed` warnings/service responses | Bus down, arbitration/wiring issue, or transient kernel write failure — check `ip -details link show can0` and bus wiring |
| `~/state.position` jumps when toggling extended feedback | Expected — disabling the extended (0x2A) frame via `~/set_feedback_config` clears the cached high-resolution value and falls back to the lower-resolution status-frame position |

## Safety Notes

- `MODE_CURRENT_BRAKE` and `~/disable` do not replace a proper e-stop / power-cutoff design for your system — they only send a CAN command, which assumes the bus and controller are still responsive.
- `~/set_origin` with `permanent: true` and `~/set_feedback_config` both **write to the motor's flash memory**. Don't call these in a loop or from a periodic process — flash has a limited write-cycle lifetime.
- MIT-mode `kp`/`kd` gains directly drive motor torque response — start with conservative gains, especially on a first bring-up with real load attached, since out-of-range values are clamped rather than rejected.
