<div align="center">

# RobotX PUCP 2026

### Autonomous Maritime Robotics Software Stack

**Pontificia Universidad Católica del Perú**

ROS 2 · Autonomous Navigation · Perception · Control · Simulation · Marine Robotics

</div>

---

## Overview

This repository contains the ROS 2 software stack developed by the **RobotX PUCP Team** for the **Maritime RobotX Challenge 2026**.

The project integrates the software required for the team's surface, underwater and aerial robotic platforms, including:

- Hardware interfaces
- Teleoperation
- Motion control
- Autonomous navigation
- Perception
- Safety systems
- Mission execution
- Simulation
- Hardware testing

The repository is organized by robotic platform and software functionality in order to keep the system modular, maintainable and scalable as new sensors, actuators and RobotX missions are incorporated. Shared subsystems that coordinate more than one vehicle are placed in `SYSTEM_ws` instead of being owned by a single vehicle workspace.

---

## Robotic Platforms

| Platform | Vehicle | Purpose | Status |
|:---:|---|---|:---:|
| **USV** | Blue Robotics **BlueBoat** | Autonomous surface navigation and RobotX mission execution | 🟡 Development |
| **UUV** | Blue Robotics **BlueROV2 Heavy** | Underwater perception, inspection, teleoperation and autonomous missions | 🟢 Real + Simulation |
| **UAV** | Platform TBD | Aerial support for RobotX missions | ⚪ Platform selection |

**Status:**  
🟢 Integrated · 🟡 In development · ⚪ Pending

---

# Repository Organization

```text
Robotx_PUCP/
│
├── assets/
│   ├── images/
│   └── logos/
│
├── docs/
│
├── USV_ws/
│   ├── hardware_tests/
│   └── src/
│       └── usv/
│           ├── usv_bringup/
│           ├── usv_control/
│           ├── usv_hardware/
│           ├── usv_mavlink/
│           ├── usv_perception/
│           └── usv_teleop/
│
├── UUV_ws/
│   ├── docs/
│   │   └── commands/
│   │       ├── real/
│   │       └── simulation/
│   ├── hardware_tests/
│   │   ├── camera/
│   │   ├── dvl/
│   │   └── ping360/
│   ├── real_ws/
│   │   └── src/
│   │       ├── bringup/
│   │       │   └── uuv_bringup/
│   │       ├── control/
│   │       │   ├── camera_control/
│   │       │   ├── rov_control/
│   │       │   └── uuv_teleop/
│   │       ├── drivers/
│   │       │   ├── bluerov2_camera/
│   │       │   ├── ping360_ros2/
│   │       │   └── uuv_mavlink/
│   │       ├── experimental/
│   │       ├── perception/
│   │       ├── tasks/
│   │       └── third_party/
│   └── simulation_ws/
│       └── src/
│           ├── control/
│           ├── sensors/
│           ├── simulator/
│           └── tasks/
│
├── SYSTEM_ws/
│   ├── docs/
│   │   └── commands/
│   │       └── COMANDOS_USBL
│   ├── hardware_tests/
│   │   └── usbl/
│   │       ├── test_system_info_udp.py
│   │       └── test_status_udp.py
│   └── src/
│       └── usbl/
│           └── seatrac_ros2/
│               ├── config/
│               │   ├── freshwater.yaml
│               │   └── seawater.yaml
│               ├── launch/
│               │   └── usbl.launch.py
│               └── seatrac_ros2/
│                   ├── seatrac_serial_node.py
│                   ├── seatrac_status_node.py
│                   └── seatrac_udp_node.py
│
├── UAV_ws/
│   ├── hardware_tests/
│   └── src/
│
└── README.md
```

---

# USV — BlueBoat

The Unmanned Surface Vehicle is based on the **Blue Robotics BlueBoat**.

Its ROS 2 software is located in:

```text
USV_ws/src/usv/
```

and is divided according to functionality.

### `usv_bringup`

Launch and startup configuration for the USV.

It is intended to provide a centralized way to start the required USV subsystems instead of manually running every node in a separate terminal.

### `usv_control`

Contains the main motion and navigation control nodes.

### `usv_hardware`

Contains interfaces with the physical hardware installed on the BlueBoat.

Current hardware-related development includes:

- Beacon system
- Reel mechanism
- MG6010 CAN motor interface
- Physical actuator control

### `usv_perception`

Contains perception and camera-related processing.

### `usv_teleop`

Contains manual teleoperation tools used during development, debugging and testing. The current package includes the existing keyboard teleoperation node. Xbox teleoperation for the BlueBoat will be added progressively using the same safety-first approach already validated on the UUV: first ROS-only command generation, then guarded MAVLink actuation.

### `usv_mavlink`

ROS 2 ↔ MAVLink interface for the physical BlueBoat. The first validated implementation is intentionally **read-only** and receives telemetry from BlueOS without sending ARM, DISARM, mode-change or propulsion commands.

Current physical and network configuration:

```text
Vehicle                  Blue Robotics BlueBoat
Autopilot board          Navigator
Firmware                 ArduPilot 4.6.3 STABLE
MAVLink vehicle type     Surface Boat
BlueOS address           192.168.2.3
ROS laptop address       192.168.2.1
QGroundControl endpoint  UDP 14550
ROS 2 endpoint           UDP 14553
Vehicle SYSID            2
Autopilot COMPID         1
```

The dedicated ROS endpoint is kept separate from QGroundControl so both interfaces can coexist without sharing the same UDP destination.

Current telemetry topics:

```text
/usv/connected
/usv/armed
/usv/mode
/usv/power/voltage
/usv/power/current
/usv/power/remaining
/usv/gps/fix_type
/usv/gps/satellites
/usv/gps/latitude
/usv/gps/longitude
/usv/attitude/yaw_deg
```

Hardware validation confirmed:

- MAVLink heartbeat reception through BlueOS UDP `14553`
- Correct filtering of the BlueBoat autopilot as `SYSID=2`, `COMPID=1`
- Surface-boat heartbeat identification
- Armed-state and navigation-mode decoding
- Battery voltage/current/remaining-capacity telemetry
- GPS and global-position message reception
- Attitude/yaw telemetry

The bridge currently starts in **READ-ONLY mode**. It does not send `MANUAL_CONTROL`, ARM/DISARM commands or mode changes. Physical command output will be added only after the Xbox command path has first been validated without actuation.

### `hardware_tests`

Contains standalone tests used to validate individual physical components before integrating them into the complete ROS 2 system.

---

# SYSTEM — Shared RobotX Services

`SYSTEM_ws` contains ROS 2 components that are shared between vehicles or provide system-level coordination. These components are intentionally kept outside `USV_ws`, `UUV_ws` and `UAV_ws` when they are not owned by a single platform.

The first subsystem migrated to this workspace is the **SeaTrac USBL** interface.

## SeaTrac USBL Architecture

Current physical and network architecture:

```text
Laptop / ROS 2
192.168.2.1
      │
      │ UDP :15000
      ▼
BlueBoat BlueOS
192.168.2.3
      │
      │ Serial Bridge
      ▼
/dev/ttyUSB0 @ 115200
      │
      │ RS232
      ▼
SeaTrac X150
      │
      │ acoustic link — enabled only for in-water tests
      ▼
SeaTrac X110 on the UUV
```

The X150 is physically connected to the BlueBoat. BlueOS acts only as a serial-to-UDP bridge; the SeaTrac protocol and ROS 2 logic run on the operator laptop in `SYSTEM_ws`.

This organization allows the same USBL measurement to later support both intended coordination modes:

- UUV follows USV
- USV follows UUV

The SeaTrac driver publishes measurements and diagnostics. Higher-level vehicle-following behavior will be implemented separately so the USBL driver itself does not command either vehicle.

## Current SeaTrac Functionality

The current implementation has been validated with the physical X150 through BlueOS UDP.

```text
BlueOS address             192.168.2.3
BlueOS Serial Bridge port  15000 / UDP server
SeaTrac serial interface   /dev/ttyUSB0 @ 115200
ROS 2 transport            UDP
STATUS rate                1 Hz
```

Validated functions:

- `SYSTEM_INFO` request and `$02` response
- Periodic `STATUS` request and `$10` response
- CRC validation and status decoding
- Voltage, temperature, pressure, depth and sound-speed decoding
- AHRS orientation and magnetometer-calibration status
- Magnetometer calibration services preserved from the original driver
- Environment read/apply services
- Fresh-water and sea-water ROS 2 profiles
- Guarded persistent-settings save

Primary topics:

```text
/seatrac/system_info_raw
/seatrac/status_raw
/seatrac/status
```

Primary services:

```text
/seatrac/environment/read
/seatrac/environment/apply
/seatrac/settings/save
/seatrac/calibration/reset_magnetometer
/seatrac/calibration/calculate_magnetometer
```

### Environment profiles

Two profiles are currently provided:

| Profile | Salinity | AUTO_VOS | AUTO_PRESSURE_OFS |
|---|---:|:---:|:---:|
| `freshwater` | `0.0 ppt` | `true` | `true` |
| `seawater` | `35.0 ppt` | `true` | `true` |

Loading a profile changes ROS 2 parameters only. It does **not** automatically modify the SeaTrac configuration.

`/seatrac/environment/apply` explicitly applies the selected environmental settings to the X150 working configuration. Current testing confirmed that a temporary change from `0.0 ppt` to `35.0 ppt` returned to `0.0 ppt` after a physical X150 power cycle when no persistent save was performed.

Persistent writes are blocked by default:

```text
allow_persistent_save = false
```

Therefore `/seatrac/settings/save` returns a failure without sending the persistent-save command unless persistent saving is deliberately enabled. Normal operation is designed to avoid unnecessary non-volatile-memory writes.

Acoustic positioning is also disabled by default:

```text
acoustic_positioning_enabled = false
```

The X150 ↔ X110 ranging/USBL-positioning layer is pending in-water validation and must not be assumed to be implemented or validated yet.

---

# UUV — BlueROV2 Heavy

The Unmanned Underwater Vehicle is based on the **Blue Robotics BlueROV2 Heavy**.

The UUV software is intentionally separated into two independent ROS 2 workspaces:

```text
UUV_ws/
├── real_ws/
└── simulation_ws/
```

This allows the physical vehicle and the simulated vehicle to evolve independently while preserving similar control and mission interfaces.

> **Important:**  
> The real and simulation environments contain some ROS 2 packages with identical names.  
> Therefore, they must be built independently and the repository root should not be used as a single colcon workspace.

---

## UUV Real Hardware

```text
UUV_ws/real_ws/src/
├── bringup/
│   └── uuv_bringup/
├── control/
│   ├── camera_control/
│   ├── rov_control/
│   └── uuv_teleop/
├── drivers/
│   ├── bluerov2_camera/
│   ├── ping360_ros2/
│   └── uuv_mavlink/
├── experimental/
├── perception/
├── tasks/
└── third_party/
```

---

## UUV Bringup

```text
bringup/
└── uuv_bringup/
```

The `uuv_bringup` package provides a centralized launch file for the physical BlueROV2 Heavy.

Current real-hardware bringup includes support for:

- Ping360 imaging sonar
- Ping360 point-cloud filtering
- BlueROV2 video stream
- Automatic `rqt_image_view` camera viewer
- Camera tilt control
- MAVLink telemetry
- Xbox teleoperation
- MAVLink manual control
- ARM/DISARM control
- Acoustic armed-state feedback
- Legacy ROV velocity control

Individual subsystems can be enabled or disabled using ROS 2 launch arguments.

---

## UUV Drivers

```text
drivers/
├── bluerov2_camera/
├── ping360_ros2/
└── uuv_mavlink/
```

### `bluerov2_camera`

Receives and publishes the BlueROV2 video stream.

### `ping360_ros2`

ROS 2 interface for the **Blue Robotics Ping360 Imaging Sonar**.

The sonar measurements are converted into ROS 2 point-cloud data for visualization and higher-level processing.

### `uuv_mavlink`

ROS 2 ↔ MAVLink interface for the physical BlueROV2 Heavy.

The current implementation communicates with BlueOS/ArduSub through UDP port `14552` and provides both telemetry and guarded manual-control transmission.

Current tested MAVLink configuration:

```text
UDP port             14552
ROS source SYSID     255
Vehicle SYSID        1
Autopilot COMPID     1
ArduSub              4.5.7
```

Main telemetry topics:

```text
/uuv/connected
/uuv/armed
/uuv/mode
/uuv/power/voltage
/uuv/power/current
/uuv/power/motors_enabled
```

Control/safety topics:

```text
/uuv/control/command_output_enabled
/uuv/control/mavlink_motion_allowed
/uuv/control/arm_request
/uuv/control/disarm_request
```

The propulsion-power state is inferred from the measured propulsion bus voltage. It is not a direct digital reading of the physical killswitch.

Current hysteresis thresholds:

```text
Voltage >= 11 V   -> propulsion power enabled
Voltage <= 8 V    -> propulsion power disabled
8 V ... 11 V      -> preserve previous state
```

The bridge transmits `MANUAL_CONTROL` at 20 Hz when command output is explicitly enabled.

Physical motion is allowed only when all of the following conditions are satisfied:

```text
MAVLink connected
AND propulsion power enabled
AND RB deadman active
AND manual command is recent
AND vehicle is armed
AND vehicle mode is MANUAL
```

The manual command watchdog currently uses a `0.30 s` timeout.

`command_output` is disabled by default.

`arm_control` is also disabled by default.

### `manual_control_preview_node`

A non-actuating diagnostic node is included to validate the ROS command path and the conversion to MAVLink-style manual-control values without transmitting commands to ArduSub.

It publishes:

```text
/uuv/control/manual_control_preview
/uuv/control/input_valid
/uuv/control/motion_allowed
```

This node is useful for controller testing and safety validation before enabling physical actuation.

---

## UUV Control

```text
control/
├── camera_control/
├── rov_control/
└── uuv_teleop/
```

### `uuv_teleop`

Xbox-based manual teleoperation interface for the physical BlueROV2.

Current controller mapping:

| Control | Function |
|---|---|
| **RB (hold)** | Motion deadman |
| **X + RB (hold 1.5 s)** | ARM request |
| **B** | Immediate DISARM request |
| **LB + D-pad Up** | Select next supported ArduSub mode |
| **LB + D-pad Down** | Select previous supported ArduSub mode |
| Left stick vertical | Forward / backward |
| Left stick horizontal | Left / right |
| Right stick horizontal | Yaw |
| RT | Up |
| LT | Down |

The node publishes:

```text
/joy
        ↓
xbox_teleop_node
        ├── /uuv/cmd_vel_manual
        ├── /uuv/deadman
        ├── /uuv/control/arm_request
        ├── /uuv/control/disarm_request
        └── /uuv/control/mode_step
```

When RB is released, `/uuv/cmd_vel_manual` returns to a neutral command. Mode stepping is edge-triggered and is accepted only while the motion deadman is released and the manual controls are neutral.

If joystick messages stop arriving, the command watchdog in the MAVLink bridge prevents stale motion commands from remaining active.

### Acoustic armed-state feedback

The `audio_feedback_node` subscribes to:

```text
/uuv/armed
```

and plays different tones when ArduSub confirms an actual armed-state transition:

```text
DISARMED -> ARMED     startup / ascending tone
ARMED -> DISARMED     shutdown / descending tone
```

The sound is generated only after the state reported by the autopilot changes. An ARM request that is rejected by ArduSub therefore does not produce the armed tone.

Audio playback currently uses `paplay` on the operator computer.

### `rov_control`

Contains the previous/direct BlueROV2 velocity-control and teleoperation-related nodes.

The current Xbox/MAVLink path is implemented through `uuv_teleop` + `uuv_mavlink`.

> **Important:** Do not intentionally run multiple independent manual-control paths at the same time. In particular, avoid enabling the legacy `rov_control` actuation path while the new MAVLink Xbox command output is active.

### `camera_control`

Contains the camera tilt controller and manual camera positioning tools.

---

## UUV Perception

```text
perception/
└── ping360_filter/
```

### `ping360_filter`

Processes Ping360 point-cloud measurements before they are used by higher-level perception or mission nodes.

---

## UUV Tasks

```text
tasks/
└── pipeline_perception/
```

Contains perception and control logic developed for pipeline-related underwater missions.

Current development includes:

- Pipeline detection
- Pipeline following
- Mission-oriented pipeline logic

---

## UUV Experimental Software

```text
experimental/
└── fake_camera/
```

Contains experimental or legacy code preserved from previous development iterations.

This code is maintained for traceability but is not considered part of the primary real-hardware stack.

---

## UUV Third-Party Software

```text
third_party/
└── gscam2/
```

External software required by the project is separated from packages developed specifically by the RobotX PUCP team.

---

# UUV Simulation

The simulation environment is located in:

```text
UUV_ws/simulation_ws/src/
├── control/
├── sensors/
├── simulator/
└── tasks/
```

It allows control, perception and mission software to be evaluated before deployment on the physical BlueROV2 Heavy.

---

## Simulator

```text
simulator/
├── ardupilot_gazebo/
└── bluerov2_gz/
```

Contains:

- BlueROV2 Gazebo models
- Underwater simulation worlds
- ArduPilot/Gazebo integration
- Vehicle simulation resources

---

## Simulated Sensors

```text
sensors/
├── fake_camera/
├── fake_dvl/
└── fake_ping360/
```

These packages reproduce sensor interfaces used by the physical system.

This allows higher-level software to operate using similar ROS 2 topics regardless of whether the vehicle is real or simulated.

---

## Simulation Control

```text
control/
└── rov_control/
```

Contains the version of the ROV control software currently used with the simulated BlueROV2.

---

## Simulation Tasks

```text
tasks/
└── pipeline_perception/
```

Contains the simulated pipeline mission software, including:

- Pipeline detection
- Pipeline following
- Pipeline mission management

---

# UUV Hardware Tests

Standalone hardware validation is organized under:

```text
UUV_ws/hardware_tests/
├── camera/
├── dvl/
└── ping360/
```

These directories are intended for individual component tests before complete vehicle integration.

---

# Development Commands and Notes

Original development notes and commands are preserved under:

```text
UUV_ws/docs/commands/
├── real/
└── simulation/
```

The original files are intentionally preserved for development traceability. USBL-specific operating notes were moved out of the UUV tree to:

```text
SYSTEM_ws/docs/commands/COMANDOS_USBL
```

Clean startup documentation and ROS 2 launch files are progressively replacing the need to manually execute multiple commands.

---

# UAV

The aerial platform for RobotX 2026 is currently under evaluation.

Its workspace has already been reserved:

```text
UAV_ws/
├── hardware_tests/
└── src/
```

The UAV hardware and software architecture will be incorporated once the final aerial platform is selected.

---

# Software Stack

| Area | Technology |
|---|---|
| Robotics Middleware | ROS 2 Humble |
| Operating System | Ubuntu |
| Languages | Python, C++ |
| Autopilot | ArduPilot / ArduSub |
| Communication | MAVLink / UDP / BlueOS Serial Bridge / pymavlink / MAVROS |
| Simulation | Gazebo |
| Computer Vision | OpenCV |
| Build System | colcon |
| Version Control | Git / GitHub |

---

# Building the Workspaces

## USV

```bash
cd ~/ROS2/ROBOT_X_IMPLEMENTATION/Robotx_PUCP/USV_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## SYSTEM — Shared Services

```bash
cd ~/ROS2/ROBOT_X_IMPLEMENTATION/Robotx_PUCP/SYSTEM_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## UUV — Real Hardware

```bash
cd ~/ROS2/ROBOT_X_IMPLEMENTATION/Robotx_PUCP/UUV_ws/real_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## UUV — Simulation

```bash
cd ~/ROS2/ROBOT_X_IMPLEMENTATION/Robotx_PUCP/UUV_ws/simulation_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

> Do not run `colcon build` from `Robotx_PUCP/`.  
> Build `USV_ws`, `SYSTEM_ws`, `UUV_ws/real_ws` and `UUV_ws/simulation_ws` independently as required.

---

# Running the Main Systems

## SYSTEM — SeaTrac USBL Bringup

After building `SYSTEM_ws`:

```bash
cd ~/ROS2/ROBOT_X_IMPLEMENTATION/Robotx_PUCP/SYSTEM_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Fresh-water profile:

```bash
ros2 launch seatrac_ros2 usbl.launch.py environment:=freshwater
```

Sea-water profile:

```bash
ros2 launch seatrac_ros2 usbl.launch.py environment:=seawater
```

The launch starts:

```text
seatrac_udp_node
seatrac_status_node
```

The profile can be inspected without changing the X150:

```bash
ros2 param get /seatrac_udp_node salinity_ppt
ros2 param get /seatrac_udp_node auto_vos
ros2 param get /seatrac_udp_node auto_pressure_offset
ros2 param get /seatrac_udp_node acoustic_positioning_enabled
ros2 param get /seatrac_udp_node allow_persistent_save
```

Read the environmental configuration currently active in the X150:

```bash
ros2 service call \
  /seatrac/environment/read \
  std_srvs/srv/Trigger "{}"
```

Apply the selected profile to the X150 working configuration for the current session:

```bash
ros2 service call \
  /seatrac/environment/apply \
  std_srvs/srv/Trigger "{}"
```

> **SeaTrac safety notes**
>
> - Loading `freshwater` or `seawater` changes ROS 2 parameters only.
> - `environment/apply` is explicit and is intended for the current working session.
> - Persistent save is blocked by default with `allow_persistent_save=false`.
> - Do not enable acoustic positioning for out-of-water development.
> - X150 ↔ X110 ranging and relative positioning are pending in-water validation.

---

## UUV — Real Hardware Bringup

After building:

```bash
cd ~/ROS2/ROBOT_X_IMPLEMENTATION/Robotx_PUCP/UUV_ws/real_ws

source /opt/ros/humble/setup.bash

source install/setup.bash
```

The main UUV launch file is:

```bash
ros2 launch uuv_bringup uuv_real.launch.py
```

By default, the launch starts sensing and MAVLink telemetry while keeping manual actuation and ARM/DISARM control disabled.

Current default behavior:

```text
Ping360 acquisition          ON
Ping360 filtering            ON
BlueROV2 camera              ON
MAVLink telemetry            ON

Xbox teleoperation           OFF
MAVLink command output       OFF
Xbox ARM/DISARM control      OFF
ROV legacy velocity control  OFF
Camera tilt control          OFF

Manual command scale         0.20
```

---

## UUV Bringup Options

Available launch arguments can be inspected using:

```bash
ros2 launch uuv_bringup uuv_real.launch.py --show-args
```

Main options include:

```text
sonar
camera
control
camera_tilt

telemetry
xbox
command_output
command_scale
arm_control

ping360_host
ping360_port
ping360_range
ping360_threshold
ping360_angle_step
mavlink_url
```

### Telemetry-only MAVLink test

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  telemetry:=true \
  xbox:=false \
  command_output:=false \
  arm_control:=false \
  sonar:=false \
  camera:=false \
  control:=false \
  camera_tilt:=false
```

### Xbox + telemetry, without physical command output

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  telemetry:=true \
  xbox:=true \
  command_output:=false \
  arm_control:=false \
  sonar:=false \
  camera:=false \
  control:=false \
  camera_tilt:=false
```

### Xbox control of the physical BlueROV2

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  telemetry:=true \
  xbox:=true \
  command_output:=true \
  arm_control:=true \
  command_scale:=0.30 \
  sonar:=false \
  camera:=false \
  control:=false \
  camera_tilt:=false
```

The command scale limits the maximum manual command:

```text
command_scale:=0.10   10 %
command_scale:=0.30   30 %
command_scale:=0.50   50 %
command_scale:=1.00   full MANUAL_CONTROL range
```

For initial physical tests, use a reduced scale and increase it progressively after validating all axes and directions.

### Ping360 only

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  telemetry:=false \
  xbox:=false \
  sonar:=true \
  camera:=false \
  control:=false \
  camera_tilt:=false
```

### BlueROV2 camera only

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  telemetry:=false \
  xbox:=false \
  sonar:=false \
  camera:=true \
  control:=false \
  camera_tilt:=false
```

> **Safety notes**
>
> - Physical MAVLink command output is disabled by default.
> - Xbox ARM/DISARM is disabled by default.
> - Keep the physical killswitch accessible during hardware testing.
> - Do not use the legacy `rov_control` actuation path simultaneously with the new `uuv_mavlink` manual-control path.
> - Avoid having QGroundControl joystick control and ROS 2 joystick control command the vehicle simultaneously.
> - The propulsion-power topic indicates the measured propulsion bus state; it does not replace the physical emergency stop.

---

# UUV Important Individual Nodes

Although `uuv_bringup` is the preferred way to start the real UUV, individual nodes can still be executed during debugging.

## Xbox Teleoperation

```bash
ros2 run joy game_controller_node
```

```bash
ros2 run uuv_teleop xbox_teleop_node
```

Useful topics:

```bash
ros2 topic echo /uuv/deadman
ros2 topic echo /uuv/cmd_vel_manual
```

## Armed-State Audio Feedback

```bash
ros2 run uuv_teleop audio_feedback_node
```

The node listens to `/uuv/armed` and generates different startup/shutdown tones after confirmed state changes.

## MAVLink Bridge

Read-only/default execution:

```bash
ros2 run uuv_mavlink mavlink_bridge_node
```

Useful status topics:

```bash
ros2 topic echo /uuv/connected
ros2 topic echo /uuv/armed
ros2 topic echo /uuv/mode
ros2 topic echo /uuv/power/voltage
ros2 topic echo /uuv/power/current
ros2 topic echo /uuv/power/motors_enabled
```

## MANUAL_CONTROL Preview

```bash
ros2 run uuv_mavlink manual_control_preview_node
```

This node does not transmit MAVLink commands. It can be used to inspect the generated `[x, y, z, r]` values before physical actuation.

## Ping360

```bash
ros2 run ping360_ros2 ping360_pointcloud_node --ros-args \
  -p host:=192.168.2.2 \
  -p port:=9092 \
  -p range_m:=5.0 \
  -p threshold:=25 \
  -p angle_step:=4
```

Filter:

```bash
ros2 run ping360_filter ping360_filter_node --ros-args \
  -p input_topic:=/ping360/points \
  -p output_topic:=/ping360/filtered_points \
  -p first_return_only:=false
```

## BlueROV2 Camera

```bash
ros2 run bluerov2_camera video_publisher
```

The camera image can be inspected using:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Legacy BlueROV2 Motion Control

```bash
ros2 run rov_control velocity_controller_node
```

Manual keyboard teleoperation:

```bash
ros2 run rov_control keyboard_teleop_node
```

> Manual keyboard teleoperation is intentionally kept separate from the main bringup because it requires an interactive terminal.

## Camera Tilt Control

Camera controller:

```bash
ros2 run camera_control camera_tilt_controller_node
```

Manual camera keyboard control:

```bash
ros2 run camera_control camera_keyboard_node
```

---

# UUV Manual-Control Safety Architecture

The current real-UUV manual-control path is:

```text
Xbox Controller
      │
      ▼
game_controller_node
      │
     /joy
      │
      ▼
xbox_teleop_node
      │
      ├── /uuv/deadman
      ├── /uuv/cmd_vel_manual
      ├── /uuv/control/arm_request
      └── /uuv/control/disarm_request
                   │
                   ▼
          uuv_mavlink_bridge
                   │
        ┌──────────┴──────────┐
        │ Safety supervision  │
        │                     │
        │ MAVLink heartbeat   │
        │ propulsion power    │
        │ RB deadman          │
        │ command watchdog    │
        │ ARMED state         │
        │ MANUAL mode         │
        └──────────┬──────────┘
                   │
                   ▼
          MAVLink MANUAL_CONTROL
                   │
                   ▼
                ArduSub
                   │
                   ▼
            BlueROV2 Heavy
```

ARM/DISARM path:

```text
X + RB for 1.5 s
        │
        ▼
    ARM request
        │
        ▼
MAV_CMD_COMPONENT_ARM_DISARM
        │
        ▼
     ArduSub
        │
        ▼
 /uuv/armed = true
        │
        ▼
 startup audio tone
```

```text
B
│
▼
DISARM request
│
▼
MAV_CMD_COMPONENT_ARM_DISARM
│
▼
ArduSub
│
▼
/uuv/armed = false
│
▼
shutdown audio tone
```

---

# UUV Simulation — Important Nodes

Build and source the simulation workspace:

```bash
cd ~/ROS2/ROBOT_X_IMPLEMENTATION/Robotx_PUCP/UUV_ws/simulation_ws

source /opt/ros/humble/setup.bash

source install/setup.bash
```

## Simulated DVL

```bash
ros2 run fake_dvl fake_dvl_node
```

## Simulated Ping360 Bridge

```bash
ros2 run fake_ping360 gz_ping360_text_bridge
```

## Simulated Camera Bridge

```bash
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

ros2 run fake_camera gz_camera_bridge
```

## Simulation ROV Controller

```bash
ros2 run rov_control velocity_controller_node
```

## Pipeline Detector

```bash
ros2 run pipeline_perception pipeline_detector_node
```

## Pipeline Follower

```bash
ros2 run pipeline_perception pipeline_follower_node
```

## Pipeline Mission Manager

```bash
ros2 run pipeline_perception pipeline_mission_manager_node
```

---

# USV — Main Commands

Build and source the USV workspace:

```bash
cd ~/ROS2/ROBOT_X_IMPLEMENTATION/Robotx_PUCP/USV_ws

source /opt/ros/humble/setup.bash

source install/setup.bash
```

The USV already includes the `usv_bringup` package for centralized startup.

Available launch files can be inspected using:

```bash
ros2 pkg prefix usv_bringup
```

Current beacon/MAVROS bringup:

```bash
ros2 launch usv_bringup beacon_system.launch.py
```

Individual USV nodes remain available for subsystem testing through their respective packages:

```text
usv_control
usv_hardware
usv_mavlink
usv_perception
usv_teleop
```

## BlueBoat MAVLink Telemetry

The current BlueBoat MAVLink bridge is read-only. Build and source the USV workspace, then run:

```bash
ros2 run usv_mavlink mavlink_bridge_node
```

Expected startup information includes:

```text
BlueBoat MAVLink bridge started in READ-ONLY mode.
Listening on UDP 0.0.0.0:14553
Target autopilot: SYSID=2, COMPID=1
BlueBoat autopilot connected.
```

Useful status topics:

```bash
ros2 topic echo /usv/connected --once
ros2 topic echo /usv/armed --once
ros2 topic echo /usv/mode --once
ros2 topic echo /usv/power/voltage --once
ros2 topic echo /usv/power/current --once
ros2 topic echo /usv/power/remaining --once
ros2 topic echo /usv/gps/fix_type --once
ros2 topic echo /usv/gps/satellites --once
ros2 topic echo /usv/gps/latitude --once
ros2 topic echo /usv/gps/longitude --once
ros2 topic echo /usv/attitude/yaw_deg --once
```

During the first physical validation, the bridge correctly reported the BlueBoat as connected, disarmed and in `HOLD`, while battery, GPS and attitude messages were received successfully. GPS fix may remain unavailable during indoor tests, which is expected when no satellites are visible.

> **Current USV MAVLink safety state**
>
> - ROS 2 command output to ArduRover is not implemented in this bridge yet.
> - ARM/DISARM transmission is not implemented yet.
> - Mode-change transmission is not implemented yet.
> - Xbox commands are not connected to the autopilot yet.
> - QGroundControl remains available independently through UDP `14550`.

---

# ROS 2 Bringup Strategy

The project is progressively moving from manually launching each ROS 2 node toward hierarchical bringup systems.

The target architecture is:

```text
RobotX
│
├── SYSTEM Services
│   ├── Shared sensing
│   ├── USBL / relative localization
│   └── Multi-vehicle coordination
│
├── USV Bringup
│   ├── Hardware
│   ├── Perception
│   ├── Control
│   └── Safety
│
├── UUV Bringup
│   ├── Drivers
│   ├── Perception
│   ├── Control
│   ├── Safety
│   └── Tasks
│
└── UAV Bringup
    └── To be implemented
```

Eventually, a RobotX-level startup entry point will coordinate all required vehicles.

The intended workflow is:

```text
One command
     │
     ▼
RobotX Bringup
     │
     ├── SYSTEM
     ├── USV
     ├── UUV
     └── UAV
```

instead of requiring one terminal for every ROS 2 node.

---

# Planned Development

The current repository structure is designed to support future development including:

- Vehicle-specific bringup packages
- BlueBoat Xbox teleoperation with ROS-only preview before physical MAVLink actuation
- Guarded BlueBoat ARM/DISARM, mode selection and manual-control output
- Manual/autonomous command arbitration
- Mission state machines
- Higher-level safety and failsafe supervision
- Common RobotX interfaces
- Autonomous mission execution
- Real/simulation interchangeability
- SeaTrac X150 ↔ X110 acoustic ranging and relative positioning
- Multi-vehicle coordination using shared USBL localization
- Complete RobotX system bringup

---

# Development Status

| Capability | USV | UUV | UAV |
|---|:---:|:---:|:---:|
| Hardware integration | 🟡 | 🟢 | ⚪ |
| Teleoperation | 🟢 | 🟢 | ⚪ |
| Motion control | 🟢 | 🟢 | ⚪ |
| MAVLink telemetry | 🟢 | 🟢 | ⚪ |
| MAVLink manual control | 🟡 | 🟢 | ⚪ |
| Perception | 🟡 | 🟢 | ⚪ |
| Simulation | 🟡 | 🟢 | ⚪ |
| Bringup | 🟡 | 🟢 | ⚪ |
| Autonomous missions | 🟡 | 🟡 | ⚪ |
| Multi-vehicle integration | ⚪ | ⚪ | ⚪ |

Shared-system status:

| Capability | Status |
|---|:---:|
| SeaTrac X150 BlueOS UDP transport | 🟢 |
| SeaTrac SYSTEM_INFO / STATUS | 🟢 |
| Freshwater / seawater profiles | 🟢 |
| Temporary environment apply | 🟢 |
| Persistent-save protection | 🟢 |
| X150 ↔ X110 acoustic ranging | 🟡 In-water validation pending |
| USBL relative positioning | 🟡 Development |
| USV/UUV follow coordination | 🟡 Planned |

---

# Team

**RobotX PUCP Team**  
Pontificia Universidad Católica del Perú  
Lima, Peru

Developed for the **Maritime RobotX Challenge 2026**.

---

<div align="center">

### RobotX PUCP 2026

**Autonomy · Robotics · Maritime Systems**

</div>
