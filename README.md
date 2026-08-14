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

The repository is organized by robotic platform and software functionality in order to keep the system modular, maintainable and scalable as new sensors, actuators and RobotX missions are incorporated.

---

## Robotic Platforms

| Platform | Vehicle | Purpose | Status |
|:---:|---|---|:---:|
| **USV** | Blue Robotics **BlueBoat** | Autonomous surface navigation and RobotX mission execution | 🟡 Development |
| **UUV** | Blue Robotics **BlueROV2 Heavy** | Underwater perception, inspection and autonomous missions | 🟢 Real + Simulation |
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
│           ├── usv_perception/
│           └── usv_teleop/
│
├── UUV_ws/
│   ├── docs/
│   │   └── commands/
│   │       ├── real/
│   │       └── simulation/
│   │
│   ├── hardware_tests/
│   │   ├── camera/
│   │   ├── dvl/
│   │   ├── ping360/
│   │   └── usbl/
│   │
│   ├── real_ws/
│   │   └── src/
│   │       ├── bringup/
│   │       ├── control/
│   │       ├── drivers/
│   │       ├── experimental/
│   │       ├── perception/
│   │       ├── tasks/
│   │       └── third_party/
│   │
│   └── simulation_ws/
│       └── src/
│           ├── control/
│           ├── sensors/
│           ├── simulator/
│           └── tasks/
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

Contains manual teleoperation tools used during development, debugging and testing.

### `hardware_tests`

Contains standalone tests used to validate individual physical components before integrating them into the complete ROS 2 system.

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
├── control/
├── drivers/
├── experimental/
├── perception/
├── tasks/
└── third_party/
```

### Bringup

```text
bringup/
└── uuv_bringup/
```

The `uuv_bringup` package provides launch files used to start multiple BlueROV2 subsystems from a single command.

Current real-hardware bringup includes support for:

- Ping360 imaging sonar
- Ping360 point-cloud filtering
- SeaTrac USBL
- SeaTrac status processing
- BlueROV2 video stream
- Camera tilt control
- ROV velocity control

Individual subsystems can be enabled or disabled using ROS 2 launch arguments.

---

## UUV Drivers

```text
drivers/
├── bluerov2_camera/
├── ping360_ros2/
└── seatrac_ros2/
```

### `bluerov2_camera`

Receives and publishes the BlueROV2 video stream.

### `ping360_ros2`

ROS 2 interface for the **Blue Robotics Ping360 Imaging Sonar**.

The sonar measurements are converted into ROS 2 point-cloud data for visualization and higher-level processing.

### `seatrac_ros2`

ROS 2 interface for the **SeaTrac USBL** system.

Current functionality includes:

- Serial communication
- Status decoding
- Magnetometer calibration
- Environment configuration
- Persistent device settings

---

## UUV Control

```text
control/
├── camera_control/
└── rov_control/
```

### `rov_control`

Contains BlueROV2 motion-control and teleoperation-related nodes.

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
├── ping360/
└── usbl/
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

The original files are intentionally preserved for development traceability.

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
| Communication | MAVLink / MAVROS |
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
> Build each vehicle workspace independently.

---

# Running the Main Systems

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

By default, the launch starts the main sensing systems while keeping vehicle motion control disabled.

Current default behavior:

```text
Ping360 acquisition    ON
Ping360 filtering      ON
SeaTrac USBL           ON
BlueROV2 camera        ON

ROV velocity control   OFF
Camera tilt control    OFF
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
usbl
camera
control
camera_tilt

usbl_port
ping360_host
ping360_port
ping360_range
ping360_threshold
ping360_angle_step
mavlink_url
```

Example: start only the Ping360 subsystem:

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  sonar:=true \
  usbl:=false \
  camera:=false \
  control:=false \
  camera_tilt:=false
```

Example: start only the SeaTrac USBL:

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  sonar:=false \
  usbl:=true \
  camera:=false \
  control:=false \
  camera_tilt:=false
```

Example: start only the BlueROV2 camera:

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  sonar:=false \
  usbl:=false \
  camera:=true \
  control:=false \
  camera_tilt:=false
```

Example: sensing stack without vehicle actuation:

```bash
ros2 launch uuv_bringup uuv_real.launch.py \
  sonar:=true \
  usbl:=true \
  camera:=true \
  control:=false \
  camera_tilt:=false
```

> **Safety note:**  
> Vehicle motion control is intentionally disabled by default.  
> Enable physical actuation only when the vehicle is prepared for testing.

---

# UUV Important Individual Nodes

Although the bringup package is the preferred way to start the real UUV, individual nodes can still be executed during debugging.

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

---

## SeaTrac USBL

Serial communication:

```bash
ros2 run seatrac_ros2 seatrac_serial_node \
  --ros-args \
  -p port:=/dev/ttyUSB0
```

Status decoder:

```bash
ros2 run seatrac_ros2 seatrac_status_node
```

---

## BlueROV2 Camera

```bash
ros2 run bluerov2_camera video_publisher
```

The camera image can be inspected using:

```bash
ros2 run rqt_image_view rqt_image_view
```

---

## BlueROV2 Motion Control

```bash
ros2 run rov_control velocity_controller_node
```

Manual keyboard teleoperation:

```bash
ros2 run rov_control keyboard_teleop_node
```

> Manual keyboard teleoperation is intentionally kept separate from the main bringup because it requires an interactive terminal.

---

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
usv_perception
usv_teleop
```

---

# ROS 2 Bringup Strategy

The project is progressively moving from manually launching each ROS 2 node toward hierarchical bringup systems.

The target architecture is:

```text
RobotX
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
     ├── USV
     ├── UUV
     └── UAV
```

instead of requiring one terminal for every ROS 2 node.

---

# Planned Development

The current repository structure is designed to support future development including:

- Vehicle-specific bringup packages
- Manual/autonomous command arbitration
- Mission state machines
- Safety and failsafe supervision
- Common RobotX interfaces
- Autonomous mission execution
- Real/simulation interchangeability
- Multi-vehicle coordination
- Complete RobotX system bringup

---

# Development Status

| Capability | USV | UUV | UAV |
|---|:---:|:---:|:---:|
| Hardware integration | 🟡 | 🟢 | ⚪ |
| Teleoperation | 🟢 | 🟢 | ⚪ |
| Motion control | 🟢 | 🟢 | ⚪ |
| Perception | 🟡 | 🟢 | ⚪ |
| Simulation | 🟡 | 🟢 | ⚪ |
| Bringup | 🟡 | 🟡 | ⚪ |
| Autonomous missions | 🟡 | 🟡 | ⚪ |
| Multi-vehicle integration | ⚪ | ⚪ | ⚪ |

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