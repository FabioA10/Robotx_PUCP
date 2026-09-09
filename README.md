<div align="center">

# RobotX PUCP 2026

### Autonomous Maritime Robotics Software Stack

**Pontificia Universidad Católica del Perú**

ROS 2 · MAVLink · BlueOS · Perception · Control · Marine Robotics

</div>

---

## Project scope

This repository contains the RobotX PUCP software stack for the Maritime RobotX Challenge 2026. It is organized by vehicle and shared services so the USV, UUV and future UAV can be developed independently.

The currently validated migration is the UUV real-hardware workspace:

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- BlueROV2 Heavy with Navigator and ArduSub 4.5.7
- BlueOS MAVLink transport
- Water Linked DVL A50
- Xbox teleoperation and Qt dashboard

The USV and shared SYSTEM workspaces remain in their previous development state. UUV Jazzy validation does not mean that those workspaces have already migrated to Jazzy.

## Platform status

| Platform | Vehicle | Status |
|:---:|---|---|
| USV | Blue Robotics BlueBoat | Development; read-only MAVLink and SeaTrac work validated separately |
| UUV | Blue Robotics BlueROV2 Heavy | Real ROS 2 Jazzy telemetry, camera, dashboard, Xbox input and DVL/EKF integration validated; wet-motion tests pending |
| UAV | To be selected | Workspace reserved; implementation pending |

Status symbols: 🟢 integrated in the stated scope · 🟡 development or physical validation pending · ⚪ not started.

---

## Repository layout

~~~text
Robotx_PUCP/
├── USV_ws/                 # BlueBoat workspace
├── UUV_ws/
│   ├── real_ws/             # BlueROV2 real-hardware workspace
│   ├── simulation_ws/       # UUV simulation workspace
│   ├── hardware_tests/      # Camera, DVL and Ping360 tests
│   └── docs/commands/       # UUV notes
├── SYSTEM_ws/               # Shared SeaTrac/USBL services
├── UAV_ws/                  # Reserved UAV workspace
├── docs/
├── assets/
└── README.md
~~~

The real and simulation UUV workspaces are separate colcon workspaces. Do not build from the repository root and do not source both UUV installs in one shell.

---

# UUV — BlueROV2 Heavy

## Hardware and network baseline

| Item | Current value |
|---|---|
| Vehicle | Blue Robotics BlueROV2 Heavy |
| Autopilot | Navigator running ArduSub 4.5.7 |
| BlueOS | 192.168.2.2 |
| ROS MAVLink UDP port | 14552 |
| Vehicle MAVLink IDs | SYSID 1, COMPID 1 |
| ROS source ID | SYSID 255 |
| DVL | Water Linked A50 |
| DVL internal address | 192.168.194.95, used by BlueOS |
| Camera topic | /camera/image/compressed |

DVL data follows this path:

~~~text
Water Linked A50
      │
      ▼
BlueOS Water Linked DVL extension
      │  VISION_POSITION_DELTA / DISTANCE_SENSOR
      ▼
ArduSub EKF3
      │
      ▼
BlueOS MAVLink UDP :14552
      │
      ▼
uuv_mavlink_bridge
      │
      ▼
ROS 2 telemetry topics
~~~

The DVL messages exposed by BlueOS use SYSID 255 and COMPID 0. They are not autopilot messages (1/1). The bridge currently publishes autopilot telemetry and does not yet expose a separate raw-DVL ROS topic.

## Milestones achieved

1. ROS 2 Jazzy verified with rclpy, python_qt_binding/PyQt and OpenCV/GStreamer.
2. Real UUV packages built with an isolated Jazzy install, avoiding the stale gscam2 underlay path.
3. Read-only MAVLink bridge validated with ArduSub heartbeat, system status, power and mode telemetry.
4. BlueROV2 H.264 camera stream received and displayed in the dashboard.
5. Xbox controller detection and ROS teleoperation topics validated.
6. Deadman, command timeout, propulsion-voltage hysteresis and mode/arm safety gates added.
7. Camera tilt and light commands tested while propulsion output remained disabled.
8. Dashboard mode requests and ArduSub command acknowledgements added.
9. BlueOS confirmed to publish DVL VISION_POSITION_DELTA and DISTANCE_SENSOR messages.
10. DVL-only EKF profile loaded. EKF status changed from 167 to 39 and local-position/local-velocity topics were observed.
11. The validated changes were merged into main in commit 33b9b92.

The DVL has not yet been validated for distance or angle accuracy in the pool. The latest bench test had the DVL out of the water, so the last BlueOS DVL report was cached and cannot be treated as a live navigation measurement.

## Current status

| Capability | Status | Notes |
|---|:---:|---|
| MAVLink connection and autopilot telemetry | 🟢 | BlueOS UDP 14552; ArduSub 1/1 |
| Camera video | 🟢 | Compressed ROS image topic |
| Dashboard state display | 🟢 | Connection, mode, armed state, battery, current and deadman |
| Lights and camera tilt | 🟢 | Accessory output can be enabled independently |
| Xbox input and deadman | 🟢 | Safe launches keep motion output disabled |
| DVL data in BlueOS | 🟢 | Source 255/0 observed |
| EKF local telemetry | 🟡 | Needs live bottom lock and wet stability test |
| Manual physical motion | 🟡 | Guarded path exists; disabled by default |
| Autonomous relative movement | ⚪ | Sequence executor not implemented |
| Set-home and return-home | ⚪ | Pending |
| MANUAL interruption of sequences | ⚪ | Pending |
| Pool distance/angle calibration | 🟡 | First wet test pending |

The system is ready for a safe in-water telemetry test, not yet for an autonomous movement mission.

---

## UUV packages

~~~text
UUV_ws/real_ws/src/
├── bringup/uuv_bringup/
├── control/uuv_teleop/
├── drivers/bluerov2_camera/
├── drivers/uuv_mavlink/
└── ui/uuv_dashboard/
~~~

The validated dashboard path uses these packages. The uuv_bringup integration launch still references optional Ping360, camera-control and legacy rov_control packages; use the dashboard launch files below for the current validated hardware test path.

### uuv_mavlink

ROS 2 to MAVLink bridge for ArduSub. It receives autopilot telemetry and contains guarded manual-control, arm/disarm, mode and accessory paths.

Default safety values:

~~~text
enable_command_output   = false
enable_arm_disarm       = false
enable_accessory_output = false
command_timeout         = 0.30 s
command_scale           = 0.20
~~~

When physical manual output is explicitly enabled, the bridge requires connection, propulsion power, deadman, recent command, armed state and MANUAL mode before transmitting motion commands.

### uuv_teleop

Xbox input node. Current mapping:

| Input | Function |
|---|---|
| RB held | Motion deadman |
| X held for 1.5 s | ARM request when arm control is enabled |
| B | DISARM request |
| LB + D-pad up/down | Step through supported vehicle modes |
| Left stick | Surge and sway |
| Right stick horizontal | Yaw |
| RT/LT | Vertical command |
| D-pad up/down without LB | Camera tilt |
| D-pad left/right | Light level step |

The node returns the motion command to neutral when RB is released or input becomes stale.

### bluerov2_camera

Receives the BlueOS H.264 RTP stream with GStreamer/OpenCV and publishes:

~~~text
/camera/image/compressed
~~~

### uuv_dashboard

Qt dashboard using python_qt_binding. It displays connection, mode, armed state, battery voltage, current, deadman state and camera video. The accessory launch adds camera tilt, lights and mode requests while keeping propulsion output and arm/disarm disabled.

The dashboard does not yet execute autonomous movement sequences. Sequence editing, set-home, relative-motion control and MANUAL interruption remain planned work.

---

## Build and source the UUV workspace

Use a clean shell when the old merged install reports a missing gscam2 local setup file:

~~~bash
cd ~/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws

env -u AMENT_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH \
    -u LD_LIBRARY_PATH \
    -u ROS_PACKAGE_PATH \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc
~~~

Inside that shell:

~~~bash
source /opt/ros/jazzy/setup.bash
cd ~/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws
colcon list
~~~

Build the validated path in an isolated install:

~~~bash
colcon --log-base log_dashboard build \
  --build-base build_dashboard \
  --install-base install_dashboard \
  --symlink-install \
  --packages-select \
    uuv_mavlink \
    uuv_dashboard \
    bluerov2_camera \
    uuv_teleop
~~~

Source the isolated install:

~~~bash
source install_dashboard/local_setup.bash
~~~

The build_dashboard, install_dashboard and log_dashboard directories are local build products and must not be committed.

---

## Validated launch profiles

### Read-only dashboard

This is the preferred first launch. It starts telemetry, camera, Xbox input and the dashboard. Propulsion command output and arm/disarm remain disabled.

~~~bash
cd ~/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws
source /opt/ros/jazzy/setup.bash
source install_dashboard/local_setup.bash

ros2 launch ./src/ui/uuv_dashboard/launch/dashboard_readonly.launch.py
~~~

### Accessories and mode requests

This profile enables camera tilt, lights and dashboard mode requests. Motion commands and arm/disarm remain disabled.

~~~bash
ros2 launch ./src/ui/uuv_dashboard/launch/dashboard_accessories.launch.py
~~~

Use it with the ROV disarmed during bench tests. Mode requests can change the autopilot mode even while propulsion output is disabled.

### Integration bringup

Inspect the integration launch arguments with:

~~~bash
ros2 launch uuv_bringup uuv_real.launch.py --show-args
~~~

Its defaults are telemetry=true, camera=true, xbox=false, command_output=false, arm_control=false, control=false and sonar=false. Optional packages must be available before enabling sonar, camera-control or legacy-control arguments. The dashboard profiles above are the currently validated launches.

---

## ROS 2 topic map

### Vehicle state and power

~~~text
/uuv/connected
/uuv/armed
/uuv/mode
/uuv/power/voltage
/uuv/power/current
/uuv/power/motors_enabled
/uuv/deadman
~~~

### Control

~~~text
/uuv/cmd_vel_manual
/uuv/control/command_output_enabled
/uuv/control/mavlink_motion_allowed
/uuv/control/arm_request
/uuv/control/disarm_request
/uuv/control/mode_request
/uuv/control/mode_step
/uuv/control/camera_tilt
/uuv/control/lights_step
~~~

### Camera and accessories

~~~text
/camera/image/compressed
/uuv/accessories/lights_percent
/uuv/joystick/camera_tilt
/uuv/joystick/lights_step
~~~

### Navigation telemetry

| Topic | Type | Meaning |
|---|---|---|
| /uuv/telemetry/local_position_ned | geometry_msgs/msg/PointStamped | Local NED x/y/z |
| /uuv/telemetry/local_velocity_ned | geometry_msgs/msg/Vector3Stamped | Local NED velocity |
| /uuv/telemetry/attitude_rpy | geometry_msgs/msg/Vector3Stamped | Roll, pitch and yaw in radians |
| /uuv/telemetry/heading_deg | std_msgs/msg/Float32 | Heading in degrees |
| /uuv/telemetry/depth_estimate_m | std_msgs/msg/Float32 | Depth estimate |
| /uuv/telemetry/relative_altitude_m | std_msgs/msg/Float32 | Relative altitude |
| /uuv/telemetry/pressure_abs_hpa | std_msgs/msg/Float32 | Absolute pressure |
| /uuv/telemetry/ekf_status_flags | std_msgs/msg/UInt32 | ArduPilot EKF status bitmask |

Useful checks:

~~~bash
ros2 node list
ros2 topic list | grep '/uuv/telemetry'
ros2 topic info -v /uuv/telemetry/local_position_ned
ros2 topic echo --once /uuv/telemetry/attitude_rpy
ros2 topic echo --once /uuv/telemetry/ekf_status_flags
~~~

A publisher count of one means the node created the topic; it does not guarantee that current messages are arriving. Use ros2 topic hz during the wet test.

---

## DVL and EKF setup

Open the Water Linked extension at:

~~~text
http://192.168.2.2/extension/waterlinkeddvl
~~~

DVL-only EKF parameters:

~~~text
AHRS_EKF_TYPE  = 3
EK3_ENABLE     = 1
VISO_TYPE      = 1
RNGFND1_TYPE   = 10
EK3_SRC1_POSXY = 6
EK3_SRC1_VELXY = 6
EK3_SRC1_POSZ  = 1
~~~

Recommended workflow:

1. Enable the DVL driver and Send range data through MAVLink.
2. Click Load parameters for DVL, not DVL+GPS.
3. Restart only the autopilot.
4. Set the vehicle location in the DVL extension when no underwater GPS is used.
5. Confirm live bottom lock before interpreting local position.

The A50 sends DVL messages through BlueOS as SYSID 255, COMPID 0. Querying IDs 1/1 for these messages returns None and does not prove a disconnect.

Read the DVL report:

~~~bash
curl -s \
  http://192.168.2.2/mavlink2rest/mavlink/vehicles/255/components/0/messages/VISION_POSITION_DELTA

curl -s \
  http://192.168.2.2/mavlink2rest/mavlink/vehicles/255/components/0/messages/DISTANCE_SENSOR
~~~

The counter and last_update fields must change while the DVL is submerged. A report obtained with the transducer in air can be a cached last message and is not a navigation measurement.

---

## First pool test

The first in-water test is telemetry only:

1. Rigidly mount the DVL with unobstructed transducers facing the pool bottom.
2. Inspect tether, penetrators, battery, emergency stop and propeller clearance.
3. Keep the ROV disarmed and start dashboard_readonly.launch.py.
4. With the vehicle stationary, verify that the DVL counter increases.
5. Check the ROS rates:

~~~bash
timeout 20 ros2 topic hz /uuv/telemetry/local_position_ned
timeout 20 ros2 topic hz /uuv/telemetry/local_velocity_ned
timeout 20 ros2 topic hz /uuv/telemetry/depth_estimate_m
~~~

6. Record EKF flags, local position, velocity, depth and heading while stationary.
7. Only after stable live telemetry is confirmed should a low-speed manual test be considered.

The A50 in air, loose in a box or without a reflecting bottom is useful for checking the BlueOS extension path but not distance accuracy. DVL velocity requires a valid reflecting surface and the dead-reckoned position can drift over time.

---

## Autonomous navigation plan

The intended autonomous interface is limited to ArduSub-controlled modes:

- ALT_HOLD for depth-controlled movements.
- POSHOLD for horizontal hold when the EKF/DVL estimate is valid.
- MANUAL always has priority and cancels an active sequence.

Planned commands are relative to the current state:

~~~text
advance 3 m
rotate +45 deg
reach a depth of 3 m below the surface
return to saved home position and heading
~~~

The controller, not the dashboard, must close the loop using DVL/EKF telemetry. A sequence must stop immediately if the vehicle enters MANUAL, loses MAVLink heartbeat, loses valid navigation telemetry, becomes disarmed or exceeds a timeout. The sequence executor, set-home state and return-home controller are not implemented in the current commit.

---

## Safety rules

- Use the read-only launch for bench and first pool tests.
- Keep command output and arm/disarm disabled until live wet telemetry is reviewed.
- Never run legacy rov_control actuation together with the new MAVLink command path.
- Do not operate QGroundControl joystick and ROS 2 joystick output simultaneously.
- Keep the physical killswitch and tether operator accessible.
- Do not interpret stale BlueOS REST data as live DVL data.
- Do not treat a ROS publisher as proof that a topic is receiving current data; check its rate.

---

# USV — BlueBoat

The BlueBoat code remains in USV_ws. Its read-only MAVLink bridge was developed separately from the UUV migration. The previously validated configuration is:

~~~text
BlueOS address       192.168.2.3
ROS UDP endpoint     14553
Vehicle SYSID        2
Autopilot COMPID     1
Firmware             ArduPilot 4.6.3 STABLE
~~~

The BlueBoat bridge currently focuses on telemetry. ARM/DISARM, mode changes and propulsion command output are not enabled by that bridge.

# SYSTEM — Shared SeaTrac services

SYSTEM_ws contains shared SeaTrac X150/USBL services. The validated path uses BlueOS serial-to-UDP transport and protects persistent settings by default. Fresh-water and sea-water profiles exist; acoustic X150↔X110 ranging and vehicle-following behavior remain pending in-water validation.

# UAV

UAV_ws is reserved for the aerial platform. Hardware selection and implementation are pending.

---

## Software stack

| Area | Current value |
|---|---|
| UUV middleware | ROS 2 Jazzy |
| UUV host OS | Ubuntu 24.04 |
| UUV language | Python 3.12 |
| Autopilot | ArduPilot/ArduSub 4.5.7 |
| Vehicle middleware | BlueOS and MAVLink |
| MAVLink library | pymavlink |
| Dashboard | Qt through python_qt_binding/PyQt |
| Video | OpenCV 4.6 and GStreamer |
| Build | colcon with ament_python packages |
| Version control | Git/GitHub |

USV and SYSTEM distributions must be checked in their own workspace documentation before being described as Jazzy-compatible.

## Git workflow

The UUV integration branch was merged into main. Current UUV work is kept on main. Temporary build directories and backup files are intentionally not versioned.

Before committing:

~~~bash
git status --short
git diff --check
~~~

Keep these build products out of commits:

~~~text
UUV_ws/real_ws/build_dashboard/
UUV_ws/real_ws/install_dashboard/
UUV_ws/real_ws/log_dashboard/
~~~

---

## Current conclusion

The UUV software stack is ready for the first controlled in-water telemetry test. Camera, dashboard, MAVLink bridge, safety gates and DVL/EKF configuration are in place. The pool test must first establish a live DVL bottom lock and stable local telemetry. Autonomous relative movement, set-home, return-home and MANUAL sequence cancellation are the next software milestones after wet validation.

---

<div align="center">

### RobotX PUCP 2026

**Autonomy · Robotics · Maritime Systems**

</div>


## Previous documentation

[Complete previous documentation: BlueBoat, SeaTrac and simulation](README_referencia_33b9b92.md). Historical reference; use the current UUV instructions above for the Jazzy dashboard.
