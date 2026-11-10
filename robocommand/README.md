# RoboCommand UUV (ArduSub)

This package integrates the RobotX RoboCommand MQTT protocol with an ArduSub
vehicle through MAVROS2. The UUV bridge publishes and consumes MAVROS topics
under `/uuv/mavros`.

## Required transport

Create a dedicated BlueOS MAVLink UDP endpoint to the surface laptop:

```text
destination host: 192.168.2.1
destination port: 14575
```

MAVROS binds `udp://:14575@`. Keep QGroundControl on its own endpoint. Stop
the Point-to-Point `uuv_mavlink` bridge, or at least its command output, before
starting this bridge; two command authorities must never control the same
ArduSub.

## Installation and launch

First complete `UUV_ws/mavros_ardusub_ws/bootstrap.sh` with administrator
credentials. The RoboCommand Python dependencies are isolated in `.venv` and
the package is built into `.install`.

```bash
cd ~/Escritorio/ROS_2/Robotx_PUCP/robocommand
./run_uuv_robocommand.sh
```

The default broker is `rc-test.local:1901`, team `PUCP`, vehicle `UUV1`.
Override only for the official competition environment, for example:

```bash
./run_uuv_robocommand.sh broker_host:=broker.example broker_port:=1883
```

## RunStart safety

`RunStart` never arms the ROV and does not change its mode by default. Before
an operator explicitly enables a mode change, the bridge requires fresh FCU
telemetry, a local MAVROS position and a depth sample. This does not replace
ArduSub's own EKF/position checks.
