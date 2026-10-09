# Operación del BlueBoat

## Estado actual

El bridge ROS 2 del BlueBoat recibe telemetría MAVLink y permanece de solo
lectura. No transmite propulsión, ARM/DISARM ni cambios de modo.

| Elemento | Valor |
|---|---|
| BlueOS | `192.168.2.3` |
| Portátil | `192.168.2.1` |
| Endpoint QGroundControl | UDP `14550` |
| Endpoint ROS 2 | UDP `14553` |
| Autopiloto | SYSID 2, COMPID 1 |
| Firmware validado | ArduPilot 4.6.3 Stable |

## Compilar y ejecutar

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP/USV_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 run usv_mavlink mavlink_bridge_node
```

Bringup de beacon/MAVROS disponible:

```bash
ros2 launch usv_bringup beacon_system.launch.py
```

## Telemetría útil

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

La ausencia de fix GPS en interiores es esperable. No describa el USV como
controlable desde ROS hasta implementar y validar las protecciones de salida.
