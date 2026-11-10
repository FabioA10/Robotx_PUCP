# MAVROS para ArduSub

Workspace independiente para comunicar ROS 2 Jazzy con el Pixhawk/ArduSub a
través de MAVLink. MAVROS no es exclusivo de drones: funciona con ArduSub y
publica las interfaces estándar `/mavros/*`.

## Instalación

Ejecuta una vez, desde una terminal normal (pedirá la contraseña de
administrador):

```bash
cd ~/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/mavros_ardusub_ws
./bootstrap.sh
```

El script instala dependencias de Jazzy, descarga los datos GeographicLib y
compila MAVROS en este workspace. No modifica `real_ws`.

## BlueOS y el ROV

En BlueOS crea un endpoint MAVLink UDP que envíe al equipo de superficie:

- IP destino: `192.168.2.1`
- Puerto destino: `14540`

No reutilices el `14552`: ese puerto pertenece al bridge `uuv_mavlink` del
proyecto. Ejecuta solo uno de los dos bridges para comandos de movimiento.

## Inicio

Cuando BlueOS esté enviando al puerto anterior:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/mavros_ardusub_ws/install/setup.bash
ros2 launch mavros_ardusub_bringup ardusub.launch.py
```

Comprueba el enlace con:

```bash
ros2 topic echo /mavros/state
```

MAVROS no soluciona por sí solo el rechazo de GUIDED: ArduSub seguirá
requiriendo profundidad, EKF y posición local válidos.
