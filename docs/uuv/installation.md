# Instalación del workspace UUV real

## Entorno soportado

- Ubuntu 24.04.
- ROS 2 Jazzy.
- Python 3.12.
- Dependencias ROS declaradas por los paquetes del workspace.
- GStreamer/OpenCV para la cámara y `python_qt_binding`/Qt para el dashboard.

No mezcle este workspace con `UUV_ws/simulation_ws`, que conserva su entorno
Humble y paquetes con nombres coincidentes.

## Preparación inicial

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws"
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -r -y
```

Revise las dependencias antes de aceptar cambios de sistema en un equipo de
operación. `rosdep` no instala firmware ni modifica parámetros de ArduSub.

## Compilación recomendada

```bash
colcon build --symlink-install \
  --packages-up-to \
    uuv_dashboard \
    uuv_mavlink \
    uuv_teleop \
    bluerov2_camera \
    ping360_ros2 \
    ping360_filter
```

Después:

```bash
source install/local_setup.bash
ros2 pkg prefix uuv_dashboard
ros2 launch uuv_dashboard uuv_sequences.launch.py --show-args
```

## Build limpio

Los artefactos de colcon no deben versionarse. Si una instalación conserva
rutas de un directorio antiguo, cierre los procesos ROS, mueva o elimine solo
los directorios `build`, `install` y `log` de este workspace y recompile. No
borre `src` ni ejecute limpiezas recursivas desde la raíz del repositorio.

Los prefijos `build_ui_*`, `install_ui_*` y `log_ui_*` son resultados de
validaciones aisladas. No deben ser la interfaz estable de ejecución ni entrar
en commits.

## Comprobar la instalación

```bash
colcon test \
  --packages-select uuv_teleop uuv_mavlink uuv_dashboard \
  --event-handlers console_direct+

colcon test-result --verbose
```

Una compilación correcta demuestra integridad de software, no autorización
para conectar actuadores. Continúe con el perfil sin salida de
[Operación del UUV](operations.md).
