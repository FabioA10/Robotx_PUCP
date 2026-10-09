# Simulación del UUV

## Separación de entorno

La simulación se conserva en `UUV_ws/simulation_ws` y usa ROS 2 Humble. No
cargue simultáneamente el overlay Jazzy del vehículo real.

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/simulation_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Los modelos de BlueROV2 y BlueROV2 Heavy, bridges de sensores y tareas de
pipeline están preservados. La documentación actual no afirma que todo el
simulador haya sido revalidado después de los cambios del UUV real.

## Nodos de desarrollo

```bash
ros2 run fake_dvl fake_dvl_node
ros2 run fake_ping360 gz_ping360_text_bridge
```

Para el bridge de cámara, cuando la versión instalada de protobuf lo requiera:

```bash
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
ros2 run fake_camera gz_camera_bridge
```

Otros nodos conservados:

```bash
ros2 run rov_control velocity_controller_node
ros2 run pipeline_perception pipeline_detector_node
ros2 run pipeline_perception pipeline_follower_node
ros2 run pipeline_perception pipeline_mission_manager_node
```

Estos comandos requieren que el mundo y los bridges correspondientes estén
activos. Verifique launch files y tópicos de la versión instalada antes de usar
la simulación como evidencia de una prueba física.
