"""
Launch file: beacon_system.launch.py
Lanza MAVROS + beacon_control juntos.

Uso:
  # Conexión por UART (Raspberry Pi → Pixhawk/ArduPilot por cable serial)
  ros2 launch usv_control beacon_system.launch.py fcu_url:=/dev/ttyAMA0:57600

  # Conexión por UDP (simulador o conexión por red)
  ros2 launch usv_control beacon_system.launch.py fcu_url:=udp://0.0.0.0:14550@

  # Solo beacon (si MAVROS ya está corriendo en otra terminal)
  ros2 launch usv_control beacon_system.launch.py launch_mavros:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Argumentos configurables ────────────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        # Cambiar a /dev/ttyAMA0:57600 si conectado por UART,
        # o /dev/ttyUSB0:57600 si es por USB
        default_value='udp://0.0.0.0:14550@',
        description='URL de conexión al flight controller (FCU). '
                    'Ejemplos: /dev/ttyAMA0:57600  |  udp://0.0.0.0:14550@'
    )

    launch_mavros_arg = DeclareLaunchArgument(
        'launch_mavros',
        default_value='true',
        description='Si true, lanza MAVROS junto al beacon. '
                    'Poner false si MAVROS ya está corriendo.'
    )

    # ── Nodo MAVROS (incluye el launch propio del paquete) ──────────────────
    mavros_share   = get_package_share_directory('mavros')
    usv_share      = get_package_share_directory('usv_control')

    pluginlists_yaml = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')
    config_yaml      = os.path.join(mavros_share, 'launch', 'apm_config.yaml')

    mavros_node = Node(
        condition=IfCondition(LaunchConfiguration('launch_mavros')),
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[
            pluginlists_yaml,
            config_yaml,
            {
                'fcu_url':            LaunchConfiguration('fcu_url'),
                'gcs_url':            '',
                'target_system_id':   1,
                'target_component_id': 1,
                'fcu_protocol':       'v2.0',
            }
        ]
    )

    # ── Nodo Beacon Control ─────────────────────────────────────────────────
    # Lee /mavros/state automáticamente:
    #   flight mode MANUAL/ACRO/STEERING → luz AMARILLA + buzzer lento
    #   flight mode AUTO/GUIDED/RTL      → luz VERDE   + doble beep
    beacon_node = Node(
        package='usv_control',
        executable='beacon_control',
        name='beacon_control_node',
        output='screen',
        parameters=[{
            # GPIO (BCM numbering) — Raspberry Pi 5
            'pin_red':    17,   # Luz ROJA     (reservada)
            'pin_yellow': 27,   # Luz AMARILLA → MODO MANUAL
            'pin_green':  22,   # Luz VERDE    → MODO AUTÓNOMO
            'pin_buzzer': 23,   # Buzzer
        }]
    )

    return LaunchDescription([
        fcu_url_arg,
        launch_mavros_arg,
        mavros_node,
        beacon_node,
    ])
