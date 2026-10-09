# RobotX PUCP 2026

Software ROS 2 del equipo RobotX PUCP para vehículos marítimos de superficie,
submarinos y servicios compartidos. El repositorio separa cada plataforma en
su propio workspace para evitar mezclar dependencias y paquetes homónimos.

> **Estado documental:** 9 de octubre de 2026. Esta revisión corresponde a
> `fix-guided-transition`. El UUV está validado en banco a nivel de software e
> interfaz; el movimiento GUIDED, las tolerancias y la navegación con DVL aún
> requieren validación controlada en agua.

## Plataformas

| Área | Plataforma | ROS 2 | Estado resumido |
|---|---|---|---|
| `USV_ws` | Blue Robotics BlueBoat | Humble | Telemetría MAVLink de solo lectura validada; actuación pendiente |
| `UUV_ws/real_ws` | BlueROV2 Heavy | Jazzy | Telemetría, mando, cámara, accesorios, interfaz y secuencias integrados; validación húmeda pendiente |
| `UUV_ws/simulation_ws` | BlueROV2 simulado | Humble | Modelos y nodos de simulación conservados; reproducibilidad actual pendiente |
| `SYSTEM_ws` | SeaTrac X150/X110 | Humble | Transporte X150 y servicios validados en banco; posicionamiento acústico en agua pendiente |
| `UAV_ws` | Holybro X500 V2 | — | Integración ROS pendiente |

## Organización

```text
Robotx_PUCP/
├── SYSTEM_ws/             # Servicios compartidos, actualmente SeaTrac
├── USV_ws/                # BlueBoat
├── UUV_ws/
│   ├── real_ws/           # BlueROV2 físico, ROS 2 Jazzy
│   ├── simulation_ws/     # Simulación, ROS 2 Humble
│   └── docs/
├── UAV_ws/                # Reserva para integración aérea
├── docs/                  # Manuales operativos y validación
└── assets/
```

No compile la raíz del repositorio como un único workspace. En particular,
`UUV_ws/real_ws` y `UUV_ws/simulation_ws` contienen paquetes con nombres
repetidos y deben construirse y cargarse por separado.

## Inicio rápido: UUV físico

### Requisitos

- Ubuntu 24.04.
- ROS 2 Jazzy.
- Python 3.12.
- BlueROV2 Heavy con Navigator y ArduSub 4.5.7.
- Para vídeo: interfaz del portátil en `192.168.2.1` y flujo H.264 UDP `5600`.
- Para MAVLink: endpoint de BlueOS hacia el portátil por UDP `14552`.

### Compilar

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws"

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-up-to \
    uuv_dashboard \
    uuv_mavlink \
    uuv_teleop \
    bluerov2_camera \
    ping360_ros2 \
    ping360_filter
```

En cada terminal nueva:

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
```

Los directorios `install_ui_*` pertenecen a validaciones aisladas. Para el uso
permanente del repositorio se documenta el prefijo estándar `install/`.

### Abrir la interfaz sin vehículo

Este perfil permite inspeccionar la interfaz y el mando sin salida física. La
cámara se desactiva porque no existe flujo de vídeo sin el BlueROV2:

```bash
ros2 launch uuv_dashboard uuv_sequences.launch.py \
  enable_sequence_output:=false \
  accessory_control:=false \
  enable_parameter_write:=false \
  camera:=false \
  sonar:=false \
  command_scale:=0.20
```

Es normal que aparezca `BlueROV2 autopilot DISCONNECTED`. Si se inicia con
`camera:=true` sin el vehículo o sin el flujo UDP, `video_publisher` terminará
con `No se pudo abrir el flujo de video`; ese error no implica que el dashboard,
el mando o el editor de misiones estén dañados.

### Inspección con el vehículo, sin movimiento

```bash
ros2 launch uuv_dashboard uuv_sequences.launch.py \
  enable_sequence_output:=false \
  accessory_control:=true \
  enable_parameter_write:=false \
  camera:=true \
  sonar:=false \
  command_scale:=0.20
```

Este perfil permite telemetría y accesorios. No permite propulsión, ARM/DISARM
ni ejecución física de secuencias.

### Movimiento manual supervisado

> **PELIGRO:** este comando habilita movimiento y ARM/DISARM. Úselo solamente
> con el vehículo asegurado, el operador frente al killswitch y sin otra fuente
> de control —por ejemplo, joystick de QGroundControl— enviando órdenes.

```bash
ros2 launch uuv_dashboard pool_manual.launch.py \
  camera:=true \
  manual_output:=true \
  arm_control:=true \
  accessory_control:=true \
  mode_control:=true \
  command_scale:=0.20
```

### Secuencias físicas supervisadas

> **PELIGRO:** `enable_sequence_output:=true` habilita GUIDED, ARM/DISARM y el
> respaldo manual por Xbox. Empiece con parámetros de escritura desactivados.

```bash
ros2 launch uuv_dashboard uuv_sequences.launch.py \
  enable_sequence_output:=true \
  accessory_control:=true \
  enable_parameter_write:=false \
  camera:=true \
  sonar:=false \
  command_scale:=0.20
```

No active `enable_parameter_write` durante una primera prueba. La escritura de
parámetros tiene condiciones adicionales y se documenta en
[Operación del UUV](docs/uuv/operations.md).

## Mando Xbox Series X

| Entrada | Función |
|---|---|
| `RB` mantenido | Hombre muerto para movimiento |
| `X + RB` durante 1,5 s, controles neutros | Solicitud de ARM |
| `B` | Solicitud inmediata de DISARM |
| Stick izquierdo vertical/horizontal | Avance-retroceso / desplazamiento lateral |
| Stick derecho horizontal | Yaw |
| `RT` / `LT` | Subir / bajar |
| `LB + D-pad ↑/↓`, con `RB` libre | Modo siguiente/anterior |
| `D-pad ↑/↓`, con `LB` y `RB` libres | Inclinar cámara |
| `D-pad ←/→`, con `LB` y `RB` libres | Luces −/+ un paso |

La cámara y las luces solo aceptan el D-pad cuando los sticks y gatillos están
neutros y `LB`, `RB` y `B` están liberados. Detalles: [Mando y seguridad](docs/uuv/controller.md).

## Interfaz y misiones

La interfaz de secuencias proporciona:

- Estado de conexión, armado, modo, energía, navegación y bloqueos.
- Vista de cámara y sonar opcionales.
- Biblioteca para guardar, cargar, renombrar y eliminar misiones.
- Borrador automático para no perder la edición en curso.
- Inserción, duplicación y reordenamiento de pasos.
- Generación de rectángulos y aplicación de tolerancias globales.
- Home local, preparación, ejecución, pausa/cancelación y registro de eventos.
- Lectura de parámetros y escritura restringida con snapshot, verificación y rollback.

La interfaz no convierte una capacidad en validada físicamente: el estado real
del autopiloto y los bloqueos visibles siguen siendo la referencia.

## Seguridad esencial

- Las salidas físicas están deshabilitadas por defecto.
- Ejecute un solo bridge MAVLink sobre UDP `14552`.
- No combine el camino manual actual con `rov_control` legado.
- El watchdog neutraliza comandos manuales con más de `0,30 s`.
- El movimiento manual requiere conexión, potencia inferida, `RB`, comando
  reciente, vehículo armado y modo real `MANUAL`.
- La potencia de motores se infiere por voltaje; no sustituye al killswitch.
- Cerrar el dashboard detiene los procesos principales de su launch.
- Antes de movimiento autónomo, valide telemetría, DVL/EKF, ejes, parada y
  cancelación en agua, usando límites conservadores.

## Pruebas

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

colcon test \
  --packages-select uuv_teleop uuv_mavlink uuv_dashboard \
  --event-handlers console_direct+

colcon test-result --verbose
```

Última validación registrada de esta integración: **146 pruebas superadas y 3
omitidas**, con seis paquetes compilados. Consulte
[Estado de validación](docs/testing/validation-status.md) para conocer el alcance
y lo que todavía no ha sido demostrado en agua.

## Documentación

| Documento | Contenido |
|---|---|
| [Instalación del UUV](docs/uuv/installation.md) | Build limpio, dependencias y separación de entornos |
| [Operación del UUV](docs/uuv/operations.md) | Perfiles, argumentos, parámetros y diagnóstico |
| [Mando y seguridad](docs/uuv/controller.md) | Mapeo Xbox, gates y respuesta segura |
| [Misiones y secuencias](docs/uuv/missions-and-sequences.md) | Flujo de creación y ejecución |
| [Hardware y red](docs/uuv/hardware.md) | BlueOS, MAVLink, cámara, Ping360 y DVL |
| [GUIDED y navegación](docs/uuv/navigation-guided.md) | Referencias, telemetría y validación pendiente |
| [Pruebas de piscina](docs/uuv/pool-testing.md) | Lista de comprobación y progresión segura |
| [Simulación del UUV](docs/uuv/simulation.md) | Workspace Humble y nodos preservados |
| [Operación del USV](docs/usv/operations.md) | BlueBoat y telemetría de solo lectura |
| [SeaTrac](docs/system/seatrac.md) | Arquitectura y servicios compartidos |
| [Estado de validación](docs/testing/validation-status.md) | Evidencia y límites de las pruebas |
| [Contribución](CONTRIBUTING.md) | Compilación, pruebas y flujo Git |
| [Historial](CHANGELOG.md) | Hitos sin convertir el README en una bitácora |

Los análisis especializados existentes se mantienen junto a su implementación,
por ejemplo [ANALISIS_GUIDED_20261007.md](UUV_ws/real_ws/ANALISIS_GUIDED_20261007.md)
y la documentación del [filtro Ping360](UUV_ws/real_ws/src/perception/ping360_filter/README.md).

## Licencia y equipo

Consulte las licencias de cada paquete y de los componentes de terceros antes
de redistribuirlos. Proyecto desarrollado por el equipo RobotX PUCP 2026.
