# Operación del BlueROV2

## Preparar el entorno

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
```

Use una terminal limpia. No cargue antes un `install_*` antiguo: los overlays
anteriores pueden conservar rutas absolutas y mezclar versiones de paquetes.

## Perfiles principales

| Perfil | Uso | Salida física predeterminada |
|---|---|---|
| `pool_readonly.launch.py` | Telemetría e interfaz pasiva | Ninguna |
| `pool_manual.launch.py` | Pilotaje Xbox supervisado | Deshabilitada; requiere argumentos explícitos |
| `uuv_sequences.launch.py` | Editor, biblioteca y ejecutor de secuencias | Deshabilitada; requiere `enable_sequence_output:=true` |
| `uuv_real.launch.py` | Bringup general y diagnóstico por subsistema | Deshabilitada |

Ejecute un solo perfil a la vez. Dos bridges no deben competir por
`udpin:0.0.0.0:14552`.

## Inspección pasiva

Sin cámara:

```bash
ros2 launch uuv_dashboard pool_readonly.launch.py camera:=false
```

Con el BlueROV2 y el vídeo disponibles:

```bash
ros2 launch uuv_dashboard pool_readonly.launch.py camera:=true
```

## Perfil manual

Argumentos vigentes:

| Argumento | Default | Efecto |
|---|---:|---|
| `camera` | `false` | Inicia el receptor de vídeo |
| `manual_output` | `false` | Permite MANUAL_CONTROL solicitado |
| `arm_control` | `false` | Permite ARM/DISARM |
| `accessory_control` | `false` | Permite cámara y luces |
| `mode_control` | `false` | Permite solicitudes de modo |
| `command_scale` | `0.20` | Escala máxima del mando, de 0 a 1 |

Prueba del mando sin actuación:

```bash
ros2 launch uuv_dashboard pool_manual.launch.py \
  camera:=false \
  manual_output:=false \
  arm_control:=false \
  accessory_control:=false \
  mode_control:=false \
  command_scale:=0.20
```

Operación física supervisada:

```bash
ros2 launch uuv_dashboard pool_manual.launch.py \
  camera:=true \
  manual_output:=true \
  arm_control:=true \
  accessory_control:=true \
  mode_control:=true \
  command_scale:=0.20
```

## Perfil de secuencias

| Argumento | Default | Efecto |
|---|---:|---|
| `enable_sequence_output` | `false` | Habilita GUIDED, salida manual de respaldo y ARM/DISARM |
| `enable_parameter_write` | `false` | Autoriza el protocolo restringido de escritura |
| `accessory_control` | `false` | Permite inclinación de cámara y luces |
| `camera` | `true` | Inicia vídeo |
| `sonar` | `false` | Inicia adquisición y filtro Ping360 |
| `command_scale` | `0.20` | Escala del respaldo Xbox |
| `water_density` | `1000.0` | Densidad para conversión de presión/profundidad |
| `event_log_directory` | vacío | Vacío usa `~/.ros/uuv_sequence_logs` |

Argumentos Ping360: `sonar_host=192.168.2.2`, `sonar_port=9092`,
`sonar_range=5.0`, `sonar_threshold=1`, `sonar_angle_step=4`,
`sonar_filter_radius=2.0` y `sonar_filter_min_intensity=90.0`.

La pestaña de configuración permite cambiar durante la sesión la escala
Xbox/MANUAL entre 0 y 100 %, sin reiniciar el launch. El cambio se publica en
`/uuv/control/command_scale_setpoint`, requiere `RB` liberado y se confirma en
`/uuv/control/command_scale`. Es temporal: al reiniciar vuelve al valor
`command_scale` del lanzamiento.

Ejemplo con sonar, sin movimiento:

```bash
ros2 launch uuv_dashboard uuv_sequences.launch.py \
  enable_sequence_output:=false \
  enable_parameter_write:=false \
  accessory_control:=false \
  camera:=true \
  sonar:=true \
  sonar_filter_radius:=2.0 \
  sonar_filter_min_intensity:=90.0
```

## Escritura restringida de parámetros

La interfaz expone estos cinco parámetros dentro del rango técnico completo
usado por el sistema:

| Parámetro | Intervalo permitido |
|---|---:|
| `MOT_PWM_MIN` | 1100–1499 µs |
| `MOT_PWM_MAX` | 1501–1900 µs |
| `WPNAV_SPEED_UP` | 0–1000 cm/s |
| `WPNAV_SPEED_DN` | 0–500 cm/s |
| `WPNAV_ACCEL_Z` | 0–500 cm/s² |

El control de propulsión permite `1–100 %`; `100 %` produce el intervalo
`1100–1900 µs`. `MOT_PWM_MIN` y `MOT_PWM_MAX` deben permanecer simétricos
respecto a 1500. El porcentaje es recorrido PWM, no porcentaje medido de
empuje, potencia o corriente. La
escritura se bloquea sin heartbeat reciente, con el vehículo armado, fuera de
`MANUAL`, con `RB` presionado, durante una secuencia o si no se confirma
ArduSub 4.5.7. Cada operación captura un snapshot, escribe de uno en uno,
verifica por lectura y ejecuta rollback si la verificación falla.

No habilite escritura durante movimiento:

```bash
ros2 launch uuv_dashboard uuv_sequences.launch.py \
  enable_sequence_output:=false \
  enable_parameter_write:=true \
  accessory_control:=false \
  camera:=true \
  sonar:=false
```

## Diagnóstico rápido

```bash
ros2 topic echo /uuv/connected --once
ros2 topic echo /uuv/armed --once
ros2 topic echo /uuv/mode --once
ros2 topic echo /uuv/power/voltage --once
ros2 topic echo /uuv/control/mavlink_motion_allowed --once
```

| Síntoma | Interpretación |
|---|---|
| `autopilot DISCONNECTED` | Normal sin SUB; con hardware, revise BlueOS y UDP 14552 |
| Fallo de `video_publisher` | No existe flujo UDP o la interfaz no tiene la IP esperada |
| El mando publica pero no mueve | Revise gates: salida, potencia, RB, armado, modo y watchdog |
| Cámara/luces no responden | Libere LB/RB/B y neutralice sticks y gatillos |
| POSHOLD rechazado | Compruebe validez del estimador; un ACK no garantiza cambio real |
| DVL repite contador | Puede ser un reporte en caché de BlueOS |

Finalice con `Ctrl+C` o cerrando el dashboard antes de iniciar otro perfil.
