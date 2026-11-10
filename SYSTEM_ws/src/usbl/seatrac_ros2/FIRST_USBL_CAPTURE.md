# Primera captura USBL X150-X110/X010

Esta guía prepara evidencia reproducible para implementar el decoder de
posición real. El perfil de captura no inicia MAVROS, controladores del USV,
seguimiento ni una fuente USBL simulada. Tampoco publica
`/usbl/relative_pose`.

## 1. Frontera de seguridad

- Retire o asegure físicamente la propulsión durante la captura inicial.
- No ejecute `fake_usbl_node`, replay CSV, MAVROS, teleoperación ni control de
  posición en el mismo grafo.
- No cambie `verification.status` en `config/usbl_frame.yaml`: debe permanecer
  `unverified` hasta medir y revisar los ejes y el montaje.
- Una trama en `/seatrac/frame_raw` tiene CRC válido, pero sigue siendo evidencia
  sin semántica de posición.
- `/seatrac/frame_capture` conserva cada trama completa byte a byte, incluyendo
  `CRLF`, aun si su ASCII o CRC son inválidos. Es un `std_msgs/String` JSON con
  esquema `seatrac.byte_capture.v1`, contador, hora Unix, reloj monotónico,
  transporte, tipo de registro, bytes hexadecimales y resultado CRC. Los
  registros `unterminated_overflow` preservan también bytes sin `CRLF` que
  excedieron el límite fail-closed.

Compruebe que no existen publicadores físicos:

```bash
ros2 topic info -v /mavros/setpoint_velocity/cmd_vel
ros2 topic info -v /mavros/setpoint_velocity/cmd_vel_unstamped
```

## 2. Preparar una sesión

Use una carpeta fuera del repositorio para no versionar bags grandes:

```bash
export ROBOTX_USBL_SESSION="$HOME/robotx_bags/usbl_YYYYMMDD_HHMMSS"
mkdir -p "$ROBOTX_USBL_SESSION/raw.bag"
cp "$(ros2 pkg prefix seatrac_ros2)/share/seatrac_ros2/config/usbl_frame.yaml" \
  "$ROBOTX_USBL_SESSION/geometry.yaml"
```

Complete junto al bag un `metadata.yaml` con:

- commit Git, distribución ROS y hora local;
- modelos, firmware e identificadores SeaTrac;
- IP/puerto, ambiente, profundidad de ambos transductores y configuración del
  tether;
- posición y orientación medidas del X150 respecto de `base_link`;
- posición externa conocida del remoto en cada ensayo;
- operador y observaciones de obstrucción o multipath.

No marque como medido un valor estimado visualmente.

## 3. Iniciar únicamente el transporte real

```bash
ros2 launch seatrac_ros2 usbl_capture.launch.py environment:=freshwater
```

La consola debe indicar `actuation=false`, `fake source=false` y
`relative-pose decoder=false`.

En otra terminal:

```bash
cd "$ROBOTX_USBL_SESSION/raw.bag"
ros2 bag record \
  /seatrac/frame_capture \
  /seatrac/frame_raw \
  /seatrac/status_raw \
  /seatrac/system_info_raw \
  /diagnostics \
  /tf \
  /tf_static
```

Si `/diagnostics` o TF no existen, anótelo; no sustituya silenciosamente una
fuente de actitud o transformación ausente.

El timestamp de `frame_capture` corresponde a la recepción del fragmento que
completó el `CRLF`. `raw_bytes_hex` es la evidencia autoritativa: permite
reconstruir exactamente la trama con `bytes.fromhex(...)`. Un error de buffer
sin terminador aparece en consola, queda registrado como
`unterminated_overflow` y hace la sesión **NO-GO**. El topic conserva hasta las
100 capturas más recientes para que un rosbag iniciado tarde reciba el inicio
de la sesión; aun así, inicie el bag inmediatamente.

## 4. Geometría mínima

Capture al menos 60 segundos por posición conocida, manteniendo ambos
vehículos quietos:

1. centro/referencia;
2. adelante del USV;
3. atrás;
4. babor;
5. estribor;
6. dos rangos adicionales cuando la piscina lo permita;
7. obstrucción deliberada y recuperación.

Registre la verdad-terreno, profundidad y hora de cada segmento. Conserve el
bag original; cualquier conversión a CSV debe ser reproducible.

## 5. Integridad y entrega

Al detener el bag:

```bash
cd "$ROBOTX_USBL_SESSION"
find raw.bag -type f -print0 | sort -z | xargs -0 sha256sum > checksums.txt
git -C "$HOME/Escritorio/ROS_2/Robotx_PUCP" rev-parse HEAD > git_commit.txt
```

La sesión queda incompleta si falta el bag original, su checksum, firmware,
identificadores, geometría o verdad-terreno.

## 6. Replay normalizado, todavía sin actuación

Después de que un decoder revisado convierta una captura a las columnas
normalizadas documentadas en `USBL_INTERFACE.md`:

```bash
ros2 launch seatrac_ros2 usbl_csv_replay.launch.py \
  csv_path:=/ruta/absoluta/fixes_normalizados.csv \
  expected_beacon_id:=2
```

Este launch no inicia el seguidor. Cargue `USV_ws` y ejecute el perfil preview
o shadow por separado para mantener una sola fuente de `/usbl/relative_pose`.

El resultado es **NO-GO** ante signos incorrectos, frame sin verificar,
timestamp no monotónico, gap máximo igual o mayor al timeout, error estacionario
P95 mayor o igual al deadband, pérdida del bag o cualquier publicador físico
inesperado. También es **NO-GO** si falta `/seatrac/frame_capture`, hay saltos de
secuencia no explicados por pérdida del bag, o se reporta un descarte por límite
de buffer.
