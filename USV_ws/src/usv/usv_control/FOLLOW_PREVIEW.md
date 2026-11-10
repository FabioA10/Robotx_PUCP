# Prueba USBL y seguimiento en modo preview

Esta guía permite comprobar la cadena X150 del bote y X110/X010 del submarino,
además del seguidor del USV, sin publicar órdenes a MAVROS. El nodo
`uuv_follow_preview_node` publica la observación continua en
`/usv/follow/cmd_vel_preview` y, solamente durante `tracking` o `holding`, una
propuesta en `/usv/follow/cmd_vel_candidate`. Ninguna de las dos interfaces
arma el bote ni controla actuadores.

> La máquina de desarrollo fue comprobada con ROS 2 Jazzy. `SYSTEM_ws` y
> `USV_ws` están documentados originalmente para Humble, por lo que la prueba
> en Jazzy no sustituye la validación posterior del entorno del BlueBoat.

## 1. Compilar y cargar los overlays

```bash
cd ~/Escritorio/ROS_2/Robotx_PUCP/SYSTEM_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select seatrac_ros2
source install/setup.bash

cd ../USV_ws
colcon build --symlink-install --packages-select usv_control
source install/setup.bash
```

En cada terminal nueva, cargar los entornos en este orden:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Escritorio/ROS_2/Robotx_PUCP/SYSTEM_ws/install/setup.bash
source ~/Escritorio/ROS_2/Robotx_PUCP/USV_ws/install/setup.bash
```

Prueba unitaria directa del controlador independiente de ROS:

```bash
cd ~/Escritorio/ROS_2/Robotx_PUCP/USV_ws/src/usv/usv_control
python3 -m pytest -q \
  test/test_command_arbiter.py \
  test/test_usv_command_shadow_node.py \
  test/test_uuv_follow_shadow_launch.py \
  test/test_uuv_follow_shadow_pipeline.py \
  test/test_uuv_follow_model.py \
  test/test_uuv_follow_preview_node.py
```

No continúe si la compilación o alguna prueba falla.

## 2. Ejecutar la simulación sin hardware

El launch inicia el simulador USBL y el seguidor preview. No inicie MAVROS,
`navigation_node`, `position_control_node`, teleoperación ni el beacon durante
esta prueba.

```bash
ros2 launch usv_control uuv_follow_preview.launch.py \
  follow_enabled:=true
```

Para ensayar pérdida periódica de señal —tres segundos sin medición cada seis
segundos— use:

```bash
ros2 launch usv_control uuv_follow_preview.launch.py \
  follow_enabled:=true \
  dropout_every_s:=6.0 \
  dropout_duration_s:=3.0
```

El estado debe pasar a `lost_target` y el preview debe quedar en cero cuando
la última medición supera `fix_timeout_s=2.5`. Al reaparecer la señal se exigen
tres mediciones nuevas antes de volver a `tracking`.

`/usbl/relative_pose` usa el perfil ROS `sensor_data` (best effort), compatible
con el simulador, el replay CSV y una fuente acústica que no deba bloquearse
por retransmisiones. Los tópicos de estado y preview permanecen reliable.

### Cadena completa en modo shadow

Para probar también la frontera de arbitraje, todavía sin salida física:

```bash
ros2 launch usv_control uuv_follow_shadow.launch.py \
  follow_enabled:=false \
  shadow_enabled:=false \
  usbl_source:=fake
```

La cadena es:

```text
/usbl/relative_pose
  -> /usv/follow/cmd_vel_candidate
  -> /usv/control/cmd_vel_shadow
```

`cmd_vel_candidate` solo se publica mientras el seguidor está en `tracking` o
`holding`. El árbitro aplica timeout, conexión, killswitch, límites de velocidad
y aceleración, y un latch de aborto que exige deshabilitar antes de reintentar.
El launch no inventa estados de seguridad: exige observaciones frescas de
conexión y killswitch, no inicia MAVROS y no posee un publicador de actuadores.

Para una prueba completamente simulada, publique ambos gates desde dos
terminales adicionales. Deben mantenerse a una frecuencia mayor que el timeout
de seguridad de 0.5 s:

```bash
ros2 topic pub -r 10 /usv/connected std_msgs/msg/Bool '{data: true}'
```

```bash
ros2 topic pub -r 10 /usv/control/killswitch std_msgs/msg/Bool '{data: false}'
```

Luego habilite explícitamente ambos nodos; arrancar deshabilitado permite
limpiar cualquier latch antes de la prueba:

```bash
ros2 param set /uuv_follow_preview_node follow_enabled true
ros2 param set /usv_command_shadow_node shadow_enabled true
```

`usbl_source` es una selección exclusiva:

- `fake` inicia exactamente un `fake_usbl_node` dentro de este launch.
- `external` no inicia ninguna fuente USBL y exige exactamente un publicador
  externo, ya sea el replay normalizado o el futuro decoder real revisado.

Para usar un CSV revisado, inicie primero la única fuente externa:

```bash
ros2 launch seatrac_ros2 usbl_csv_replay.launch.py \
  csv_path:=/RUTA/ABSOLUTA/captura_normalizada.csv
```

Después inicie la cadena shadow sin simulador:

```bash
ros2 launch usv_control uuv_follow_shadow.launch.py \
  follow_enabled:=false \
  shadow_enabled:=false \
  usbl_source:=external
```

No combine el replay con `usbl_source:=fake`. Antes de habilitar el seguidor,
confirme que existe exactamente un publicador, inicie los dos gates simulados
anteriores y habilite ambos nodos:

```bash
ros2 topic info -v /usbl/relative_pose
```

Estados y salidas auditables:

```bash
ros2 topic echo /usv/control/arbiter_status
ros2 topic echo /usv/control/cmd_vel_shadow
ros2 node info /usv_command_shadow_node
```

No remapee `cmd_vel_candidate`, `cmd_vel_preview` ni `cmd_vel_shadow` hacia un
tópico MAVROS. Son interfaces de observación y aceptación de software.

## 3. Monitorizar y registrar evidencia

Abra terminales adicionales, cargue ambos overlays y ejecute:

```bash
ros2 topic hz /usbl/relative_pose
```

```bash
ros2 topic echo /usbl/relative_pose
```

```bash
ros2 topic echo /usv/follow/status
```

```bash
ros2 topic echo /usv/follow/cmd_vel_preview
```

Compruebe también que el nodo preview no publica en las salidas físicas:

```bash
ros2 node info /uuv_follow_preview_node
ros2 topic info -v /mavros/setpoint_velocity/cmd_vel
ros2 topic info -v /mavros/setpoint_velocity/cmd_vel_unstamped
```

En esta sesión ambos tópicos MAVROS deben estar ausentes o sin publicadores.
Para guardar la simulación o una futura salida real ya decodificada:

```bash
mkdir -p ~/robotx_bags
cd ~/robotx_bags
ros2 bag record \
  /usbl/relative_pose \
  /usv/follow/status \
  /usv/follow/cmd_vel_preview \
  /usv/follow/cmd_vel_candidate \
  /usv/control/cmd_vel_shadow \
  /usv/control/arbiter_status
```

Detenga el bag con `Ctrl+C` y anote junto a él la configuración, posiciones
medidas, hora, firmware y cualquier pérdida observada.

### Reproducir un bag de posición USBL

Terminal 1:

```bash
ros2 run usv_control uuv_follow_preview_node --ros-args \
  -p follow_enabled:=true \
  -p use_sim_time:=true
```

Terminal 2:

```bash
ros2 bag play RUTA_DEL_BAG --clock
```

Use este replay solamente si el bag contiene `/usbl/relative_pose`. El reloj
simulado es necesario para que las marcas históricas no se rechacen como
mediciones vencidas.

## 4. Capturar el X150 y el X110/X010 reales

No ejecute `fake_usbl_node` al mismo tiempo que el driver real. Con el X150 del
bote conectado mediante el puente UDP de BlueOS, inicie:

```bash
ros2 launch seatrac_ros2 usbl_capture.launch.py environment:=freshwater
```

En otra terminal registre las tramas antes de interpretar el protocolo:

```bash
mkdir -p ~/robotx_bags
cd ~/robotx_bags
ros2 bag record \
  /seatrac/frame_capture \
  /seatrac/frame_raw \
  /seatrac/status_raw \
  /seatrac/system_info_raw
```

`/seatrac/frame_capture` es la evidencia forense primaria: conserva bytes,
terminador, secuencia, tiempos de recepción y resultado CRC incluso para datos
inválidos o un overflow sin CRLF. `/seatrac/frame_raw` contiene sólo tramas que
ya pasaron el CRC, aunque su CID todavía pueda estar sin clasificar. Para una
captura limpia no invoque servicios de calibración, entorno o guardado durante
el ensayo.

La integración actual todavía no decodifica el posicionamiento acústico real
a `/usbl/relative_pose`. Por tanto, recibir estado del X150 o guardar tramas no
significa que el seguimiento real ya esté disponible.

Para cada posición conocida del X110/X010 registre:

- distancia y dirección medidas externamente respecto del X150;
- posición del transpondedor: adelante, atrás, izquierda y derecha;
- orientación y ubicación física del X150 en el USV;
- profundidad de ambos transductores y tensión/posición del tether;
- frecuencia, latencia, huecos, saltos y tramas rechazadas;
- firmware, identificadores acústicos y configuración de ambos SeaTrac;
- reflejos o lecturas múltiples, especialmente en la piscina de 0.5 m.

El mensaje que consume el seguidor debe llegar transformado a
`frame_id=base_link`, con `x` hacia delante, `y` hacia la izquierda y `z` hacia
arriba del USV. La pose de montaje del X150 y la conversión de ejes SeaTrac
pertenecen al adaptador USBL; el seguidor no debe adivinarlas. No conecte el
decoder real hasta verificar físicamente esa transformación, sus signos y ejes.

## 5. Criterios antes de considerar actuadores

Todos los puntos siguientes son obligatorios:

1. Compilación limpia y prueba unitaria sin fallos.
2. Los únicos tópicos de mando generados son los no actuantes
   `/usv/follow/cmd_vel_preview`, `/usv/follow/cmd_vel_candidate` y, si se
   ejecuta el shadow, `/usv/control/cmd_vel_shadow`; ninguno publica a MAVROS.
3. Existe exactamente un publicador de `/usbl/relative_pose`; nunca se ejecutan
   al mismo tiempo la fuente fake, el replay y el decoder real.
4. La posición lleva timestamp monotónico, frame correcto y covarianza
   horizontal finita, no negativa y coherente con la dispersión observada.
5. Adelante produce `x>0`, izquierda `y>0` y derecha `y<0` en todas las
   repeticiones.
6. La frecuencia real permite tres fixes consecutivos y el intervalo máximo
   normal es menor que 2.5 s. Si no se cumple, no aumente el timeout sin
   reevaluar primero la seguridad.
7. Una pérdida mayor de 2.5 s lleva a `lost_target` y a velocidades exactamente
   cero; la recuperación exige tres fixes nuevos.
8. Fixes fuera de rango, vencidos, con frame incorrecto, covarianza inválida o
   saltos imposibles producen cero.
9. El preview nunca retrocede y respeta `linear.x <= 0.20 m/s` y
   `|angular.z| <= 0.30 rad/s`.
10. Con el X110/X010 quieto durante al menos 60 s, el error radial del 95 % de
   las muestras es menor que 0.10 m, valor provisional inferior al deadband de
   0.20 m. Si la piscina produce más dispersión o multipath, el resultado es
   No-Go.
11. Existe un bag reproducible que demuestra seguimiento, pérdida, parada y
    reacquisición, y un operador revisó los signos y límites.
12. Antes de una prueba motorizada debe existir un árbitro único de comandos,
    parada manual/killswitch, gate explícito de habilitación y prueba separada
    a potencia limitada. El preview por sí solo no autoriza actuación.

Si cualquiera de estos criterios falla, mantenga el sistema en preview y use
el bag para corregir el decoder o el controlador fuera del agua.
