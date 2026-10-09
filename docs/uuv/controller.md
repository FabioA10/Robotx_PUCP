# Mando Xbox y seguridad

## Mapeo medido

| Entrada | Solicitud ROS |
|---|---|
| `RB` | Hombre muerto `/uuv/deadman` |
| `X + RB` 1,5 s, controles neutros | `/uuv/control/arm_request` |
| `B` | `/uuv/control/disarm_request` |
| `LB + D-pad ↑/↓` | Paso de modo +1/−1 |
| Stick izquierdo Y/X | Surge / sway |
| Stick derecho X | Yaw |
| `RT` / `LT` | Ascenso / descenso |
| D-pad ↑/↓ | Velocidad de inclinación de cámara +1/−1 |
| D-pad ←/→ | Luces −1/+1 paso |

El mapeo corresponde al Xbox Series X probado por el proyecto. Verifique
`/joy` si cambia el modelo de mando o el driver.

## Prioridades y exclusiones

1. `B` tiene máxima prioridad y cancela una preparación de ARM.
2. ARM requiere `X + RB`, todos los ejes neutros y 1,5 segundos continuos.
3. Movimiento solo se publica mientras `RB` está presionado.
4. Cambio de modo requiere `LB`, `RB` liberado y controles neutros.
5. Accesorios requieren `LB`, `RB` y `B` liberados y controles neutros.

Así el D-pad puede compartir cambio de modo, cámara y luces sin ambigüedad.

## Gates del bridge

El movimiento manual físico requiere simultáneamente:

```text
salida habilitada
AND heartbeat MAVLink reciente
AND potencia de propulsión inferida
AND RB activo
AND comando reciente
AND vehículo armado
AND modo MANUAL
```

El bridge ejecuta el lazo a 20 Hz y descarta comandos con más de 0,30 s. Al
soltar `RB`, el nodo teleop publica una orden neutra.

La escala Xbox/MANUAL puede modificarse entre 0 y 100 % desde la pestaña de
configuración. Es un ajuste temporal del bridge y solo se acepta con `RB`
liberado. No cambia `MOT_PWM_MIN/MAX`; ambos límites se multiplican en capas
diferentes de la cadena de mando.

La potencia se infiere usando el voltaje del bus. Esto es una ayuda de software,
no una lectura directa del killswitch ni un sustituto de la parada física.

## Comprobación sin actuadores

```bash
ros2 launch uuv_dashboard pool_manual.launch.py \
  camera:=false \
  manual_output:=false \
  arm_control:=false \
  accessory_control:=false \
  mode_control:=false
```

En otra terminal:

```bash
ros2 topic echo /joy
ros2 topic echo /uuv/input/xbox_status
ros2 topic echo /uuv/cmd_vel_manual
ros2 topic echo /uuv/deadman
```

No use simultáneamente el joystick de QGroundControl, el camino legado de
`rov_control` y el bridge actual. Debe existir un único propietario de mando.
