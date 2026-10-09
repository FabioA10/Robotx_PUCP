# Pruebas del UUV en piscina

## Alcance

Procedimiento incremental para una piscina aproximada de 2 × 4 m. Cada etapa
debe aprobarse antes de habilitar la siguiente.

## 1. Preparación en seco

- Compile y ejecute las pruebas automatizadas.
- Inspeccione conectores, tether, batería, hélices y killswitch.
- Confirme una sola fuente de MAVLink y un solo mando activo.
- Abra la interfaz con todas las salidas desactivadas.
- Compruebe telemetría, cámara, mando y registros.
- Guarde la misión prevista y una copia de sus parámetros.

## 2. Vehículo sumergido y desarmado

```bash
ros2 launch uuv_dashboard uuv_sequences.launch.py \
  enable_sequence_output:=false \
  enable_parameter_write:=false \
  accessory_control:=true \
  camera:=true \
  sonar:=false \
  command_scale:=0.20
```

Confirme profundidad, heading, actualización DVL/EKF, voltaje y estado real de
modo. Pruebe cámara y luces con sticks, gatillos, `LB` y `RB` neutros.

## 3. Movimiento manual limitado

Use `pool_manual.launch.py` con una escala reducida. Valide por separado:

1. DISARM y neutralización.
2. Avance/retroceso.
3. Lateral izquierda/derecha.
4. Ascenso/descenso.
5. Yaw horario/antihorario.
6. Liberación de `RB` y timeout de 0,30 s.

Mantenga desplazamientos cortos y el killswitch accesible.

## 4. Hold y GUIDED

- Confirme estacionario y deriva antes de ordenar desplazamiento.
- Empiece con profundidad y yaw pequeños; continúe con distancia corta.
- Pruebe cancelación manual antes de encadenar pasos.
- Aumente longitud o velocidad solo con evidencia estable.
- Ejecute un rectángulo y retorno a Home únicamente al final.

## Registro mínimo

Anote fecha, commit, firmware, launch y argumentos, misión, parámetros,
condiciones del agua, medición externa, error final, deriva, tiempo de
asentamiento, cancelaciones y cualquier intervención del operador.

No marque una capacidad como validada porque compiló o funcionó sin SUB.
