# Misiones y secuencias del UUV

## Objetivo

El dashboard separa la **edición** de una misión de su **ejecución física**. Es
posible construir, guardar y revisar secuencias con todas las salidas apagadas.

## Flujo recomendado

1. Abra `uuv_sequences.launch.py` con `enable_sequence_output:=false`.
2. Cree o cargue una misión de la biblioteca.
3. Configure Home y revise el marco de referencia.
4. Inserte los pasos y defina sus tolerancias.
5. Use duplicación o generación rectangular cuando reduzca errores repetitivos.
6. Revise la previsualización, duración, distancias y límites.
7. Guarde una versión nombrada; el borrador automático no sustituye una versión.
8. Conecte el vehículo y confirme telemetría válida.
9. Prepare la secuencia todavía sin salida y resuelva todos los bloqueos.
10. Habilite salida únicamente para el ensayo supervisado.

## Herramientas de edición

- Guardar, cargar, renombrar y eliminar misiones.
- Borrador automático de la edición actual.
- Insertar, duplicar, mover y eliminar pasos.
- Generar patrones rectangulares.
- Aplicar tolerancias globales y ajustar excepciones por paso.
- Previsualizar el objetivo sin transmitirlo al vehículo.

Use nombres que expresen propósito y revisión, por ejemplo
`piscina_rectangulo_1x2_v03`, en lugar de `prueba_final`.

## Estado de ejecución

La interfaz debe distinguir:

- Misión editada o con cambios sin guardar.
- Misión preparada y bloqueos pendientes.
- Solicitud enviada frente a estado confirmado por el autopiloto.
- Paso actual, progreso y criterio de asentamiento.
- Pausa, cancelación, toma manual y resultado final.

Un botón presionado o un ACK MAVLink no demuestra que el vehículo alcanzó el
modo ni el objetivo. La confirmación proviene de la telemetría real.

## Home y cancelación

Home es local a la sesión de operación y debe comprobarse antes de ejecutar un
retorno. La toma manual y la pérdida de condiciones seguras deben cancelar la
propiedad GUIDED y descartar objetivos pendientes. Valide esta transición en
agua antes de usar retorno automático.

## Abrir el editor sin SUB

```bash
ros2 launch uuv_dashboard uuv_sequences.launch.py \
  enable_sequence_output:=false \
  enable_parameter_write:=false \
  accessory_control:=false \
  camera:=false \
  sonar:=false
```

## Registro

Los eventos se guardan por defecto en `~/.ros/uuv_sequence_logs`. Puede elegir
otra carpeta:

```bash
ros2 launch uuv_dashboard uuv_sequences.launch.py \
  enable_sequence_output:=false \
  camera:=false \
  event_log_directory:="$HOME/RobotX_logs/secuencias"
```

Conserve el registro, la misión exacta y la configuración del launch para cada
ensayo físico reproducible.
