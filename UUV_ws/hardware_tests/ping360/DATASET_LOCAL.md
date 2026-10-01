# Grabaciones locales Ping360

Las grabaciones se conservan localmente en:

`UUV_ws/real_ws/recordings/ping360/`

No se versionan en Git porque contienen archivos binarios grandes. Todas
publican `/ping360/points` como `sensor_msgs/msg/PointCloud2` y fueron
registradas con ROS 2 Jazzy el 25 de septiembre de 2026.

| Grabación | Duración | Mensajes | Uso |
|---|---:|---:|---|
| PRIMERA_PRUEBA | 35.72 s | 239 | Captura inicial |
| ESPERAR_MISMA_POSICION | 153.46 s | 1024 | Evaluación estacionaria |
| PRUEBA_GIROS | 195.92 s | 1307 | Movimiento con giros |
| PRUEBA_GIRO_2 | 235.06 s | 1568 | Segunda prueba de giros |
| PRUEBA_DESPLAZAMIENTO | 91.21 s | 609 | Desplazamiento |
| prueba_desplazamiento_2 | 86.09 s | 577 | Segunda prueba de desplazamiento |

El bag `ping360_borde_prueba` no se considera evidencia porque contiene cero
mensajes.
