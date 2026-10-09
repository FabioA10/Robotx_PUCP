# Hardware y red del UUV

## Línea base

| Elemento | Configuración actual |
|---|---|
| Vehículo | BlueROV2 Heavy |
| Autopiloto | Navigator, ArduSub 4.5.7 |
| BlueOS | `192.168.2.2` |
| Portátil | `192.168.2.1` |
| MAVLink ROS | UDP `14552`, vehículo 1/1, origen ROS 255 |
| Cámara | H.264 RTP, UDP `5600` |
| Ping360 | `192.168.2.2:9092` |
| DVL | Water Linked A50, interno `192.168.194.95` |

## MAVLink

BlueOS debe reenviar el autopiloto al puerto `14552` del portátil. La conexión
ROS usa `udpin:0.0.0.0:14552`; no inicie dos receptores sobre el mismo endpoint.

Temas básicos:

```text
/uuv/connected
/uuv/armed
/uuv/mode
/uuv/power/voltage
/uuv/power/current
/uuv/power/motors_enabled
```

## Cámara

`bluerov2_camera` recibe el flujo y publica `/camera/image/compressed`. Sin
vehículo o sin stream, use `camera:=false`; el fallo del publicador es esperado
si se solicita vídeo inexistente.

Compruebe la IP del portátil:

```bash
ip -brief address
```

## Ping360

El perfil de secuencias inicia la adquisición y el filtro de contorno con
`sonar:=true`. Para la piscina de 2 × 4 m, el valor inicial documentado del
radio local del filtro es `2.0 m`; debe ajustarse con rosbags reales sin asumir
que sustituye una localización completa.

## DVL y EKF

El A50 alimenta a ArduSub mediante la extensión de Water Linked en BlueOS. ROS
consume principalmente la estimación del autopiloto; no debe confundirse con
un tópico raw del DVL. Un informe repetido con el mismo contador o marca de
tiempo puede ser caché y no una medida viva.

La presencia de mensajes no demuestra precisión. La validez debe establecerse
con el sensor montado y sumergido, continuidad temporal y distancias medidas.
