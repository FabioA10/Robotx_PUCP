# Agente embarcado de secuencias absolutas V7

Este agente mueve la ejecución de puntos absolutos X/Y/Z/rumbo desde la laptop a
BlueOS. La interfaz sigue siendo ROS 2, pero después de preparar y ejecutar el
plan, la Raspberry conserva los puntos y envía localmente los setpoints MAVLink.

## Política ante pérdidas de enlace

- Hasta 1,0 s sin heartbeat de la interfaz: el recorrido continúa sin reducir
  velocidad ni cambiar el punto.
- Después de 1,0 s: el plan no se cancela. Se congela el tiempo del punto, se
  captura la pose actual y se mantiene localmente en GUIDED.
- Al regresar la interfaz: se reanuda automáticamente el mismo punto.
- Si se pierde el heartbeat local de ArduSub o la posición necesaria, no se
  navega a ciegas: se detiene la ejecución y se aplica la transición segura ya
  implementada por el puente V6.
- Nunca se arma automáticamente.
- En el último punto se mantienen los tres intentos de POSHOLD. Si no se
  confirma, el objetivo final queda retenido en GUIDED esperando al operador.

## Instalación dentro de BlueOS

Hazlo con el ROV desarmado. Copia el archivo `UUV_SEQUENCE_AGENT_BLUEOS_V7.tar.gz`
a la terminal de BlueOS y ejecuta:

```bash
cd /home/pi
tar -xzf UUV_SEQUENCE_AGENT_BLUEOS_V7.tar.gz
cd uuv_sequence_agent
sudo bash install_blueos.sh
```

El instalador valida `armv7l`, Python, aiohttp, mavlink2rest, el estado desarmado
y el administrador `run-service`. Guarda copias de seguridad antes de modificar
`/usr/bin/start-blueos-core` y registra el agente para los siguientes arranques.

Para revisar el servicio:

```bash
sudo bash /usr/blueos/userdata/uuv_sequence_agent/status_agent.sh
```

Para retirarlo de forma reversible, siempre con el ROV desarmado:

```bash
sudo bash /usr/blueos/userdata/uuv_sequence_agent/uninstall_blueos.sh
```

## Inicio de la interfaz en la laptop

Después de aplicar y compilar el parche ROS 2, usa el overlay que indique el
validador y lanza explícitamente el backend embarcado:

```bash
source /opt/ros/jazzy/setup.bash
source "$OVERLAY_V7/local_setup.bash"

ros2 launch uuv_dashboard uuv_sequences.launch.py \
  sequence_backend:=embedded \
  sequence_agent_url:=http://192.168.2.2:9132 \
  enable_sequence_output:=true \
  water_density:=1000.0
```

Con `sequence_backend:=embedded`, el ejecutor GUIDED de la laptop no se inicia y
su puente deja deshabilitada la salida de secuencias. Esto evita doble autoridad.
El joystick, accesorios, cámara y sonar siguen ejecutándose en la laptop.

Para volver temporalmente al comportamiento anterior, usa
`sequence_backend:=local`. No ejecutes ambos launch al mismo tiempo.
