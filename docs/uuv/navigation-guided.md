# Navegación y ejecución GUIDED

## Alcance implementado

La rama integra generación de objetivos relativos de profundidad, distancia y
yaw, propiedad de la secuencia, Home local, cancelación y retorno al modo manual.
La lógica cuenta con pruebas automatizadas, pero el desempeño físico todavía
debe medirse en agua.

## Referencias

- Profundidad: presión/estimación vertical del autopiloto y densidad configurada.
- Heading/yaw: actitud informada por ArduSub.
- Posición y velocidad local: estimador de ArduSub alimentado por DVL cuando es válido.
- Home: referencia local capturada para la sesión, no una garantía geodésica.

Antes de aceptar un objetivo, el sistema debe disponer de heartbeat fresco,
estado compatible, telemetría válida y ausencia de otra fuente de mando.

## Criterios para una validación húmeda

1. DVL sumergido y con datos nuevos durante reposo.
2. Estimación local continua y sin saltos inadmisibles.
3. Signos de profundidad, avance, lateral y yaw verificados.
4. Cambio real a GUIDED confirmado por telemetría.
5. Cancelación a MANUAL medida y objetivos anteriores descartados.
6. Tolerancias evaluadas con medición externa de piscina.
7. Deriva registrada durante asentamiento y después del objetivo.
8. Pérdida de enlace, DVL o estimador ensayada sin comportamiento inseguro.

No use un único flag EKF, una muestra de posición o la aceptación MAVLink como
evidencia suficiente. Consulte también
[`ANALISIS_GUIDED_20261007.md`](../../UUV_ws/real_ws/ANALISIS_GUIDED_20261007.md).
