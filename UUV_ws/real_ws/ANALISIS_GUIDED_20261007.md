# Análisis de transición MANUAL → GUIDED del BlueROV2 Heavy

Fecha de prueba: 7 de octubre de 2026
Rama: fix-guided-transition
Commit registrado durante la prueba: 018d331
Firmware: ArduSub 4.5.7
Controlador registrado: guided_bumpless_v3
Interfaz: uuv_dashboard con Xbox
Escala manual utilizada: 0.40

## 1. Objetivo

Conseguir que el BlueROV2 Heavy:

- Entre en GUIDED correctamente desde el primer intento.
- No genere impulsos bruscos, hundimientos ni giros inesperados.
- Capture y mantenga la posición y orientación existentes al solicitar GUIDED.
- Respete los límites configurados desde la interfaz.
- Mantenga una conducción manual suficientemente rápida y controlable.
- Reduzca la corriente y los esfuerzos sobre el SSR DC de 75 A.
- Registre suficiente información para identificar cualquier nuevo fallo.

## 2. Configuración aplicada en ArduSub

Parámetros principales verificados después del reinicio:

- MOT_PWM_MIN = 1300
- MOT_PWM_MAX = 1700
- MOT_BAT_CURR_MAX = 35 A
- MOT_BAT_CURR_TC = 0.5 s
- WPNAV_SPEED = 45 cm/s
- WPNAV_SPEED_UP = 18 cm/s
- WPNAV_SPEED_DN = 18 cm/s
- WPNAV_ACCEL = 50 cm/s²
- WPNAV_ACCEL_Z = 50 cm/s²
- WPNAV_RADIUS = 5 cm
- WPNAV_JERK = 1
- PSC_JERK_XY = 1
- PSC_JERK_Z = 5
- ATC_SLEW_YAW = 2000 cdeg/s
- ATC_RATE_Y_MAX = 20 deg/s
- ATC_ACCEL_Y_MAX = 9000 cdeg/s²
- WP_YAW_BEHAVIOR = 0

Estos parámetros permanecieron guardados después del reinicio.

## 3. Archivo analizado

Se analizó:

- prueba_guided_20261007_191242.tar.gz
- Duración: 437.13 s
- Mensajes ROS: 136288
- Mensajes MAVLink de diagnóstico: 28715
- Mensajes del Xbox: 7165
- Mensajes de velocidad manual: 7159
- Setpoints de secuencia: 1436
- Corriente y voltaje: 809 muestras
- PWM de motores: 809 muestras

La captura contiene entradas del Xbox, posición, velocidad, orientación,
profundidad, corriente, voltaje, PWM, setpoints, cambios de modo y eventos del
controlador.

## 4. Resultado de los tres ingresos a GUIDED

### Intento 1

- GUIDED confirmado a las 19:18:19.169.
- Duración antes de volver a MANUAL: 0.51 s.
- Velocidad vertical máxima: 0.181 m/s.
- Desviación vertical antes del aborto: aproximadamente 0.020 m.
- Velocidad horizontal máxima: 0.026 m/s.
- Desviación de yaw: 1.35 grados.
- Desviación máxima de PWM: 129 us desde 1500.
- Corriente máxima muestreada: 3.26 A.
- Resultado: aborto por velocidad vertical excesiva.

### Intento 2

- GUIDED confirmado a las 19:18:23.919.
- Duración antes de volver a MANUAL: 2.40 s.
- Velocidad vertical máxima: 0.092 m/s.
- Desviación vertical al dispararse el aborto: 0.121 m.
- Desviación observada antes de confirmar MANUAL: 0.176 m.
- Velocidad horizontal máxima: 0.035 m/s.
- Desviación de yaw: 1.46 grados.
- Desviación máxima de PWM: 113 us.
- Corriente máxima muestreada: 4.06 A.
- Resultado: aborto por desplazamiento vertical.

### Intento 3

- GUIDED confirmado a las 19:18:29.859.
- Velocidad vertical máxima durante los primeros cinco segundos: 0.052 m/s.
- Desviación vertical durante los primeros cinco segundos: 0.086 m.
- Velocidad horizontal máxima: 0.046 m/s.
- Desviación de yaw: 1.03 grados.
- Desviación máxima de PWM: 84 us.
- Corriente máxima muestreada: 1.22 A.
- Resultado: el sistema declaró GUIDED estable.

## 5. Conclusiones sobre la transición

La transición mejoró progresivamente:

- PWM vertical: 129 us → 113 us → 84 us.
- Velocidad vertical: 0.181 m/s → 0.092 m/s → 0.052 m/s.
- Corriente: 3.26 A → 4.06 A → 1.22 A.

La mejora percibida por el operador está respaldada por los datos.

Sin embargo, el requisito todavía no está cumplido:

- El primer intento continúa siendo brusco.
- El segundo todavía se desplaza verticalmente.
- El tercero es aceptado, pero llega a desplazarse 8.6 cm.
- Continúa siendo necesario intentar GUIDED varias veces.

## 6. Problemas descartados

La captura permite descartar:

- Setpoint de posición equivocado.
- Orientación objetivo antigua.
- Comando residual del Xbox.
- Stick vertical activado.
- Velocidad previa importante antes de GUIDED.
- Pérdida de telemetría durante la secuencia.
- Saturación completa del PWM.
- Activación del límite de 35 A.
- Corriente cercana a la capacidad nominal del SSR.

Antes de cada transición se enviaba:

- x = 0
- y = 0
- z = 500
- r = 0

Por lo tanto, el Xbox estaba en neutro.

La posición y el yaw solicitados coincidían con la pose capturada.

## 7. Causa más probable

El comportamiento es compatible con una inicialización no continua del
controlador vertical de ArduSub.

En MANUAL, ArduSub envía el throttle directamente al mezclador de motores.
Sin embargo, el controlador de posición vertical utiliza el último throttle
guardado en el controlador de actitud para inicializar el integrador Z.

Ese valor puede estar desactualizado al realizar el primer cambio desde MANUAL.
Después de un primer intento en GUIDED, el controlador conserva una corrección
vertical más cercana a la necesaria. Esto explicaría por qué los siguientes
intentos son progresivamente más suaves.

Adicionalmente, AP_Motors6DOF convierte directamente el empuje calculado a PWM.
Los parámetros MOT_SLEW_UP_TIME y MOT_SLEW_DN_TIME no aplican una rampa final
efectiva a los motores en esta configuración.

## 8. Evaluación eléctrica

Durante toda la prueba:

- Corriente máxima muestreada: 4.66 A.
- Voltaje máximo: 15.574 V.
- Voltaje mínimo: 15.119 V.
- Límite configurado: 35 A.
- Capacidad nominal indicada del SSR: 75 A.

No se registró ninguna corriente sostenida cercana a 35 A o 75 A.

El límite MOT_BAT_CURR_MAX no intervino durante la prueba. Por tanto, dicho
límite no explica la lentitud del mando.

La corriente y el PWM fueron registrados aproximadamente a 2 Hz. Esa frecuencia
no permite detectar con certeza picos eléctricos de pocos milisegundos ni el
pico producido al energizar el SSR.

El registro demuestra que el consumo durante los movimientos fue bajo, pero no
puede certificar que un transitorio eléctrico de conexión nunca supere 75 A.

## 9. Evaluación del Xbox

La escala 0.40 llegó correctamente al puente MAVLink.

Con stick completo se observó:

- MANUAL_CONTROL máximo: aproximadamente +/-400.
- Velocidad horizontal máxima: 0.135 a 0.153 m/s.
- Corriente manual máxima: aproximadamente 1 A.
- PWM manual: aproximadamente 84 a 91 us desde 1500.

La lentitud se explica por la combinación de:

1. command_scale = 0.40.
2. Rango PWM reducido de 1100–1900 a 1300–1700.

La reducción del PWM disminuye a la mitad la salida física disponible para un
mismo comando normalizado.

La escala del Xbox y la velocidad horizontal configurada para las secuencias
GUIDED son controles independientes.

## 10. Evaluación de la secuencia automática

Configuración solicitada:

- Avance: 1 m a máximo 0.45 m/s.
- Giro: 180 grados a máximo 20 deg/s.
- Movimiento lateral: 1 m a máximo 0.45 m/s.

Resultados:

### Avance

- Duración: 25.95 s.
- Desplazamiento: aproximadamente 0.992 m.
- Velocidad horizontal máxima: 0.115 m/s.
- Corriente máxima: 2.23 A.

### Giro

- Duración: 31.60 s.
- Giro conseguido: aproximadamente 177.45 grados.
- Velocidad angular máxima: 11.14 deg/s.
- Corriente máxima: 2.14 A.

### Movimiento lateral

La prueba fue cancelada antes de completarlo.

- Duración observada: 14.14 s.
- Desplazamiento: 0.440 m.
- Velocidad horizontal máxima: 0.097 m/s.
- Corriente máxima: 4.26 A.

Los valores de velocidad configurados en la interfaz funcionan como límites
máximos, no como garantía de que el ROV alcanzará esas velocidades.

## 11. Recuperación adaptativa

Se analizaron 568 muestras del controlador de secuencias.

Motivos observados:

- 100 muestras: respuesta más lenta de la esperada.
- 85 muestras: deriva transversal y respuesta lenta.
- 26 muestras: posición fuera del umbral.
- 21 muestras: deriva transversal.
- 336 muestras: sin motivo de recuperación.

El controlador estuvo aproximadamente el 41 % de la ejecución dentro de alguna
condición de recuperación.

La configuración actual utiliza:

- Tolerancia XY: 0.05 m.
- Radio de recuperación: 0.055 m.

Solo existen 5 mm entre ambos umbrales. Esto provoca cambios frecuentes entre
control de precisión y recuperación.

Además:

- recovery_speed = 0.40 m/s.
- recovery_min_speed = 0.25 m/s.
- response_ratio = 0.35.
- recovery_delay = 1.0 s.

El ROV solo alcanzó alrededor de 0.115 m/s. Por eso es clasificado
constantemente como más lento de lo esperado.

## 12. Cambios recomendados

### Mantener temporalmente

- MOT_PWM_MIN = 1300.
- MOT_PWM_MAX = 1700.
- MOT_BAT_CURR_MAX = 35.
- MOT_BAT_CURR_TC = 0.5.
- PSC_JERK_XY = 1.
- PSC_JERK_Z = 5.
- WPNAV_ACCEL = 50.
- WPNAV_ACCEL_Z = 50.

No ampliar todavía el PWM a 1250–1750 porque aumentaría también el impulso del
primer GUIDED.

### Mejorar el mando manual

Cambiar inicialmente:

- command_scale: 0.40 → 0.60.

Este cambio solo debe afectar MANUAL.

El rango físico 1300–1700 y el límite de corriente de 35 A deben permanecer
activos. Después se debe medir nuevamente velocidad, corriente y PWM.

No se recomienda subir directamente a 1.0.

### Corregir la transición dentro de ArduSub

Preparar un firmware personalizado basado exactamente en ArduSub 4.5.7 que:

- Sincronice el throttle del controlador durante MANUAL.
- Inicialice correctamente el integrador vertical al entrar en GUIDED.
- Use el throttle y PWM que realmente estaban aplicados antes del cambio.
- Incorpore una rampa específica para MANUAL → GUIDED.
- Limite inicialmente el cambio de PWM a aproximadamente 100–150 us/s.
- Aplique la rampa principalmente a los motores verticales.
- Permita que MANUAL, DISARM y el corte de emergencia actúen inmediatamente.
- Registre el integrador Z, throttle, aceleración y PWM durante la transición.

La rampa debe actuar en el firmware porque ROS no tiene acceso al resultado
final del mezclador de motores.

### Ajustar recuperación después de corregir GUIDED

Propuesta inicial:

- xy_tolerance = 0.05 m.
- recovery_radius = 0.12 m.
- recovery_speed = 0.20 a 0.25 m/s.
- recovery_min_speed = 0.08 a 0.10 m/s.
- recovery_delay = 2.0 s.
- response_ratio = 0.20.
- position_gain = 1.20, sin cambio inicial.
- velocity_damping = 2.00, sin cambio inicial.

### Mejorar el criterio de aceptación de GUIDED

Después de corregir la inicialización y la rampa:

- Velocidad vertical máxima de transición: 0.08 m/s.
- Desviación vertical máxima: 0.05 m.
- Desviación horizontal máxima: 0.08 m.
- Variación de yaw máxima: 5 grados.
- Condición estable mantenida durante 1.5 a 2.0 s.

No se deben reducir ahora los umbrales sin corregir previamente el controlador,
porque solamente se obtendrían más abortos y más intentos repetidos.

### Mejorar el registro

Registrar como mínimo:

- SYS_STATUS a 10–20 Hz.
- SERVO_OUTPUT_RAW a 20–50 Hz.
- LOCAL_POSITION_NED a 20 Hz.
- ATTITUDE a 20–50 Hz.
- Aceleración vertical a 50 Hz.
- Log interno BIN de ArduSub.
- Corriente externa con sensor Hall si se necesita certificar el SSR.

## 13. Problemas de reproducibilidad encontrados

La prueba utilizó paquetes de tres instalaciones distintas:

- uuv_mavlink desde install_release_physical.
- uuv_dashboard desde install_ui_tuning.
- uuv_teleop desde install_guided.

La cámara y Ping360 intentaron arrancar aunque debían estar desactivados.
Ping360 terminó con ConnectionRefusedError y la cámara continuó publicando.

El runtime registró la revisión guided_bumpless_v3, pero el commit indicado en
el manifiesto no permite reconstruir completamente ese mismo runtime.

Debe realizarse un build limpio y único de todos los paquetes físicos antes de
la siguiente prueba.

## 14. Plan de trabajo

Orden recomendado:

1. Guardar en Git la fuente local real que produjo esta prueba.
2. Unificar uuv_mavlink, uuv_dashboard y uuv_teleop en una instalación limpia.
3. Corregir los argumentos camera y sonar del launch.
4. Aumentar la frecuencia del diagnóstico eléctrico y de PWM.
5. Aumentar command_scale manual de 0.40 a 0.60.
6. No modificar todavía los límites PWM.
7. Preparar el parche de ArduSub para transferencia vertical sin salto.
8. Compilar y validar el firmware primero sin hélices o con máxima seguridad.
9. Realizar una prueba en agua del primer MANUAL → GUIDED después de reiniciar.
10. No repetir GUIDED si el primer intento falla.
11. Analizar el log BIN y el rosbag.
12. Recién después ajustar recuperación y velocidad de secuencia.

## 15. Criterio final de éxito

El cambio se considerará correcto solamente si:

- El primer GUIDED después de cada reinicio es suave.
- No necesita intentos previos de preparación.
- No existe hundimiento ni ascenso perceptiblemente brusco.
- No existe giro inesperado.
- Conserva la posición dentro de la tolerancia configurada.
- No produce un salto abrupto de PWM.
- El Xbox mantiene una respuesta adecuada.
- La corriente permanece dentro del límite configurado.
- MANUAL y DISARM conservan prioridad inmediata.
- El mismo resultado se repite después de varios reinicios.
