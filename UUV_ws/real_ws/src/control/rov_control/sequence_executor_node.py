# Justo antes o en el instante en que envías la solicitud para cambiar a modo GUIDED:

# 1. Capturar el estado actual desde la telemetría (asegúrate de usar las variables correctas de tu nodo)
# Estas variables asumen que ya las estás guardando en callbacks de /uuv/telemetry/...
profundidad_actual = self.current_depth_estimate 
rumbo_actual = self.current_yaw_rad

# 2. Inicializar los objetivos para que coincidan exactamente con el estado actual
self.target_z = profundidad_actual
self.target_yaw = rumbo_actual

# 3. Establecer un estado inicial de "Estabilización" o "Hold"
self.modo_estabilizacion_inicial = True
self.tiempo_inicio_guided = self.get_clock().now()

# ... (Tu código para solicitar modo GUIDED a MAVLink) ...

# Luego, en tu loop de control principal (timer) o donde publiques a SET_POSITION_TARGET_LOCAL_NED:

if self.modo_estabilizacion_inicial:
    # Calcular cuánto tiempo ha pasado desde que se solicitó el modo GUIDED
    tiempo_transcurrido = (self.get_clock().now() - self.tiempo_inicio_guided).nanoseconds / 1e9
    
    if tiempo_transcurrido < 1.5:  # Mantener velocidades estrictamente en cero por 1.5 segundos
        msg = PositionTargetLocalNED()
        msg.coordinate_frame = 1 # FRAME_LOCAL_NED
        msg.type_mask = 2503     # Usar máscara 2503 (velocidades limitadas y yaw absoluto)
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vz = 0.0             # Esto evita el salto vertical (hundimiento brusco)
        msg.yaw = self.target_yaw # Mantiene el rumbo medido inicialmente
        self.publisher_setpoint.publish(msg)
        return # Salir del loop para no calcular comandos proporcionales aún
    else:
        # Una vez pasados los 1.5 segundos, salir del modo de estabilización y ejecutar la ruta real
        self.modo_estabilizacion_inicial = False

# ... (Tu lógica original de control proporcional P para la ruta sigue a partir de aquí) ...
