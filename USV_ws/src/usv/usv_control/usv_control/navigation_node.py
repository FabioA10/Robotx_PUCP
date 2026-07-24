import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32 
import math
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class NavegacionAutonoma(Node):
    def __init__(self):
        super().__init__('navegacion_autonoma_node')
        
        # Suscripciones pasivas (Solo actualizan variables internas cuando llegan datos)
        self.sub_goal = self.create_subscription(Point, '/robotx/waypoint_objetivo', self.goal_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos_profile_sensor_data)
        self.sub_vision = self.create_subscription(Float32, '/robotx/alarma_frontal', self.vision_callback, 10)
        
        # El único canal que habla con los motores
        self.pub_cmd = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Variables de estado (Memoria del bote)
        self.yaw_actual = 0.0  
        self.alarma_vision = 0.0 
        self.target_dx = 0.0
        self.target_dy = 0.0
        self.tiene_waypoint = False
        
        # ====================================================================
        # NUEVO: BUCLE DE CONTROL INDEPENDIENTE A 10 Hz (0.1 segundos)
        # ====================================================================
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("=== NODO DE NAVEGACIÓN (CON LOOP REACTIVO 10Hz) ACTIVO ===")

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw_actual = math.atan2(siny_cosp, cosy_cosp)

    def vision_callback(self, msg):
        self.alarma_vision = msg.data

    def goal_callback(self, msg):
        self.target_dx = msg.x
        self.target_dy = msg.y 
        self.tiene_waypoint = True

    # Esta función se ejecuta sola 10 veces por segundo, haya o no haya waypoints
    def control_loop(self):
        cmd = Twist()
        estado = ""
        distancia = math.hypot(self.target_dx, self.target_dy) if self.tiene_waypoint else 0.0
        
        # --- 1. PRIORIDAD ABSOLUTA: EVASIÓN (Ignora al GPS si hay peligro) ---
        if self.alarma_vision > 0.0:
            estado = "[EVASIÓN VISIÓN]"
            # CAMBIO: Quitamos la restricción física de cero avance. 
            # Damos un ligero empuje (ej. 0.2 o 0.3) para vencer el arrastre lateral.
            cmd.linear.x = 0.3 
            
            if self.alarma_vision == 1.0:
                # Disminuimos un poco el requerimiento de Z para no saturar el controlador de vuelo
                cmd.angular.z = -0.8 
                estado += " -> PIVOTE DERECHO (CON AVANCE)"
            elif self.alarma_vision == 2.0:
                cmd.angular.z = 0.8
                estado += " -> PIVOTE IZQUIERDO (CON AVANCE)"
            elif self.alarma_vision == 3.0:
                # El retroceso táctico sí puede requerir Z = 0
                cmd.linear.x = -0.5 
                cmd.angular.z = 0.0
                estado += " -> RETROCESO TÁCTICO"
                
        # --- 2. CONTROL DE NAVEGACIÓN NORMAL (Solo si la cámara dice que está libre) ---
        elif self.tiene_waypoint and distancia > 0.2:
            angulo_boya_global = math.atan2(self.target_dy, self.target_dx)
            angulo_error = angulo_boya_global - self.yaw_actual
            angulo_error = math.atan2(math.sin(angulo_error), math.cos(angulo_error))

            if abs(angulo_error) < 0.20:
                cmd.angular.z = 0.0
                cmd.linear.x = 0.5
                estado = "[RECTO GPS]"
            elif abs(angulo_error) > 0.60:
                cmd.angular.z = np.clip(angulo_error * 0.8, -0.6, 0.6)
                cmd.linear.x = 0.0
                estado = "[PIVOTE GPS]"
            else:
                cmd.angular.z = np.clip(angulo_error * 0.7, -0.6, 0.6)
                cmd.linear.x = 0.5
                estado = "[CURVA GPS]"
                
        # --- 3. ESTADO DE REPOSO (Sin waypoints ni peligros) ---
        else:
            cmd.linear.x, cmd.angular.z = 0.0, 0.0
            estado = "[ESPERANDO ÓRDENES/LLEGAMOS]"

        self.get_logger().info(
            f"Fase: {estado} | Alarma IA: {self.alarma_vision} | "
            f"Vel(X): {cmd.linear.x:.1f} | Giro(Z): {cmd.angular.z:+.2f}"
        )

        # Publica la decisión en el tópico de motores a un ritmo constante de 10Hz
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavegacionAutonoma()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()