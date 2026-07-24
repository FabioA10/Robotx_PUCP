import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import subprocess

# Importamos nuestra lógica core (Asegúrate de que existan en tu paquete)
from usv_perception.core.camera import BufferlessUSBCapture
from usv_perception.core.evasion import EvasionProcessor

class VisionNode(Node):
    def __init__(self):
        super().__init__('robotx_vision_evasion_node')
        
        # 1. Declarar parámetros dinámicos
        self.declare_parameter('modo_laboratorio', True)
        self.declare_parameter('cam_index', 0)
        self.declare_parameter('ip_base', '192.168.2.1') # IP predeterminada para la conexión por antena/ethernet
        self.declare_parameter('puerto_udp', 5000)
        
        self.modo_lab = self.get_parameter('modo_laboratorio').get_parameter_value().bool_value
        cam_idx = self.get_parameter('cam_index').get_parameter_value().integer_value
        ip_base = self.get_parameter('ip_base').get_parameter_value().string_value
        puerto = self.get_parameter('puerto_udp').get_parameter_value().integer_value

        # 2. Inicializar Publisher (Solo Alarma para la Navegación)
        self.pub_alarma = self.create_publisher(Float32, '/robotx/alarma_frontal', 10)
        
        # 3. Inicializar Core (Lógica aislada)
        self.camara = BufferlessUSBCapture(index=cam_idx)
        self.procesador = EvasionProcessor()
        
        # 4. Inicializar Transmisión UDP (Bypass con Subprocess para GStreamer)
        gst_cmd = [
            "gst-launch-1.0", "-q", 
            "fdsrc", "!",
            "rawvideoparse", "use-sink-caps=false", "format=bgr", "width=640", "height=360", "framerate=30/1", "!",
            "videoconvert", "!", "video/x-raw,format=I420", "!",
            "x264enc", "tune=zerolatency", "bitrate=4000", "speed-preset=ultrafast", "intra-refresh=true", "!",
            "rtph264pay", "config-interval=1", "pt=96", "!",
            "udpsink", f"host={ip_base}", f"port={puerto}", "sync=false"
        ]
        
        try:
            # Eliminamos stderr=DEVNULL para poder ver errores en la terminal si los hay
            self.out_video = subprocess.Popen(gst_cmd, stdin=subprocess.PIPE)
            self.get_logger().info(f"[+] Transmisión UDP vía Subprocess iniciada hacia {ip_base}:{puerto}")
        except Exception as e:
            self.get_logger().error(f"[-] Error al iniciar GStreamer CLI: {e}")
            self.out_video = None
        
        # 5. Timer principal (~30 Hz)
        self.timer = self.create_timer(1./30., self.timer_callback)
        
        entorno_str = "LABORATORIO SECO" if self.modo_lab else "ACUATICO/MAR"
        self.get_logger().info(f"[+] Nodo USV Vision Activo. Entorno: {entorno_str}")

    def timer_callback(self):
        ret, frame_raw = self.camara.read()
        if not ret or frame_raw is None:
            return
        
        # Procesar frame
        detecciones, accion, alarma_num, output_hud = self.procesador.procesar_frame(frame_raw, modo_laboratorio=self.modo_lab)
        
        # --- Dibujar HUD Visual ---
        h_img, w_img = output_hud.shape[:2]
        y_horizonte = int(h_img * 0.45)
        y_mitad_pista = y_horizonte + int((h_img - y_horizonte) * 0.50)
        
        limite_izq_mid, limite_der_mid = self.procesador.obtener_limites_pista(y_mitad_pista, h_img, w_img)
        
        if limite_izq_mid is not None and limite_der_mid is not None:
            puntos_alerta = np.array([[int(w_img*0.40), y_horizonte], [int(w_img*0.60), y_horizonte], [limite_der_mid, y_mitad_pista], [limite_izq_mid, y_mitad_pista]], np.int32)
            puntos_criticos = np.array([[limite_izq_mid, y_mitad_pista], [limite_der_mid, y_mitad_pista], [int(w_img*0.95), h_img], [int(w_img*0.05), h_img]], np.int32)
            
            overlay = output_hud.copy()
            cv2.fillPoly(overlay, [puntos_alerta], (0, 255, 255))
            cv2.fillPoly(overlay, [puntos_criticos], (0, 0, 255))
            cv2.addWeighted(overlay, 0.15, output_hud, 0.85, 0, output_hud)

        for (x, y, w, h, id_clase, estado) in detecciones:
            color_hud = (0, 0, 255) if estado == 2 else ((0, 255, 255) if estado == 1 else (0, 255, 0))
            label = "BOYA ROJA" if id_clase == 1 else "BOYA VERDE"
            color_tag = (0, 0, 255) if id_clase == 1 else (0, 255, 0)
            
            cv2.rectangle(output_hud, (x, y), (x+w, y+h), color_hud, 2)
            cv2.putText(output_hud, f"{label}", (x, y - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_tag, 2)

        cv2.rectangle(output_hud, (0, 0), (w_img, 45), (15, 15, 15), -1)
        cv2.putText(output_hud, f"ESTADO USV: {accion}", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

        # 1. Publicar Alarma a la Navegación (Por ROS2)
        msg_alarma = Float32()
        msg_alarma.data = float(alarma_num)
        self.pub_alarma.publish(msg_alarma)

        # 2. Transmitir Video (Por UDP directo al proceso de GStreamer)
        if self.out_video and self.out_video.poll() is None: # Verifica que el proceso siga vivo
            try:
                self.out_video.stdin.write(output_hud.tobytes())
            except Exception as e:
                self.get_logger().error(f"Error escribiendo frame a GStreamer: {e}")

    def destroy_node(self):
        self.camara.release()
        if self.out_video:
            self.out_video.stdin.close()
            self.out_video.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo = VisionNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Finalizando nodo USV de forma segura...")
    finally:
        # Destruimos el nodo y validamos el contexto para evitar el error en Jazzy
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()