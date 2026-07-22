import time
import math
import threading
# Importamos la clase y la excepción desde tu librería corregida mg6010.py
from mg6010 import MG6010, MG6010Error

class SpoolController:
    def __init__(self, motor: MG6010, diameter_mm: float = 135.0):
        self.motor = motor
        # Cálculos mecánicos basados en el diámetro del carrete
        self.diameter_m = diameter_mm / 1000.0
        self.circumference_m = math.pi * self.diameter_m
        self.deg_per_meter = 360.0 / self.circumference_m
        
        # Variables de control dinámico
        self.target_distance = 0.0
        self.running = False
        self._control_thread = None

    def meters_to_degrees(self, meters: float) -> float:
        return meters * self.deg_per_meter

    def degrees_to_meters(self, degrees: float) -> float:
        return degrees / self.deg_per_meter

    def start_automatic_control(self):
        """Energiza el motor e inicia el bucle cerrado en un hilo secundario."""
        self.running = True
        try:
            self.motor.enable_motor()
            print("[INFO] Motor energizado con éxito (Torque ON).")
        except MG6010Error as e:
            print(f"[ERROR] No se pudo activar el motor: {e}")
            return
        
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    def stop_automatic_control(self):
        """Detiene el bucle de control y libera el torque para poder moverlo a mano."""
        self.running = False
        if self._control_thread:
            self._control_thread.join()
        try:
            self.motor.disable_motor()
            print("\n[INFO] Motor liberado con éxito (Torque OFF). Cable suelto.")
        except MG6010Error as e:
            print(f"\n[ERROR] No se pudo liberar el motor de forma segura: {e}")

    def _control_loop(self):
        # ====================================================================
        # PARÁMETROS DE SINTONIZACIÓN (Modifica estos valores en el laboratorio)
        # ====================================================================
        KP_SPEED = 320.0       # Sensibilidad: cuántos Grados/Seg acelera por cada metro de error.
        MAX_SPEED_DPS = 1500.0 # Límite físico absoluto para proteger el cable de tirones.
        MIN_SPEED_DPS = 50.0   # Velocidad mínima requerida para vencer la fricción mecánica.
        DEADBAND_M = 0.02      # Tolerancia de zona muerta (2 cm). Evita oscilaciones por oleaje menor.

        print("[CONTROL] Bucle automático de alta frecuencia (50Hz) iniciado.")
        
        while self.running:
            try:
                # 1. Leer posición angular acumulada del motor y convertir a metros lineales
                current_angle = self.motor.read_position()
                current_distance = self.degrees_to_meters(current_angle)
                
                # 2. Calcular error lineal instantáneo
                error = self.target_distance - current_distance

                # 3. Evaluar comportamiento dinámico suave
                if abs(error) <= DEADBAND_M:
                    # Si está dentro del margen de 2cm, le pide mantener la posición estática suavemente
                    target_angle = self.meters_to_degrees(self.target_distance)
                    self.motor.move_to_angle(target_angle, MIN_SPEED_DPS)
                    speed_limit = MIN_SPEED_DPS
                else:
                    # Control Proporcional: A mayor error, el límite de velocidad máxima sube automáticamente
                    calculated_speed = abs(error) * KP_SPEED
                    speed_limit = max(MIN_SPEED_DPS, min(MAX_SPEED_DPS, calculated_speed))
                    
                    # Enviar comando de posición con el límite de velocidad adaptativo
                    target_angle = self.meters_to_degrees(self.target_distance)
                    self.motor.move_to_angle(target_angle, speed_limit)

                # Mostrar telemetría interactiva en la terminal
                print(f"\r[TELEMETRÍA] Objetivo: {self.target_distance:.2f}m | Actual: {current_distance:.2f}m | Error: {error:.2f}m | Límite Vel: {speed_limit:.1f} DPS", end="")
                
                # Frecuencia de refresco estable de 50 Hz (cada 20 milisegundos)
                time.sleep(0.02)

            except MG6010Error as e:
                print(f"\n[ERROR DE COMUNICACIÓN CAN]: {e}")
                self.running = False
            except Exception as e:
                print(f"\n[ERROR EXCEPCIÓN]: {e}")
                self.running = False


if __name__ == "__main__":
    print("--- INICIANDO ENTORNO DE PRUEBAS PARA EL CARRETE (MG6010) ---")
    
    # Instanciación limpia respetando los nuevos parámetros de tu librería (interface='socketcan')
    try:
        with MG6010(motor_id=1, channel="can0", interface="socketcan") as motor:
            controller = SpoolController(motor, diameter_mm=135.0)
            
            # ====================================================================
            # FASE 1: PRUEBA DE LECTURA PASIVA (MOVIMIENTO LIBRE)
            # ====================================================================
            print("\n>>> FASE 1: Motor liberado por 5 segundos. ¡Jala el cable con la mano!")
            controller.motor.disable_motor()  # Ahora procesará la respuesta sin Timeouts.
            
            for i in range(25):
                angle = motor.read_position()
                dist = controller.degrees_to_meters(angle)
                print(f"\r[LECTURA MANUAL] Tiempo restante: {5.0 - (i*0.2):.1f}s | Distancia medida: {dist:.3f} metros", end="")
                time.sleep(0.2)
            
            # ====================================================================
            # FASE 2: SEGUIMIENTO AUTOMÁTICO DINÁMICO (SIMULACIÓN DE SUBMARINO)
            # ====================================================================
            print("\n\n>>> FASE 2: Activando seguimiento automático...")
            print("[INFO] Simulando el movimiento del submarino bajo el agua usando una onda senoidal.")
            print("[INFO] Rango simulado: Entre 1.0 y 4.0 metros de distancia. Ciclo de 15 segundos.")
            print("[INFO] Presiona Ctrl+C en cualquier momento para detener la prueba y liberar el motor.")
            
            controller.start_automatic_control()
            start_time = time.time()
            
            while True:
                elapsed = time.time() - start_time
                
                # Generamos una trayectoria continua en el tiempo que emula el oleaje y descenso del sub.
                # Nota: En el futuro, cuando uses ROS, esta línea se reemplazará por la lectura del tópico.
                controller.target_distance = 2.5 + 1.5 * math.sin(2 * math.pi * elapsed / 15.0)
                
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[INFO] Pruebas canceladas manualmente por el usuario.")
    except Exception as e:
        print(f"\n[ERROR CRÍTICO EN SCRIPT]: {e}")
    finally:
        # Asegurar bajo cualquier circunstancia que el motor quede libre al salir
        if 'controller' in locals():
            controller.stop_automatic_control()
        print("[INFO] Laboratorio cerrado de manera segura.")
