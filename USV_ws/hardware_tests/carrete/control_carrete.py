#!/usr/bin/env python3
import sys
import time
from mg6010_can import MG6010, MG6010Error

def mover_carrete(angulo_objetivo: float):
    VELOCIDAD_FIXED_DPS = 1200.0 
    MOTOR_ID = 1
    INTERFAZ_CAN = "can0"
    REDUCTION = 6

    print(f"[+] Inicializando conexión con el motor MG6010 (ID: {MOTOR_ID})...")
    motor = MG6010(motor_id=MOTOR_ID, channel=INTERFAZ_CAN, reduction=REDUCTION)
    
    try:
        motor.connect()
        # Habilitar motor (requerido para que acepte comandos de movimiento)
        motor.enable_motor()
        motor.clear_error()

        pos_actual = motor.read_position()
        print(f"[+] Posición absoluta en RAM del motor: {pos_actual:.2f}°")
        
        if abs(pos_actual - angulo_objetivo) < 1.0:
            print("[+] El motor ya se encuentra en la posición objetivo. Finalizando.")
            return

        print(f"[+] Enviando comando: Mover a {angulo_objetivo}° a {VELOCIDAD_FIXED_DPS} dps.")
        motor.move_to_angle(angulo_objetivo, max_speed_dps=VELOCIDAD_FIXED_DPS)
        
        print("[+] Monitoreando movimiento (Presiona Ctrl+C para detener de emergencia)...")
        tolerancia_grados = 1.5
        vueltas_estaticas = 0
        ultima_pos = pos_actual

        while True:
            time.sleep(0.1) 
            pos_actual = motor.read_position()
            status = motor.read_status()
            velocidad_actual = abs(status.speed_dps)

            print(f"\r-> Posición actual: {pos_actual:.2f}° | Velocidad: {velocidad_actual} dps   ", end="", flush=True)

            if abs(pos_actual - angulo_objetivo) <= tolerancia_grados and velocidad_actual == 0:
                break
            
            if abs(pos_actual - ultima_pos) < 0.2 and velocidad_actual == 0:
                vueltas_estaticas += 1
                if vueltas_estaticas > 10: 
                    break
            else:
                vueltas_estaticas = 0
            
            ultima_pos = pos_actual

        pos_final_real = motor.read_position()
        print("\n\n==================================================")
        print(f"[¡ÉXITO!] Movimiento concluido.")
        print(f"[>] POSICIÓN FINAL REAL: {pos_final_real:.2f}°")
        print("==================================================")

    except MG6010Error as e:
        print(f"\n[-] Error en el bus CAN durante la operación: {e}")
    except KeyboardInterrupt:
        print("\n\n[-] ¡Parada de emergencia ejecutada por el usuario!")
    finally:
        try:
            # STOP (0x81) detiene el eje pero NO borra la RAM multivuelta
            motor.stop()
            motor.disconnect()
            print("[+] Interfaz CAN liberada. (Motor en Stop, memoria preservada)")
        except:
            pass

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso correcto:")
        print("  python3 mover_carrete.py <angulo>")
        sys.exit(1)

    try:
        input_angulo = float(sys.argv[1])
        mover_carrete(input_angulo)
    except ValueError:
        print("[-] Error: El ángulo debe ser un número válido.")