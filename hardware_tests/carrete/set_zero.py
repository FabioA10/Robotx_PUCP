#!/usr/bin/env python3
import sys
import can
import time

def set_zero_rom():
    MOTOR_ID = 1
    INTERFAZ_CAN = "can0"
    ARB_ID = 0x140 + MOTOR_ID

    print(f"[*] Iniciando proceso de Cero Absoluto para MG6010 (ID: {MOTOR_ID})")
    
    try:
        # Abrimos el bus directamente
        bus = can.Bus(channel=INTERFAZ_CAN, bustype="socketcan")
        
        # Comando 0x19: Quemar posición actual en la ROM
        data = bytes([0x19]) + b"\x00" * 7
        msg = can.Message(arbitration_id=ARB_ID, data=data, is_extended_id=False)
        
        print("[!] Enviando comando de escritura en ROM (0x19)...")
        bus.send(msg)
        
        # Le damos medio segundo para que la instrucción viaje y se ejecute
        time.sleep(0.5)
        bus.shutdown()
        
        print("\n==================================================")
        print("[¡ATENCIÓN!] Comando enviado correctamente.")
        print("El microcontrolador del motor ahora está congelado escribiendo la memoria.")
        print("\nPASOS A SEGUIR AHORA:")
        print("1. APAGA la fuente de alimentación (24V/48V) del motor.")
        print("2. Espera 3 segundos.")
        print("3. Vuelve a encender la fuente.")
        print("4. El motor despertará considerando esta posición como 0°.")
        print("==================================================\n")
        
    except can.CanError as e:
        print(f"\n[-] Error crítico enviando el mensaje por el bus CAN: {e}")
    except Exception as e:
        print(f"\n[-] Error inesperado: {e}")

if __name__ == "__main__":
    confirmacion = input("¿Estás seguro de que deseas establecer la posición actual como el CERO absoluto en la ROM del motor? (s/n): ")
    if confirmacion.lower() == 's':
        set_zero_rom()
    else:
        print("Operación cancelada.")