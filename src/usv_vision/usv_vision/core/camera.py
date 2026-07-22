import cv2
import threading
import time
import sys

class BufferlessUSBCapture:
    """
    Captura de video USB sin buffer para minimizar la latencia.
    """
    def __init__(self, index=0, width=1280, height=720):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            print(f"[-] Error crítico: No se pudo abrir la cámara USB en el índice {index}")
            sys.exit(1)
        
        # Configuración MJPG para optimizar ancho de banda USB
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.w_nativa = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h_nativa = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        print("=" * 60)
        print(f"[+] SENSOR ÓPTICO INICIADO: {self.w_nativa}x{self.h_nativa} píxeles")
        print("=" * 60)
        
        self.frame = None
        self.ret = False
        self.running = True
        
        # Hilo demonio para vaciar el buffer
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.ret = ret
                self.frame = frame
            else:
                time.sleep(0.001)

    def read(self):
        """Retorna el frame más reciente disponible."""
        return self.ret, self.frame

    def release(self):
        """Libera los recursos de hardware."""
        self.running = False
        self.cap.release()