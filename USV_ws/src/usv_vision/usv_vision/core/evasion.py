import cv2
import numpy as np

class EvasionProcessor:
    """
    Procesador de visión para detección de boyas y cálculo de evasión.
    """
    def __init__(self):
        # El historial ahora pertenece a la instancia, no es global
        self.historial_detecciones = []

    def obtener_limites_pista(self, y, h_img, w_img):
        y_horizonte = int(h_img * 0.45) 
        x_top_izq, x_top_der = int(w_img * 0.40), int(w_img * 0.60)
        x_bot_izq, x_bot_der = int(w_img * 0.05), int(w_img * 0.95)
        
        if y < y_horizonte:
            return None, None
            
        factor = (y - y_horizonte) / (h_img - y_horizonte)
        limite_izq = int(x_top_izq + (x_bot_izq - x_top_izq) * factor)
        limite_der = int(x_top_der + (x_bot_der - x_top_der) * factor)
        
        return limite_izq, limite_der

    def agrupar_bloques_cercanos(self, candidatos, max_dist=50):
        if not candidatos: return []
        agrupados = []
        usados = set()
        for i in range(len(candidatos)):
            if i in usados: continue
            x1, y1, w1, h1, cls1, y_base1 = candidatos[i]
            x_min, y_min = x1, y1
            x_max, y_max = x1 + w1, y1 + h1
            for j in range(i + 1, len(candidatos)):
                if j in usados or candidatos[j][4] != cls1: continue
                x2, y2, w2, h2, cls2, y_base2 = candidatos[j]
                dist = np.hypot((x1 + w1/2) - (x2 + w2/2), (y1 + h1/2) - (y2 + h2/2))
                if dist < max_dist:
                    x_min = min(x_min, x2); y_min = min(y_min, y2)
                    x_max = max(x_max, x2 + w2); y_max = max(y_max, y2 + h2)
                    usados.add(j)
            w_f = x_max - x_min
            h_f = y_max - y_min
            agrupados.append((x_min, y_min, w_f, h_f, cls1, y_min + h_f))
            usados.add(i)
        return agrupados

    def extraer_candidatos_por_color(self, mask, id_clase, y_horizonte, h_img):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidatos = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 15:  
                x_b, y_b, w_b, h_b = cv2.boundingRect(cnt)
                y_base = y_b + h_b
                if y_base < y_horizonte: continue 
                candidatos.append((x_b, y_b, w_b, h_b, id_clase, y_base))
        return candidatos

    def procesar_frame(self, frame_bgr, modo_laboratorio=True):
        """
        Recibe un frame puro, lo procesa y retorna las detecciones y la acción de navegación.
        """
        img_procesamiento = cv2.resize(frame_bgr, (640, 360), interpolation=cv2.INTER_LINEAR)
        h_img, w_img = img_procesamiento.shape[:2]
        y_horizonte = int(h_img * 0.45)
        
        blurred = cv2.GaussianBlur(img_procesamiento, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        
        # Máscaras base
        mask_agua = cv2.inRange(hsv, (90, 80, 30), (135, 255, 255))
        mask_sin_agua = cv2.bitwise_not(mask_agua)
        
        h, s, v = cv2.split(hsv)
        b, g, r = cv2.split(blurred)
        
        _, mask_v = cv2.threshold(v, 40, 255, cv2.THRESH_BINARY)
        umbral_croma = 60 if modo_laboratorio else 20
        
        # Procesamiento Rojo y Verde
        dom_rojo = cv2.subtract(r, cv2.max(g, b))
        _, mask_r_chroma = cv2.threshold(dom_rojo, umbral_croma, 255, cv2.THRESH_BINARY)
        mask_red = cv2.bitwise_and(mask_r_chroma, mask_v)
        
        dom_verde = cv2.subtract(g, cv2.max(r, b))
        _, mask_g_chroma = cv2.threshold(dom_verde, umbral_croma, 255, cv2.THRESH_BINARY)
        mask_green = cv2.bitwise_and(mask_g_chroma, mask_v)

        if not modo_laboratorio:
            mask_red = cv2.bitwise_and(mask_red, mask_sin_agua)
            mask_green = cv2.bitwise_and(mask_green, mask_sin_agua)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        
        cand_r = self.extraer_candidatos_por_color(mask_red, 1, y_horizonte, h_img)
        cand_g = self.extraer_candidatos_por_color(mask_green, 2, y_horizonte, h_img)
        candidatos_fusionados = self.agrupar_bloques_cercanos(cand_r + cand_g, max_dist=55)
        
        edges_raw = cv2.Canny(gray, 20, 75)
        objetos_actuales = []

        # Análisis de bordes y límites
        for (x_b, y_b, w_b, h_b, id_clase, y_base) in candidatos_fusionados:
            aspect_ratio = float(h_b) / w_b
            if aspect_ratio > 2.2:
                h_b = int(w_b * 1.6)
                y_base = y_b + h_b

            roi_edges = edges_raw[y_b:y_b+h_b, x_b:x_b+w_b]
            min_edges_required = 6 if id_clase == 2 else 12
            if np.sum(roi_edges > 0) < min_edges_required: continue
            if w_b < 5 or h_b < 5: continue
            
            limite_izq, limite_der = self.obtener_limites_pista(y_base, h_img, w_img)
            estado_objeto = 0
            if limite_izq is not None and limite_der is not None:
                if limite_izq <= (x_b + w_b/2) <= limite_der:
                    factor_proximidad = (y_base - y_horizonte) / (h_img - y_horizonte)
                    if factor_proximidad >= 0.45: estado_objeto = 2
                    else: estado_objeto = 1  
            
            objetos_actuales.append([x_b, y_b, w_b, h_b, id_clase, estado_objeto, 8])

        # Tracking simple (Historial)
        for act in objetos_actuales:
            encontrado = False
            for prev in self.historial_detecciones:
                dist_centros = np.hypot((act[0]+act[2]/2) - (prev[0]+prev[2]/2), (act[1]+act[3]/2) - (prev[1]+prev[3]/2))
                max_match_dist = 55 if prev[2] > 45 else 35
                if dist_centros < max_match_dist:
                    prev[0], prev[1], prev[2], prev[3] = act[0], act[1], act[2], act[3]
                    prev[5] = act[5]; prev[4] = act[4]; prev[6] = act[6] 
                    encontrado = True
                    break
            if not encontrado: self.historial_detecciones.append(act)

        objetos_finales_hud = []
        critico, lejano = False, False
        lado_obstaculo = 0.0

        for obj in self.historial_detecciones[:]:
            obj[6] -= 1 
            if obj[6] <= 0:
                self.historial_detecciones.remove(obj)
            else:
                objetos_finales_hud.append((obj[0], obj[1], obj[2], obj[3], obj[4], obj[5]))
                if obj[5] == 2:
                    critico = True
                    lim_izq, lim_der = self.obtener_limites_pista(obj[1]+obj[3], h_img, w_img)
                    if lim_izq is not None and lim_der is not None:
                        lado_obstaculo = (obj[0] + obj[2]/2) - (lim_izq + (lim_der - lim_izq) / 2)
                elif obj[5] == 1:
                    lejano = True

        # Toma de decisiones
        if critico:
            accion = "EVASION: PIVOTE DERECHO" if lado_obstaculo < 0 else "EVASION: PIVOTE IZQUIERDO"
            alarma_num = 1.0 if lado_obstaculo < 0 else 2.0
        elif lejano:
            accion = "ALERTA: FRENADO/RETROCESO TACTICO"
            alarma_num = 3.0 
        else:
            accion = "CANAL LIBRE: AVANCE GPS"
            alarma_num = 0.0 

        # Se retorna toda la información procesada para que ROS o el entorno la use
        return objetos_finales_hud, accion, alarma_num, img_procesamiento