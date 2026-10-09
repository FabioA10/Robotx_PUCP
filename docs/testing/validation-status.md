# Estado de validación

## Última integración documentada

| Campo | Resultado |
|---|---|
| Fecha | 9 de octubre de 2026 |
| Rama | `fix-guided-transition` |
| Integración funcional | `e0288f2` |
| HEAD observado antes de esta reorganización | `5353a390` |
| Compilación | 6 paquetes finalizados |
| `uuv_teleop` | 2 passed, 1 skipped |
| `uuv_mavlink` | 92 passed, 1 skipped |
| `uuv_dashboard` | 52 passed, 1 skipped |
| Total | **146 passed, 3 skipped** |

La validación se ejecutó con instalaciones aisladas de colcon y descartando
overlays anteriores antes de compilar.

## Cubierto por software

- Ruta única para accesorios desde mando e interfaz hacia el bridge.
- Cámara y luces con exclusión respecto de movimiento y cambio de modo.
- Gates de movimiento, ARM/DISARM, accesorios y parámetros.
- Modelo de accesorios y bridge MAVLink.
- Creación, persistencia y edición avanzada de misiones.
- Ejecutor y modelo de secuencias.
- Gestión restringida de cinco parámetros con rollback.
- Lanzamientos e interfaz de operación.

## No demostrado por esas pruebas

- Precisión real de profundidad, distancia y yaw.
- Deriva y asentamiento en la piscina.
- Robustez del DVL/EKF bajo movimiento.
- Detención física ante pérdida de enlace o estimador.
- Ejecución completa de rectángulo y retorno a Home.
- Posicionamiento acústico X150-X110.
- Control físico del USV desde ROS 2.

## Regla de estado

Use estas etiquetas en documentación e informes:

| Estado | Significado |
|---|---|
| Implementado | Existe código y revisión de interfaz |
| Probado automáticamente | Pruebas reproducibles sin hardware aprobadas |
| Validado en banco | Hardware conectado sin demostrar comportamiento en agua |
| Validado en agua | Métrica, tolerancia y registro físico aprobados |

No promueva una función al siguiente estado sin adjuntar evidencia reproducible.
