# Historial de cambios

Este archivo conserva hitos verificables. Los detalles de una sesión de prueba
pertenecen en `docs/testing/` o junto al subsistema correspondiente.

## 2026-10-09

- Unificación de cámara y luces desde mando e interfaz mediante
  `accessory_control` y un único bridge MAVLink.
- Biblioteca de misiones con guardar, cargar, renombrar, eliminar y borrador
  automático.
- Edición con insertar, duplicar, patrón rectangular y tolerancias globales.
- Gestión restringida de parámetros con snapshot, verificación y rollback.
- Validación automatizada: 146 pruebas superadas y 3 omitidas.
- Exclusión de artefactos de colcon en `.gitignore`.
- Reorganización de la documentación raíz en guías especializadas.

## 2026-10-07

- Análisis y corrección de transiciones GUIDED, toma manual y cancelación.
- Consulte `UUV_ws/real_ws/ANALISIS_GUIDED_20261007.md`.

## 2026-09

- Migración y validación inicial del UUV real con ROS 2 Jazzy.
- Telemetría ArduSub, cámara, mando Xbox, accesorios y dashboards de seguridad.
- Observación inicial del camino DVL/EKF; precisión en agua quedó pendiente.
- Telemetría de solo lectura del BlueBoat y servicios SeaTrac preservados.
