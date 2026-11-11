# Guía de anotación — RobotX Task 1 (USV)

## Clases permitidas

- `task_robobuoy`
- `beacon_red_on`
- `beacon_green_on`
- `beacon_blue_on`

No se deben crear otras clases para este proyecto.

## Reglas de etiquetado

1. **`task_robobuoy`**: dibuja una caja grande que abarque toda la RoboBuoy
   oficial visible, incluido el cuerpo, el poste y la baliza lateral, esté
   iluminada o no.
2. **`beacon_red_on`**, **`beacon_green_on`** y **`beacon_blue_on`**: dibuja
   una caja pequeña y ajustada únicamente al LED lateral visible cuando esté
   encendido.
3. Se permiten cajas superpuestas: la caja del LED queda dentro de la caja
   `task_robobuoy`.
4. No crear ni inferir las etiquetas `entry`, `exit`, `flashing`, `solid`,
   `off` o `dark`, porque las imágenes son fotos aisladas.
5. No etiquetar agua, reflejos, edificios, luces de ciudad, bote propio ni
   otras embarcaciones.
6. Las fotos sin RoboBuoys deben permanecer sin cajas como ejemplos negativos.
