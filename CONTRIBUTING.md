# Contribuir a RobotX PUCP

## Principios

- Trabaje en el workspace de la plataforma; no compile la raíz completa.
- Mantenga separadas las instalaciones de ROS 2 Jazzy y Humble.
- No confirme `build/`, `install/`, `log/`, grabaciones ni respaldos locales.
- No habilite salidas físicas para resolver un problema de compilación o telemetría.
- Documente por separado lo implementado, lo probado en banco y lo validado en agua.

## Flujo Git

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP"
git status --short --branch
git fetch origin
git pull --ff-only
```

Revise antes de confirmar:

```bash
git diff --check
git diff --stat
git status --short
```

Prepare archivos intencionalmente. Evite `git add .` cuando existan datos o
artefactos locales:

```bash
git add README.md CONTRIBUTING.md docs/
git diff --cached --check
git diff --cached --stat
git commit -m "docs: reorganizar guias operativas de RobotX"
git push
```

## Validación UUV

```bash
cd "$HOME/Escritorio/ROS_2/Robotx_PUCP/UUV_ws/real_ws"
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-up-to uuv_dashboard uuv_mavlink uuv_teleop

source install/local_setup.bash
colcon test --packages-select uuv_teleop uuv_mavlink uuv_dashboard
colcon test-result --verbose
```

Para cambios de seguridad, agregue pruebas puras o simuladas antes de probar
con hardware. Registre explícitamente las salidas habilitadas, el firmware, la
fecha, el entorno y los casos omitidos.

## Documentación

- `README.md` debe seguir siendo una portada y ruta de inicio rápido.
- Los procedimientos detallados pertenecen en `docs/`.
- Los informes fechados deben ir en `docs/testing/` o junto al subsistema.
- Los comandos deben copiarse de los launch files vigentes y probarse después
  de cada cambio de argumentos.
- Use un solo `#` por documento y mantenga una jerarquía consistente.
