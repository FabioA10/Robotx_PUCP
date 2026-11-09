#!/usr/bin/env bash
set -Eeuo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TARGET="/usr/blueos/userdata/uuv_sequence_agent"
STARTUP="/usr/bin/start-blueos-core"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [ "$(id -u)" -ne 0 ]; then
    echo "DETENIDO: ejecuta este instalador como root dentro de BlueOS."
    exit 1
fi

echo "===== VALIDACIÓN SEGURA ====="
python3 "$HERE/check_disarmed.py"
[ "$(uname -m)" = "armv7l" ] || {
    echo "DETENIDO: se esperaba armv7l y se obtuvo $(uname -m)"
    exit 1
}
python3 - <<'PY'
import aiohttp
print('PYTHON_AIOHTTP=OK', aiohttp.__version__)
PY
curl -fsS --max-time 2 http://127.0.0.1:6040/v1/mavlink >/dev/null
[ -x /usr/bin/run-service ] || {
    echo "DETENIDO: falta /usr/bin/run-service"
    exit 1
}
[ -f "$STARTUP" ] || {
    echo "DETENIDO: falta $STARTUP"
    exit 1
}

if tmux has-session -t uuv_sequence_agent 2>/dev/null; then
    "$HERE/stop_agent.sh"
fi

echo "===== INSTALANDO ARCHIVOS PERSISTENTES ====="
if [ -e "$TARGET" ] && [ "$HERE" != "$TARGET" ]; then
    mv "$TARGET" "${TARGET}.before_v7_${STAMP}"
fi
if [ "$HERE" != "$TARGET" ]; then
    mkdir -p "$TARGET"
    cp -a "$HERE"/. "$TARGET"/
fi
chmod 0755 "$TARGET"/*.sh "$TARGET"/*.py

echo "===== REGISTRANDO SERVICIO BLUEOS ====="
python3 "$TARGET/install_hook.py" install "$STARTUP"
bash -n "$STARTUP"

echo "===== INICIANDO AGENTE ====="
"$TARGET/start_agent.sh"
"$TARGET/status_agent.sh"

echo
echo "CONTROL_EMBARCADO_V7_INSTALADO=SI"
echo "API=http://192.168.2.2:9132"
