#!/usr/bin/env bash
set -Eeuo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TARGET="/usr/blueos/userdata/uuv_sequence_agent"
STARTUP="/usr/bin/start-blueos-core"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [ "$(id -u)" -ne 0 ]; then
    echo "DETENIDO: ejecuta como root dentro de BlueOS."
    exit 1
fi

python3 "$HERE/check_disarmed.py"
"$HERE/stop_agent.sh"
python3 "$HERE/install_hook.py" uninstall "$STARTUP"
bash -n "$STARTUP"

if [ "$HERE" != "$TARGET" ] && [ -d "$TARGET" ]; then
    mv "$TARGET" "${TARGET}.disabled_${STAMP}"
fi

echo "CONTROL_EMBARCADO_V7_INSTALADO=NO"
echo "Los archivos se conservaron como respaldo; no se borraron."
