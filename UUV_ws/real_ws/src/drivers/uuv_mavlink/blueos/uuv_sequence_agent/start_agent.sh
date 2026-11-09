#!/usr/bin/env bash
set -Eeuo pipefail

NAME="uuv_sequence_agent"
COMMAND="python3 /usr/blueos/userdata/uuv_sequence_agent/main.py"

if tmux has-session -t "$NAME" 2>/dev/null; then
    echo "DETENIDO: ya existe la sesión tmux $NAME"
    exit 1
fi

tmux new-session -d -s "$NAME"
tmux send-keys -t "$NAME:0" \
    "run-service '$NAME' '$COMMAND' 180 50 0 0" C-m

for _INTENTO in $(seq 1 30); do
    if python3 - <<'PY'
import json
from urllib.request import urlopen

with urlopen('http://127.0.0.1:9132/health', timeout=0.5) as response:
    body = json.load(response)
if body.get('revision') != 'absolute_embedded_link_v7':
    raise SystemExit(1)
PY
    then
        echo "AGENTE_V7_ACTIVO=SI"
        exit 0
    fi
    sleep 0.2
done

echo "DETENIDO: el agente no respondió. Últimas líneas:"
tmux capture-pane -pt "$NAME:0" -S -80 || true
exit 1
