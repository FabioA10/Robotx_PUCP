#!/usr/bin/env bash
set -u

echo "===== PROCESO ====="
tmux list-sessions 2>&1 | grep uuv_sequence_agent || true

echo
echo "===== SALUD ====="
python3 - <<'PY'
import json
from urllib.request import urlopen

for path in ('/health', '/v1/status'):
    print(path)
    try:
        with urlopen('http://127.0.0.1:9132' + path, timeout=1.0) as response:
            print(json.dumps(json.load(response), ensure_ascii=False, indent=2))
    except Exception as exc:
        print('ERROR:', exc)
PY

echo
echo "===== ÚLTIMAS LÍNEAS ====="
tmux capture-pane -pt uuv_sequence_agent:0 -S -100 2>&1 || true
