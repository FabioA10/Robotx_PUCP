#!/usr/bin/env bash
set -Eeuo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$HERE/check_disarmed.py"

if tmux has-session -t uuv_sequence_agent 2>/dev/null; then
    tmux send-keys -t uuv_sequence_agent:0 C-c
    sleep 1
    tmux kill-session -t uuv_sequence_agent 2>/dev/null || true
fi

echo "AGENTE_V7_ACTIVO=NO"
