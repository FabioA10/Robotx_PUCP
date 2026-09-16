#!/usr/bin/env bash
# Run from a terminal with Jazzy and the UUV overlay already sourced.
set -euo pipefail
if ! command -v ros2 >/dev/null; then
    echo 'Primero carga /opt/ros/jazzy/setup.bash y el overlay del UUV.' >&2
    exit 1
fi
if [[ "${ROS_DISTRO:-}" != jazzy ]]; then
    echo 'Esta prueba corresponde al entorno ROS 2 Jazzy.' >&2
    exit 1
fi
pool_log_root="${1:-$HOME/uuv_test_logs}"
mkdir -p "$pool_log_root"
pool_output="$pool_log_root/pool_$(date -u +%Y%m%dT%H%M%S)_$$"
echo "Grabando en $pool_output. Finaliza con Ctrl+C."
exec ros2 bag record -o "$pool_output" \
    /uuv/connected /uuv/armed /uuv/mode /uuv/power/voltage \
    /uuv/control/command_output_enabled \
    /uuv/telemetry/local_position_ned \
    /uuv/telemetry/local_velocity_ned \
    /uuv/telemetry/attitude_rpy /uuv/telemetry/heading_deg \
    /uuv/telemetry/depth_estimate_m /uuv/telemetry/relative_altitude_m \
    /uuv/telemetry/pressure_abs_hpa /uuv/telemetry/ekf_status_flags \
    /uuv/sequence/status
