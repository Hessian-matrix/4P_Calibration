#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: %s 0|1|2|3\n' "${0##*/}"
    printf 'Local settings: local/calibration.conf and local/target.yaml beside this script.\n'
}

if [[ $# -eq 1 && ( $1 == --help || $1 == -h ) ]]; then
    usage
    exit 0
fi
if [[ $# -ne 1 || ! $1 =~ ^[0-3]$ ]]; then
    usage >&2
    exit 2
fi
camera=$1
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

CALIBRATION_HOST=
# This is trusted, user-owned shell configuration, not downloaded camera data.
if [[ -f "$script_dir/local/calibration.conf" ]]; then
    source "$script_dir/local/calibration.conf"
fi
if [[ ! $CALIBRATION_HOST =~ ^[a-zA-Z0-9][a-zA-Z0-9.-]*$ ]]; then
    printf 'Set CALIBRATION_HOST to the device IPv4 address or hostname in %s/local/calibration.conf\n' "$script_dir" >&2
    exit 2
fi
python="$script_dir/.venv/bin/python"
if [[ ! -x $python ]]; then
    printf 'Missing %s; create .venv and install this package as described in README.md.\n' "$python" >&2
    exit 2
fi
target="$script_dir/local/target.yaml"
if [[ ! -f $target ]]; then
    printf 'Missing measured target: %s; fill in the board template before capture.\n' "$target" >&2
    exit 2
fi

url="rtsp://${CALIBRATION_HOST}:$((554 + camera))/PRR"
output_dir="$script_dir/calibration_runs/robobaton_4p/cam${camera}/$(date -u +%Y%m%dT%H%M%S.%NZ)"
preview=()
if [[ -n ${DISPLAY:-} || -n ${WAYLAND_DISPLAY:-} ]]; then
    preview=(--preview-window --preview-scale 0.5 --preview-detect-every-n 5)
else
    printf 'No graphical display; use s/h/f/r/q + Enter in this terminal.\n'
fi
printf 'Camera: cam%s  RTSP: %s\nTarget: %s\nOutput: %s\n' "$camera" "$url" "$target" "$output_dir"
# Isolated mode ignores ROS/PYTHONPATH packages; the environment owns dependencies.
exec "$python" -I -u -m robobaton_calibration \
    --rtsp-url "$url" \
    --camera-id "cam${camera}" \
    --target "$target" \
    --output-dir "$output_dir" \
    "${preview[@]}"
