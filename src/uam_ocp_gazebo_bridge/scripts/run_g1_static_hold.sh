#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${G1_WORKSPACE:-/home/zlhq/px4_fly_ws}"
CONFIG="${WORKSPACE}/src/uam_ocp_gazebo_bridge/config/g1_static_hold.yaml"
SCENARIO="neutral"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --scenario) SCENARIO="$2"; shift 2 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done

source "${WORKSPACE}/devel/setup.bash"
exec rosrun uam_ocp_gazebo_bridge g1_static_hold_runner.py \
  --config "${CONFIG}" --scenario "${SCENARIO}"
