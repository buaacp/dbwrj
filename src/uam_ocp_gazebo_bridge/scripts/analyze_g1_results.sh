#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${G1_WORKSPACE:-/home/zlhq/px4_fly_ws}"
PYTHON="${G1_ANALYSIS_PYTHON:-/home/zlhq/micromamba-root/envs/eagle_mpc/bin/python3.10}"
export PYTHONPATH="${WORKSPACE}/src/uam_ocp_gazebo_bridge/src:${PYTHONPATH:-}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/g1_matplotlib}"
exec "${PYTHON}" "${WORKSPACE}/src/uam_ocp_gazebo_bridge/scripts/analyze_g1_results.py" "$@"
