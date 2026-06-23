#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/clean_sim_processes.sh [--purge-state]

Options:
  --purge-state   Also move PX4 SITL state out of the way and remove Gazebo tmp files.
                 The PX4 state directory is backed up, not deleted.
EOF
}

PURGE_STATE=0
for arg in "$@"; do
  case "$arg" in
    --purge-state)
      PURGE_STATE=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

PATTERNS=(
  "arm_hover_low_speed_test"
  "test_velocity_control"
  "mission_control"
  "test_fly_bulb"
  "arm_control_bulb"
  "whole_body_pose_control"
  "get_local_pose"
  "controller_spawner"
  "robot_state_publisher"
  "mavros"
  "px4"
  "gzclient"
  "gzserver"
  "roslaunch"
  "rosmaster"
  "roscore"
)

echo "[clean] Existing simulation-related processes:"
pgrep -a -f "$(IFS='|'; echo "${PATTERNS[*]}")" || true

echo "[clean] Sending SIGTERM..."
for pattern in "${PATTERNS[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done

sleep 3

if pgrep -f "$(IFS='|'; echo "${PATTERNS[*]}")" >/dev/null; then
  echo "[clean] Some processes are still alive; sending SIGKILL..."
  for pattern in "${PATTERNS[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done
  sleep 1
fi

if [[ "$PURGE_STATE" -eq 1 ]]; then
  timestamp="$(date +%Y%m%d_%H%M%S)"
  for px4_state in "${HOME}"/.ros/sitl_*; do
    [[ -d "$px4_state" ]] || continue
    [[ "$px4_state" == *.bak.* ]] && continue
    backup="${px4_state}.bak.${timestamp}"
    echo "[clean] Moving PX4 SITL state: $px4_state -> $backup"
    mv "$px4_state" "$backup"
  done

  echo "[clean] Removing Gazebo temporary files from /tmp..."
  find /tmp -maxdepth 1 -user "$(id -un)" \
    \( -name 'gazebo*' -o -name 'gzserver*' -o -name 'gzclient*' \) \
    -exec rm -rf {} +
fi

echo "[clean] Remaining simulation-related processes:"
remaining="$(pgrep -a -f "$(IFS='|'; echo "${PATTERNS[*]}")" || true)"
if [[ -n "$remaining" ]]; then
  echo "$remaining"
  echo "[clean] WARNING: Some processes are still present."
  exit 1
fi

echo "[clean] Done. ROS/Gazebo/PX4 simulation processes are clean."
