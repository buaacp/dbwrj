#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/zlhq/px4_fly_ws}"
PX4_DIR="${PX4_DIR:-/home/zlhq/PX4_Firmware}"
WORLD_FILE="${WORLD_FILE:-${WS_DIR}/src/arm_control/worlds/weightless_ball.world}"
TAKEOFF_Z="${TAKEOFF_Z:-0.8}"
SIM_DELAY="${SIM_DELAY:-3}"
TAKEOFF_TIMEOUT="${TAKEOFF_TIMEOUT:-90}"
SIM_GUI="${SIM_GUI:-true}"

SIM_LAUNCH_PID=""
UAV_LAUNCH_PID=""
ARM_LAUNCH_PID=""

cleanup() {
  echo "[start_bulb_task] stopping launched processes..."
  for pid in "${ARM_LAUNCH_PID}" "${UAV_LAUNCH_PID}" "${SIM_LAUNCH_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

source_ros_env() {
  if [[ -f /opt/ros/melodic/setup.bash ]]; then
    source /opt/ros/melodic/setup.bash
  elif [[ -f /opt/ros/noetic/setup.bash ]]; then
    source /opt/ros/noetic/setup.bash
  fi

  if [[ -f "${WS_DIR}/devel/setup.bash" ]]; then
    source "${WS_DIR}/devel/setup.bash"
  fi

  if [[ -f "${PX4_DIR}/Tools/setup_gazebo.bash" ]]; then
    source "${PX4_DIR}/Tools/setup_gazebo.bash" "${PX4_DIR}" "${PX4_DIR}/build/px4_sitl_default"
  fi

  export ROS_PACKAGE_PATH="${PX4_DIR}:${PX4_DIR}/Tools/sitl_gazebo:${ROS_PACKAGE_PATH:-}"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"
  until rostopic list 2>/dev/null | grep -qx "${topic}"; do
    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "[start_bulb_task] timeout waiting for topic ${topic}" >&2
      return 1
    fi
    sleep 1
  done
}

current_z() {
  timeout 2s rostopic echo -n 1 /iris_0/mavros/local_position/pose/pose/position/z 2>/dev/null \
    | awk 'NF {print $1; exit}'
}

wait_for_takeoff() {
  local start_s z
  start_s="$(date +%s)"
  echo "[start_bulb_task] waiting for takeoff: z >= ${TAKEOFF_Z} m"
  while true; do
    z="$(current_z || true)"
    if [[ -n "${z}" ]] && awk -v z="${z}" -v target="${TAKEOFF_Z}" 'BEGIN {exit !(z >= target)}'; then
      echo "[start_bulb_task] takeoff detected: z=${z} m"
      return 0
    fi
    if (( "$(date +%s)" - start_s >= TAKEOFF_TIMEOUT )); then
      echo "[start_bulb_task] timeout waiting for takeoff, last z=${z:-unknown}" >&2
      return 1
    fi
    sleep 1
  done
}

main() {
  source_ros_env

  echo "[start_bulb_task] launching simulation..."
  roslaunch le_arm iris_arm.launch world:="${WORLD_FILE}" gui:="${SIM_GUI}" &
  SIM_LAUNCH_PID="$!"

  sleep "${SIM_DELAY}"

  wait_for_topic "/iris_0/mavros/local_position/pose" 60

  echo "[start_bulb_task] launching UAV bulb task..."
  roslaunch test_fly test_fly_bulb.launch start_pose_bridge:=false &
  UAV_LAUNCH_PID="$!"

  wait_for_takeoff

  echo "[start_bulb_task] launching arm bulb controller..."
  roslaunch arm_control control_arm_bulb.launch &
  ARM_LAUNCH_PID="$!"

  echo "[start_bulb_task] all nodes launched. Press Ctrl-C to stop."
  wait
}

main "$@"
