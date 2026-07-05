#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${KVOY_WORKSPACE:-/home/robocon-2026/Desktop/RC2026/kvoy_quadruped}"
ROS_DISTRO="${ROS_DISTRO:-foxy}"
PARAMS_FILE="${KVOY_PARAMS_FILE:-}"
PARAMS_OVERRIDE_FILE="${KVOY_PARAMS_OVERRIDE_FILE:-}"
POLICIES_FILE="${KVOY_POLICIES_FILE:-}"

cd "${WORKSPACE}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"

if [[ -f "${WORKSPACE}/.venv_policy_export/bin/activate" ]]; then
  source "${WORKSPACE}/.venv_policy_export/bin/activate"
fi

args=()
if [[ -n "${PARAMS_FILE}" ]]; then
  args+=("params_file:=${PARAMS_FILE}")
fi
if [[ -n "${PARAMS_OVERRIDE_FILE}" ]]; then
  args+=("params_override_file:=${PARAMS_OVERRIDE_FILE}")
fi
if [[ -n "${POLICIES_FILE}" ]]; then
  args+=("policies_file:=${POLICIES_FILE}")
fi

exec ros2 launch robot_bringup bringup.launch.py "${args[@]}"
