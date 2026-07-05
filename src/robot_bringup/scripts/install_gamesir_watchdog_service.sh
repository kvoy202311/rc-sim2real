#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="kvoy-gamesir-watchdog.service"
WORKSPACE="${KVOY_WORKSPACE:-/home/robocon-2026/Desktop/RC2026/kvoy_quadruped}"
SERVICE_SRC="${WORKSPACE}/install/robot_bringup/share/robot_bringup/systemd/${SERVICE_NAME}"
SERVICE_DST="/etc/systemd/system/${SERVICE_NAME}"

if [[ ! -f "${SERVICE_SRC}" ]]; then
  echo "[ERROR] Service file not found: ${SERVICE_SRC}" >&2
  echo "Build first: colcon build --packages-select robot_bringup" >&2
  exit 1
fi

sudo cp "${SERVICE_SRC}" "${SERVICE_DST}"
sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}"

echo "[OK] Installed and started ${SERVICE_NAME}"
echo "Status: sudo systemctl status ${SERVICE_NAME} --no-pager"
echo "Logs:   sudo journalctl -u ${SERVICE_NAME} -f"
