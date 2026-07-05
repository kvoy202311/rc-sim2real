#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${1:-kvoy-bringup.service}"
WORKSPACE="${KVOY_WORKSPACE:-/home/robocon-2026/Desktop/RC2026/kvoy_quadruped}"
SERVICE_SRC="${WORKSPACE}/install/robot_bringup/share/robot_bringup/systemd/${SERVICE_NAME}"
SERVICE_DST="${HOME}/.config/systemd/user/${SERVICE_NAME}"

if [[ ! -f "${SERVICE_SRC}" ]]; then
  echo "[ERROR] Service file not found: ${SERVICE_SRC}" >&2
  echo "Build first: colcon build --packages-select robot_bringup" >&2
  exit 1
fi

mkdir -p "${HOME}/.config/systemd/user"
cp "${SERVICE_SRC}" "${SERVICE_DST}"

systemctl --user daemon-reload
systemctl --user enable --now "${SERVICE_NAME}"

if command -v loginctl >/dev/null 2>&1; then
  sudo loginctl enable-linger "$(id -un)"
fi

echo "[OK] Installed and started ${SERVICE_NAME}"
echo "Logs:    journalctl --user -u ${SERVICE_NAME} -f"
echo "Restart: systemctl --user restart ${SERVICE_NAME}"
