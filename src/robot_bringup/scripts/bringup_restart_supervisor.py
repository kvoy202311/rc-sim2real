#!/usr/bin/env python3
"""Safe gamepad-triggered restart helper for the systemd bringup service."""

from __future__ import annotations

import subprocess
import time
import os

import rclpy
from rclpy.node import Node

from kvoy_msgs.msg import GamepadCmd, RobotState


class BringupRestartSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("bringup_restart_supervisor")

        self.declare_parameter("restart_enabled", True)
        self.declare_parameter("service_name", "kvoy-bringup.service")
        self.declare_parameter("restart_hold_s", 3.0)
        self.declare_parameter("restart_cooldown_s", 10.0)
        self.declare_parameter("require_waiting_state", True)
        self.declare_parameter("require_systemd_env", True)

        self.restart_enabled = bool(self.get_parameter("restart_enabled").value)
        self.service_name = str(self.get_parameter("service_name").value)
        self.restart_hold_s = max(0.1, float(self.get_parameter("restart_hold_s").value))
        self.restart_cooldown_s = max(0.0, float(self.get_parameter("restart_cooldown_s").value))
        self.require_waiting_state = bool(self.get_parameter("require_waiting_state").value)
        self.require_systemd_env = bool(self.get_parameter("require_systemd_env").value)
        self.systemd_managed = os.environ.get("KVOY_SYSTEMD_MANAGED") == "1"

        self.fsm_state = None
        self.combo_pressed_since = None
        self.restart_fired_for_press = False
        self.last_restart_time = 0.0

        self.create_subscription(GamepadCmd, "/gamepad_cmd", self.on_gamepad, 10)
        self.create_subscription(RobotState, "/robot_state", self.on_robot_state, 10)

        if self.require_systemd_env and not self.systemd_managed:
            self.get_logger().warn(
                "bringup_restart_supervisor will ignore restart requests because "
                "KVOY_SYSTEMD_MANAGED=1 is not set"
            )
        else:
            self.get_logger().info(
                "bringup_restart_supervisor ready: hold LT+RT+A+B for "
                f"{self.restart_hold_s:.1f}s while FSM is WAITING"
            )

    def on_robot_state(self, msg: RobotState) -> None:
        self.fsm_state = int(msg.fsm_state)

    def on_gamepad(self, msg: GamepadCmd) -> None:
        if not self.restart_enabled:
            return
        if self.require_systemd_env and not self.systemd_managed:
            return

        now = time.monotonic()
        combo = bool(msg.btn_lt and msg.btn_rt and msg.btn_a and msg.btn_b)
        if not combo:
            self.combo_pressed_since = None
            self.restart_fired_for_press = False
            return

        if self.combo_pressed_since is None:
            self.combo_pressed_since = now
            return

        if self.restart_fired_for_press:
            return

        if now - self.combo_pressed_since < self.restart_hold_s:
            return

        if now - self.last_restart_time < self.restart_cooldown_s:
            self.get_logger().warn("Restart ignored: still in restart cooldown")
            self.restart_fired_for_press = True
            return

        if self.require_waiting_state and self.fsm_state != RobotState.FSM_WAITING:
            self.get_logger().warn(
                f"Restart ignored: FSM must be WAITING, current={self.fsm_state}"
            )
            self.restart_fired_for_press = True
            return

        self.restart_fired_for_press = True
        self.last_restart_time = now
        self.get_logger().warn(f"Restarting user service: {self.service_name}")
        subprocess.Popen(
            ["systemctl", "--user", "restart", self.service_name, "--no-block"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def main() -> None:
    rclpy.init()
    node = BringupRestartSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
