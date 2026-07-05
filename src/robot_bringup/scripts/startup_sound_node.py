#!/usr/bin/env python3
"""Play a short non-blocking startup sound after bringup starts."""

from __future__ import annotations

import math
import struct
import subprocess
import tempfile
import wave
from pathlib import Path

import rclpy
from rclpy.node import Node


class StartupSoundNode(Node):
    def __init__(self) -> None:
        super().__init__("startup_sound_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("delay_s", 2.0)
        self.declare_parameter("frequency_hz", 880.0)
        self.declare_parameter("duration_s", 0.18)
        self.declare_parameter("repeat", 2)
        self.declare_parameter("gap_s", 0.08)
        self.declare_parameter("volume", 0.35)
        self.declare_parameter("aplay_device", "")

        self.enabled = bool(self.get_parameter("enabled").value)
        self.delay_s = max(0.0, float(self.get_parameter("delay_s").value))
        self.frequency_hz = max(20.0, float(self.get_parameter("frequency_hz").value))
        self.duration_s = max(0.02, float(self.get_parameter("duration_s").value))
        self.repeat = max(1, int(self.get_parameter("repeat").value))
        self.gap_s = max(0.0, float(self.get_parameter("gap_s").value))
        self.volume = min(1.0, max(0.0, float(self.get_parameter("volume").value)))
        self.aplay_device = str(self.get_parameter("aplay_device").value).strip()

        self.played = False
        self.timer = self.create_timer(max(0.01, self.delay_s), self.on_timer)

    def on_timer(self) -> None:
        if self.played:
            return
        self.played = True
        self.timer.cancel()

        if not self.enabled:
            self.get_logger().info("Startup sound disabled")
            return

        try:
            wav_path = self.create_beep_wav()
            cmd = ["aplay", "-q"]
            if self.aplay_device:
                cmd.extend(["-D", self.aplay_device])
            cmd.append(str(wav_path))
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info("Startup sound requested")
        except Exception as exc:
            self.get_logger().warn(f"Startup sound failed: {exc}")

    def create_beep_wav(self) -> Path:
        sample_rate = 44100
        tone_samples = max(1, int(sample_rate * self.duration_s))
        gap_samples = int(sample_rate * self.gap_s)
        amplitude = int(32767 * self.volume)

        path = Path(tempfile.gettempdir()) / "kvoy_startup_sound.wav"
        with wave.open(str(path), "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(sample_rate)
            for repeat_idx in range(self.repeat):
                for i in range(tone_samples):
                    # Short fade avoids clicks on small USB speakers.
                    edge = min(i, tone_samples - 1 - i)
                    fade = min(1.0, max(0.0, edge / max(1, int(sample_rate * 0.01))))
                    value = int(
                        amplitude *
                        fade *
                        math.sin(2.0 * math.pi * self.frequency_hz * i / sample_rate)
                    )
                    wav.writeframesraw(struct.pack("<h", value))
                if repeat_idx + 1 < self.repeat:
                    wav.writeframesraw(b"\x00\x00" * gap_samples)
        return path


def main() -> None:
    rclpy.init()
    node = StartupSoundNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
