#!/usr/bin/env python3
"""Recover GameSir receiver when it boots in non-joystick HID mode."""

from __future__ import annotations

import glob
import math
import os
import subprocess
import struct
import time
import wave
from pathlib import Path


VENDOR_ID = "3537"
GOOD_PRODUCT_ID = "1022"
BAD_PRODUCT_ID = "0575"


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def log(message: str) -> None:
    print(f"[gamesir_usb_watchdog] {message}", flush=True)


def find_gamesir_usb_device() -> tuple[str, str] | None:
    for device in sorted(Path("/sys/bus/usb/devices").iterdir()):
        vendor = read_text(device / "idVendor").lower()
        product = read_text(device / "idProduct").lower()
        if vendor == VENDOR_ID and product in {GOOD_PRODUCT_ID, BAD_PRODUCT_ID}:
            return device.name, product
    return None


def has_gamesir_joystick() -> bool:
    by_id_links = glob.glob("/dev/input/by-id/*GameSir*joystick")
    return bool(by_id_links) or Path("/dev/input/js0").exists()


def reset_usb_device(device_name: str) -> bool:
    device = Path("/sys/bus/usb/devices") / device_name
    authorized = device / "authorized"
    unbind = Path("/sys/bus/usb/drivers/usb/unbind")
    bind = Path("/sys/bus/usb/drivers/usb/bind")

    if authorized.exists():
        try:
            log(f"power-cycling USB authorization for {device_name}")
            authorized.write_text("0", encoding="utf-8")
            time.sleep(2.0)
            authorized.write_text("1", encoding="utf-8")
            return True
        except OSError as exc:
            log(f"USB authorization reset failed for {device_name}: {exc}")

    try:
        log(f"falling back to USB driver unbind/bind for {device_name}")
        unbind.write_text(device_name, encoding="utf-8")
        time.sleep(1.0)
        bind.write_text(device_name, encoding="utf-8")
        return True
    except OSError as exc:
        log(f"USB reset failed for {device_name}: {exc}")
        return False


def create_beep_wav(path: Path, frequency_hz: float, duration_s: float, repeat: int, gap_s: float) -> None:
    sample_rate = 44100
    tone_samples = max(1, int(sample_rate * duration_s))
    gap_samples = int(sample_rate * gap_s)
    amplitude = int(32767 * 0.35)

    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        for repeat_idx in range(repeat):
            for i in range(tone_samples):
                edge = min(i, tone_samples - 1 - i)
                fade = min(1.0, max(0.0, edge / max(1, int(sample_rate * 0.01))))
                value = int(
                    amplitude *
                    fade *
                    math.sin(2.0 * math.pi * frequency_hz * i / sample_rate)
                )
                wav.writeframesraw(struct.pack("<h", value))
            if repeat_idx + 1 < repeat:
                wav.writeframesraw(b"\x00\x00" * gap_samples)


def play_alert_sound(aplay_device: str) -> None:
    wav_path = Path("/tmp/kvoy_gamesir_watchdog_alert.wav")
    try:
        create_beep_wav(wav_path, frequency_hz=440.0, duration_s=0.16, repeat=4, gap_s=0.08)
        cmd = ["aplay", "-q"]
        if aplay_device:
            cmd.extend(["-D", aplay_device])
        cmd.append(str(wav_path))
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError as exc:
        log(f"Failed to play alert sound: {exc}")


def restart_user_bringup(user: str, uid: str, service_name: str) -> None:
    env = os.environ.copy()
    env["XDG_RUNTIME_DIR"] = f"/run/user/{uid}"
    cmd = [
        "runuser",
        "-u",
        user,
        "--",
        "systemctl",
        "--user",
        "restart",
        service_name,
    ]
    try:
        subprocess.run(cmd, env=env, check=False, timeout=10)
    except (OSError, subprocess.TimeoutExpired) as exc:
        log(f"Failed to restart {service_name}: {exc}")


def main() -> int:
    user = os.environ.get("KVOY_USER", "robocon-2026")
    uid = os.environ.get("KVOY_UID", "1001")
    service_name = os.environ.get("KVOY_BRINGUP_SERVICE", "kvoy-bringup.service")
    check_interval_s = float(os.environ.get("KVOY_GAMESIR_CHECK_INTERVAL_S", "2.0"))
    reset_cooldown_s = float(os.environ.get("KVOY_GAMESIR_RESET_COOLDOWN_S", "5.0"))
    post_reset_wait_s = float(os.environ.get("KVOY_GAMESIR_POST_RESET_WAIT_S", "4.0"))
    max_fast_resets = int(os.environ.get("KVOY_GAMESIR_MAX_FAST_RESETS", "4"))
    slow_retry_s = float(os.environ.get("KVOY_GAMESIR_SLOW_RETRY_S", "30.0"))
    alert_after_resets = int(os.environ.get("KVOY_GAMESIR_ALERT_AFTER_RESETS", "4"))
    alert_interval_s = float(os.environ.get("KVOY_GAMESIR_ALERT_INTERVAL_S", "20.0"))
    alert_aplay_device = os.environ.get("KVOY_GAMESIR_ALERT_APLAY_DEVICE", "plughw:2,0")

    log(
        "started; waiting for GameSir 3537:1022 joystick mode "
        f"and recovering 3537:0575 mode for user service {service_name}"
    )

    last_reset_time = 0.0
    last_alert_time = 0.0
    consecutive_resets = 0
    was_ready = False

    while True:
        device = find_gamesir_usb_device()
        ready = bool(device and device[1] == GOOD_PRODUCT_ID and has_gamesir_joystick())

        if ready:
            if not was_ready:
                log("GameSir joystick is ready; restarting bringup so SDL reopens it")
                restart_user_bringup(user, uid, service_name)
            was_ready = True
            consecutive_resets = 0
            last_alert_time = 0.0
            time.sleep(check_interval_s)
            continue

        was_ready = False
        if not device:
            log("GameSir USB receiver not found; waiting")
            time.sleep(check_interval_s)
            continue

        device_name, product = device
        now = time.monotonic()
        cooldown = slow_retry_s if consecutive_resets >= max_fast_resets else reset_cooldown_s
        if now - last_reset_time < cooldown:
            time.sleep(check_interval_s)
            continue

        if product == BAD_PRODUCT_ID:
            log(
                f"GameSir is in non-joystick mode {VENDOR_ID}:{BAD_PRODUCT_ID} "
                f"on USB device {device_name}; resetting USB device"
            )
        else:
            log(
                f"GameSir is {VENDOR_ID}:{product} but joystick node is missing; "
                f"resetting USB device {device_name}"
            )

        last_reset_time = now
        consecutive_resets += 1
        reset_usb_device(device_name)
        if consecutive_resets >= alert_after_resets and now - last_alert_time >= alert_interval_s:
            last_alert_time = now
            log(
                "GameSir still has no joystick after repeated recovery attempts; "
                "playing local alert and continuing to retry"
            )
            play_alert_sound(alert_aplay_device)
        time.sleep(post_reset_wait_s)


if __name__ == "__main__":
    raise SystemExit(main())
