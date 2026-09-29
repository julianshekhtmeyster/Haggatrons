"""Provision the W11's Wi-Fi once over USB; never prints or commits credentials."""

from __future__ import annotations

import argparse
import getpass
import json
import os
import secrets
import time
from pathlib import Path

import serial


ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / "runs" / "robot_config.json"


def field(value: str, maximum: int) -> bytes:
    encoded = value.encode("utf-8")
    if not 1 <= len(encoded) <= maximum:
        raise ValueError(f"Field must contain 1–{maximum} UTF-8 bytes")
    return bytes([len(encoded)]) + encoded


def save_config(host: str, token: str) -> None:
    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    fd = os.open(CONFIG, flags, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump({"host": host, "token": token}, stream)
        stream.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 USB serial port, e.g. /dev/cu.usbmodem101")
    parser.add_argument("--ssid", required=True, help="2.4 GHz Wi-Fi network name")
    args = parser.parse_args()
    password = getpass.getpass("Wi-Fi password: ")
    token = secrets.token_hex(16)
    packet = b"P" + field(args.ssid, 32) + field(password, 64) + field(token, 32)

    with serial.Serial(args.port, 921600, timeout=1, write_timeout=3) as port:
        time.sleep(2)
        port.reset_input_buffer()
        port.write(packet)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            line = port.readline().decode("ascii", "replace").strip()
            if line == "WIFI SAVED":
                break
            if line.startswith("ERR "):
                raise RuntimeError(line)
        else:
            raise TimeoutError("Board did not confirm Wi-Fi provisioning")
        print("Wi-Fi settings saved on the board; waiting for its IP address...")
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            port.write(b"S")
            line = port.readline().decode("ascii", "replace").strip()
            if line.startswith("WIFI ") and line != "WIFI DISCONNECTED":
                host = line[5:]
                save_config(host, token)
                print(f"Connected: {host}. Local access token saved to {CONFIG}.")
                return
            time.sleep(1)
    raise TimeoutError("Board saved Wi-Fi settings but did not join within 30 seconds; check 2.4 GHz network and power")


if __name__ == "__main__":
    main()
