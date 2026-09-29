"""Capture one W11 JPEG and IMU sample over Wi-Fi without an OpenAI call."""

from __future__ import annotations

import argparse
import json
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

from observation_protocol import decode_observation


ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / "runs" / "robot_config.json"


def request(host: str, token: str, endpoint: str) -> bytes:
    url = f"http://{host}/{endpoint}"
    req = urllib.request.Request(url, headers={"X-Robot-Token": token})
    with urllib.request.urlopen(req, timeout=15) as response:
        return response.read(2_000_100)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="Override the provisioned IP address")
    args = parser.parse_args()
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    host = args.host or config["host"]
    token = config["token"]
    status = json.loads(request(host, token, "status"))
    observation = decode_observation(request(host, token, "observation"))
    folder = ROOT / "runs" / datetime.now(timezone.utc).strftime("wireless_%Y%m%dT%H%M%SZ")
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "frame.jpg").write_bytes(observation.jpeg)
    (folder / "observation.json").write_text(
        json.dumps({"status": status, **observation.metadata()}, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Saved {len(observation.jpeg)} JPEG bytes and IMU metadata to {folder}")


if __name__ == "__main__":
    main()
