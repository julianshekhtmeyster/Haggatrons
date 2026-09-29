"""Capture camera+IMU observations, ask a VLM for proposals, and log each step.

This program has no motor commands. Close the live preview before running it,
because the ESP32 serial port can only be opened by one program at a time.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import time
from datetime import datetime, timezone
from pathlib import Path

from PIL import Image
import serial

from flower_explore.openai_observer import (
    MODEL as OPENAI_MODEL, INSTRUCTIONS as OPENAI_INSTRUCTIONS,
    load_api_key as load_openai_key, observe_jpeg,
)
from observation_protocol import RESOLUTIONS, read_observation


ROOT = Path(__file__).resolve().parent


def save_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--resolution", choices=RESOLUTIONS, default="640x480")
    parser.add_argument("--goal", default="Understand this scene and suggest a safe next look")
    parser.add_argument("--count", type=int, default=1, help="Number of observations, 1-10")
    parser.add_argument("--interval", type=float, default=0, help="Seconds between observations")
    parser.add_argument("--model", help="Override the default OpenAI model")
    parser.add_argument("--capture-only", action="store_true", help="Save frames and IMU without API calls")
    args = parser.parse_args()
    if not 1 <= args.count <= 10 or args.interval < 0:
        parser.error("count must be 1-10 and interval must be nonnegative")
    model = args.model or OPENAI_MODEL
    api_key = "" if args.capture_only else load_openai_key()
    if not args.capture_only and not api_key:
        parser.error(f"Put OPENAI_API_KEY in {ROOT / '.env'} first (do not paste it in chat)")

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    run_dir = ROOT / "runs" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    prompt_bytes = OPENAI_INSTRUCTIONS.encode("utf-8")
    save_json(run_dir / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "port": args.port,
        "resolution": args.resolution,
        "goal": args.goal,
        "count": args.count,
        "model": None if args.capture_only else model,
        "mode": "capture_only" if args.capture_only else "openai",
        "prompt_sha256": hashlib.sha256(prompt_bytes).hexdigest(),
        "motor_commands_enabled": False,
    })

    port = serial.Serial(port=None, baudrate=921600, timeout=0.5, write_timeout=2)
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
    except serial.SerialException as exc:
        raise SystemExit(f"Cannot open {args.port}; close the W11 USB Camera window. {exc}") from exc
    try:
        time.sleep(1)
        port.reset_input_buffer()
        port.write(b"R" + RESOLUTIONS[args.resolution])
        time.sleep(0.2)
        for index in range(1, args.count + 1):
            capture_start = time.monotonic()
            observation = read_observation(port)
            capture_seconds = time.monotonic() - capture_start
            captured_utc = datetime.now(timezone.utc).isoformat()
            with Image.open(io.BytesIO(observation.jpeg)) as image:
                image.verify()
                width, height = image.size
            metadata = observation.metadata()
            metadata.update({
                "captured_utc": captured_utc,
                "width": width,
                "height": height,
                "capture_transfer_seconds": round(capture_seconds, 3),
            })
            stem = f"{index:04d}"
            (run_dir / f"frame_{stem}.jpg").write_bytes(observation.jpeg)
            save_json(run_dir / f"observation_{stem}.json", metadata)
            print(f"Observation {index}: {width}x{height}, IMU valid={observation.imu_valid}")

            if not args.capture_only:
                model_start = time.monotonic()
                try:
                    decision, usage = observe_jpeg(
                        observation.jpeg, args.goal, api_key, model
                    )
                except Exception as exc:
                    safe_error = str(exc).replace(api_key, "[REDACTED]")
                    save_json(run_dir / f"error_{stem}.json", {
                        "error": safe_error
                    })
                    raise RuntimeError(safe_error) from None
                save_json(run_dir / f"decision_{stem}.json", {
                    "decision": decision,
                    "model": model,
                    "usage": usage,
                    "model_seconds": round(time.monotonic() - model_start, 3),
                    "motor_action_executed": False,
                })
                print(f"  Proposed: {decision['proposed_action']} — {decision['action_reason']}")
            if index < args.count:
                time.sleep(args.interval)
    finally:
        port.close()
    print(f"Saved run: {run_dir}")


if __name__ == "__main__":
    main()
