"""Run the local two-worker Flower demo, optionally using one saved ESP32 frame."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from flower_explore.openai_observer import load_api_key


ROOT = Path(__file__).resolve().parent


def latest_frame() -> Path:
    frames = sorted((ROOT / "runs").glob("*/frame_0001.jpg"))
    if not frames:
        raise SystemExit("No saved ESP32 frame found in runs/. Capture one first.")
    return frames[-1].resolve()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=3)
    parser.add_argument("--vlm", action="store_true",
                        help="Send one saved ESP32 frame to GPT-6 Luna in bot 0")
    args = parser.parse_args()
    if not 1 <= args.steps <= 150:
        parser.error("--steps must be between 1 and 150")
    executable = Path(sys.executable).with_name("flwr.exe" if os.name == "nt" else "flwr")
    if not executable.exists():
        found = shutil.which("flwr")
        if found is None:
            parser.error("Install the project dependencies in a virtual environment first")
        executable = Path(found)
    env = os.environ.copy()
    env["FLWR_HOME"] = str(ROOT / ".flwr")
    env["FLWR_DISABLE_RUNTIME_DEPENDENCY_INSTALLATION"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    env["PATH"] = str(executable.parent) + os.pathsep + env.get("PATH", "")
    command = [str(executable), "run", ".", "--stream",
               "--federation-config", "num-supernodes=2 client-resources-num-cpus=1",
               "--run-config", f"steps={args.steps}"]
    if args.vlm:
        key = load_api_key()
        if not key:
            parser.error(f"Put your OpenAI key after OPENAI_API_KEY= in {ROOT / '.env'}")
        env["OPENAI_API_KEY"] = key
        frame = latest_frame()
        command.extend(["--run-config", "vlm-frame=" + json.dumps(frame.as_posix())])
        print(f"Using saved frame: {frame}", flush=True)
        print("Exactly one GPT-6 Luna image request will be attempted.", flush=True)
    else:
        print("Key-free Flower simulation: no API requests.", flush=True)
    # Flower can report a failed simulation while its CLI still exits zero.
    completed_summary = False
    with subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                          errors="replace", bufsize=1) as process:
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            completed_summary |= "FLOWER_SUMMARY " in line
        code = process.wait()
    raise SystemExit(code if code else (0 if completed_summary else 1))


if __name__ == "__main__":
    main()
