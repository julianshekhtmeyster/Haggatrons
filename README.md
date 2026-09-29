# Haggatrons

An initial two-robot exploration prototype for a planned four-bot build, with three connected parts:

- `sim.py` explores a grid with two simulated robots, a shared map, distinct target assignments, and a movement safety gate. It needs only Python 3.11+.
- `run_flower.py` runs the same coordination loop through two Flower workers. It can optionally send one saved camera frame to an OpenAI observer. This proposal is logged; it does not drive the robots.
- `perception_harness.py` reads a JPEG and IMU sample from an ESP32 camera over USB, optionally requests a visual proposal from OpenAI, and saves results in `runs/`.
- `firmware/wifi_robot` and `wireless_capture.py` provide battery-powered camera and IMU capture over the same Wi-Fi network as the Mac. See [WIRELESS_ROBOT.md](WIRELESS_ROBOT.md).

There are no motor commands in this project yet. The grid sensor is simulated; a real camera frame does not update the grid map. See [HARNESS.md](HARNESS.md) for the USB protocol and [CAMERA_SETUP.md](CAMERA_SETUP.md) for the bench firmware.

## Setup

Use Python 3.11 or newer. On macOS or Linux:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[camera]'
```

On Windows, use `.venv\Scripts\python.exe` in place of `.venv/bin/python`.

## Run without an API key

```sh
.venv/bin/python sim.py --steps 30
.venv/bin/python run_flower.py --steps 3
```

`sim.py` also works with any Python 3.11+ interpreter without installing dependencies. The Flower run requires the dependencies above. Both commands print coverage and movement results.

## OpenAI key

Copy `.env.example` to `.env` in this folder, then put your key after `OPENAI_API_KEY=`:

```text
OPENAI_API_KEY=your_key_here
```

The program also accepts an `OPENAI_API_KEY` environment variable. `.env` is ignored by Git. Keep the key out of chat and commits. No key is needed for either key free simulation. To run a model observation, first capture a frame with `perception_harness.py --capture-only --port PORT`, then run `run_flower.py --steps 1 --vlm`. On macOS, `PORT` is usually a `/dev/cu.*` device; on Windows it may be `COM5`. Only one process can hold the port at a time.

The visual observer defaults to `gpt-6-luna` and requires API access to that model. A live OpenAI request also requires a connected camera or a saved frame. The OpenAI path can be tested separately with `perception_harness.py --port PORT --count 1`.

The original MIT license notice is retained in `LICENSE`.
