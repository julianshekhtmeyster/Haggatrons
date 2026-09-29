# Haggatrons

A human-supervised team of small camera robots that explore a room together. A Flower coordinator assigns each robot a distinct goal. Each robot's worker turns its goal into a short, bounded move. The robots' own firmware refuses anything unsafe. The operator watches everything in an Electron control centre and can stop the fleet at any time.

Start with [PRODUCT.md](PRODUCT.md) for the intent and [CONTEXT.md](CONTEXT.md) for what has been verified.

```
Electron app ⇄ Python backend ⇄ USB ⇄ master ESP32 ~~encrypted ESP-NOW~~ robot 1, robot 2 (…4)
                   │
                   └─ Flower: ServerApp (coordinator) + one ClientApp worker per robot
```

No router or Wi-Fi network is needed. The robots talk only to the master, which is plugged into the Mac.

## Layout

| Path | What it is |
| --- | --- |
| `app/` | Electron + React control centre (starts the backend for you) |
| `haggatrons/` | Backend: fleet link, safety state, missions, shared map, local API, simulator |
| `flower_explore/` | Flower ServerApp (coordinator) and ClientApp (robot worker) |
| `firmware/master/` | Master ESP32: USB ⇄ ESP-NOW relay, heartbeats, host watchdog |
| `firmware/robot/` | Robot: camera, IMU, range sensor, motors, local safety |
| `firmware/libraries/HaggatronsProtocol/` | Wire protocol shared by both sketches |
| `config/fleet.json` | Robot IDs, MACs, calibration, start poses (no secrets) |
| [PINOUT.md](PINOUT.md) | Robot wiring diagram |
| [ESP_NOW.md](ESP_NOW.md) | Board MACs and how the link works |
| `sim.py`, `perception_harness.py`, `firmware/usb_camera/` | Original grid simulator and USB bench tools |

## Setup

```sh
python3 -m venv .venv
.venv/bin/pip install -e '.[dev]'
cd app && npm install
```

## Run the control centre

```sh
cd app && npm start        # builds the UI, launches Electron, which starts the backend
```

Click **Use simulator** to try everything without hardware, or pick the master's port and click **Connect master**. Then:

1. **Arm**, then **Start mission** (runner: Flower). Robots move only while the fleet is armed and the mission is running.
2. **STOP** (or Esc, or ⌘.) e-stops every robot immediately. **Pause** halts motion and holds the mission.
3. The timeline records every observation, assignment and the reason for it, gate decision, and move outcome. Each mission also writes `runs/missions/<id>/events.jsonl`.

Simulated frames and moves are labelled SIM everywhere.

Headless: `python -m haggatrons mission --sim --steps 10` (add `--runner local` to skip Flower).

## Hardware bring-up

```sh
python -m haggatrons flash master --port /dev/cu.usbmodemXXXX
python -m haggatrons provision master --port /dev/cu.usbmodemXXXX
python -m haggatrons flash robot --port /dev/cu.usbmodemYYYY     # one robot at a time
python -m haggatrons provision robot --id 1 --port /dev/cu.usbmodemYYYY
```

`provision robot` checks the board's MAC against `config/fleet.json` before writing keys. Wire the robots per [PINOUT.md](PINOUT.md) and follow its wheels-off-the-floor checks before any floor run. Firmware builds need Arduino ESP32 core 3.3.x and `SensorLib@0.3.1` plus `VL53L0X` (Pololu) in your Arduino libraries.

## Tests

```sh
.venv/bin/python -m pytest
```

The tests cover protocol parity with the C header (compiled at test time), the shared map and gate, and the whole backend against the simulated master: capture with dropped chunks, arming, e-stop, the host-silence watchdog, clearance stops, pause, and a full mission.

## OpenAI key

Put `OPENAI_API_KEY=...` in `.env` (Git-ignored). Vision (`gpt-6-luna`) is off by default and capped per mission by a call budget. Its output is advisory: it adds weak evidence to the map and can steer a worker away from a blocked view. It never commands a motor directly.

The original MIT license notice is retained in `LICENSE`.
