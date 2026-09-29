# Haggatrons engineering context

Updated 2026-09-29 (ESP-NOW control center branch). This is the handoff document for the current repository state. [PRODUCT.md](PRODUCT.md) defines the intended behavior; this file separates working code from work still to do.

## Decisions from the project discussion

- Build an open-source, human-supervised group of autonomous exploration bots for the [Flower Labs Stanford hackathon](https://luma.com/flwrlabs-bamu). Flower must carry meaningful worker/coordinator collaboration.
- The bots explore an unknown area. Each robot processes what it sees independently. A master coordinator assigns work, while each robot handles its own short movements.
- Four physical bots are planned from the parts list; the present Flower simulation has two workers, and only one physical W11 has been tested.
- Use OpenAI only for model calls. `gpt-6-luna` is the selected camera observer. Limit live VLM calls while testing.
- The Mac can remain the coordinator, but the operating bots must be battery-powered and physically disconnected from it. The wireless connection mechanism is not a product decision.
- Two N20 DC motors per bot are driven through one TB6612FNG H-bridge. The driver's part number does not identify how an assembled board is wired to ESP32 GPIOs.

## Verified repository state

Branch `feature/espnow-control-center` replaces the Wi-Fi/HTTP prototype with an ESP-NOW fleet. A master ESP32 plugged into the Mac relays to the robots, and a control page in Chrome, served by the Python backend, supervises. See [README.md](README.md), [ESP_NOW.md](ESP_NOW.md) and [PINOUT.md](PINOUT.md).

| Area | Verified | Not yet verified |
| --- | --- | --- |
| Firmware | `firmware/master` and `firmware/robot` compile for the W11 (Arduino ESP32 core 3.3.11, SensorLib 0.3.1, Pololu VL53L0X). | Not yet flashed to the boards. No ESP-NOW range or throughput measurements. |
| Protocol | The C header and the Python mirror are checked against each other by compiling the header in tests (struct sizes, constants, COBS, CRC). | — |
| Backend | 23 tests pass against the simulated master and robots: frame reassembly with dropped chunks, arming, e-stop, host-silence watchdog, clearance stop, pause, full local mission. | Not run against real hardware. |
| Flower | ServerApp/ClientApp rewritten: one worker per robot; observe → assign → decide → gate → execute. | Not executed yet: `flwr[simulation]` could not be installed on the build network. Run `python -m haggatrons mission --sim` once dependencies are installed. |
| Control page | React page typechecks, builds, and is served by `python -m haggatrons serve` (checked: page, assets, token-protected API, disarm when no page is open). | Not yet visually checked in a browser. |
| Motors | Firmware drives the TB6612 per PINOUT.md, with STBY low at boot, capped speed and duration, a heartbeat watchdog, a range-guarded forward move, and IMU-measured turns. | Wiring must be changed to PINOUT.md. Motor polarity, yaw sign and speed calibration still need the wheels-off-the-floor checks. |
| Localization | Dead reckoning from gyro yaw and commanded speed × time, with a growing uncertainty. The shared map uses range-sensor rays plus weak vision evidence. | No wheel encoders; pose drifts. Robots must start at the poses in `config/fleet.json`. |

Removed: `firmware/wifi_robot`, `provision_robot.py`, `wireless_capture.py`, `run_flower.py`, `WIRELESS_ROBOT.md`, `FLOWER.md`.

## Code map

- `haggatrons/protocol.py`: wire protocol mirror. `link.py`: USB serial link. `simlink.py`: simulated master and robots.
- `haggatrons/fleet.py`: arming, e-stop, keepalives, captures, moves, pose estimates. `mission.py`: mission lifecycle and the Flower subprocess. `server.py`: local token-protected API and event stream.
- `haggatrons/worldmap.py`: shared map, distinct goals, proposals, coordinator gate. `loop.py`: the per-tick worker and coordinator steps used by Flower.
- `flower_explore/`: the Flower ServerApp and ClientApp. `app/`: the control page (served from `app/dist`).
- `firmware/`: master, robot, shared protocol library, and the original USB camera bench sketch.

## Hardware facts and unknowns

The parts list contains four ESP32 camera boards, four 3.7 V LiPo batteries, eight N20 motors, and four TB6612FNG dual motor drivers. A TB6612FNG needs direction and PWM signals for channels A and B plus `STBY`, logic power, motor power, and common ground. The [Toshiba datasheet](https://toshiba.semicon-storage.com/info/datasheet_en_20141001.pdf?did=10660) defines the signals and electrical ratings. The [W11 pinout](https://wiki.meshnology.com/W11/W11_ESP32S3_Mini_Module/) shows available GPIOs. `WIRELESS_ROBOT.md` proposes a seven-pin assignment that avoids the pins used by the current camera and IMU sketch. Treat it as a proposed wiring plan, not a measured connection on the assembled bot.

Before loaded movement, determine the actual GPIO-to-driver connections, verify a common ground and supply routing, and check the motor stall current against the driver at the intended motor-supply voltage. A camera/IMU-only firmware build cannot infer any of these physical connections.

## Next milestones, in order

1. **Untethered observation:** configure a suitable wireless connection, unplug USB, run from the LiPo, and capture a fresh JPEG and IMU sample on the Mac. Record connection stability and observation latency. No VLM call is needed for this check.
2. **Motor bring-up:** confirm the actual TB6612FNG wiring and power, add a local driver with zero output on boot, bounded PWM and duration, explicit stop, and a watchdog independent of Mac responsiveness. Test with wheels off the floor before floor movement.
3. **One physical worker:** turn the live observation into a worker report with uncertainty and status. Feed it into the Flower coordinator. Keep VLM proposals advisory and log proposed versus executed action.
4. **Closed loop:** issue one safe, short assignment; have the bot act locally; collect the outcome; re-observe; and allow a human to stop at any point.
5. **Team behavior:** add a second physical bot, then scale toward four. Verify that workers share useful discoveries, receive distinct goals, and avoid conflicting movements. Maintain a repeatable simulated path for regression checks.

## Demo evidence to collect

Save one unplugged camera/IMU capture, a short action log, Flower messages showing two distinct assignments, and a stop or communication-loss test. Label simulated and physical evidence clearly. A live Luna call is useful once the physical loop is stable; saved images can cover most development and demo rehearsals.
