# Haggatrons engineering context

Updated 2026-09-29. This is the handoff document for the current repository state. [PRODUCT.md](PRODUCT.md) defines the intended behavior; this file separates working code from work still to do.

## Decisions from the project discussion

- Build an open-source, human-supervised group of autonomous exploration bots for the [Flower Labs Stanford hackathon](https://luma.com/flwrlabs-bamu). Flower must carry meaningful worker/coordinator collaboration.
- The bots explore an unknown area. Each robot processes what it sees independently. A master coordinator assigns work, while each robot handles its own short movements.
- Four physical bots are planned from the parts list; the present Flower simulation has two workers, and only one physical W11 has been tested.
- Use OpenAI only for model calls. `gpt-6-luna` is the selected camera observer. Limit live VLM calls while testing.
- The Mac can remain the coordinator, but the operating bots must be battery-powered and physically disconnected from it. The wireless connection mechanism is not a product decision.
- Two N20 DC motors per bot are driven through one TB6612FNG H-bridge. The driver's part number does not identify how an assembled board is wired to ESP32 GPIOs.

## Verified repository state

| Area | What is working | What is not yet working |
| --- | --- | --- |
| Simulation | `sim.py` has a grid world, shared map, frontier assignments, and a movement safety gate. | Its poses and obstacles are simulated, not estimated from hardware. |
| Flower | `run_flower.py` runs two `ClientApp` workers with a `ServerApp` coordinator; workers observe and propose and the server gates simulated moves. | It expects exactly two simulated nodes. It does not command physical motors. |
| Camera and IMU | The W11 firmware returned a 640×480 JPEG and valid QMI8658 IMU data after flashing the wireless sketch. `OBS1` carries a frame, timestamps, and six raw IMU values. | IMU values are not fused into position or a physical map. |
| Visual model | One live `gpt-6-luna` image request succeeded with structured output. The observer validates a bounded action proposal. | The proposal is only logged. It does not change the map or actuate a robot. |
| Wireless | `firmware/wifi_robot/wifi_robot.ino` is flashed and compiled. `provision_robot.py` and `wireless_capture.py` provide one-bot setup and capture tools. | No battery-powered, unplugged end-to-end capture has been verified yet. The current tools are a prototype, not a multi-bot transport. |
| Motors | The TB6612FNG signal requirements and a possible W11 GPIO map are documented in `WIRELESS_ROBOT.md`. | Actual wiring, power limits, motor firmware, watchdog, and movement tests remain unverified. Motor outputs are disabled in the current sketch. |

The latest wireless code was committed and pushed as `5ac8486` (`Feature: Add wireless W11 capture harness`). The local factory flash backup is ignored by Git. The project `.env` and `runs/` are also ignored. Never copy secret values into this document or commit them.

## Code map

- `sim.py`: grid world, simulated robot workers, shared-map coordinator, and safety gate.
- `flower_explore/server_app.py`: Flower server exchange, merging observations, target assignments, movement gate, and run summary.
- `flower_explore/client_app.py`: two simulated workers; optionally asks the visual observer about one saved JPEG.
- `flower_explore/openai_observer.py`: OpenAI Responses request, `gpt-6-luna` model choice, structured schema, and validation.
- `observation_protocol.py`: paired camera/IMU `OBS1` decoder.
- `firmware/usb_camera/usb_camera.ino`: known bench USB camera/IMU implementation.
- `firmware/wifi_robot/wifi_robot.ino`: current board firmware, preserving USB capture and adding an authenticated wireless observation endpoint; no motor commands.
- `provision_robot.py`, `wireless_capture.py`: one-bot provisioning and capture tools. Their local token/IP file is Git ignored.
- `perception_harness.py`: USB capture and optional VLM call; `runs/` contains ignored output.
- `WIRELESS_ROBOT.md`, `CAMERA_SETUP.md`, `HARNESS.md`, `FLOWER.md`: detailed setup and protocol notes.

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
