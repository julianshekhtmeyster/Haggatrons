# Battery-powered W11: first wireless milestone

The Mac stays on the same Wi-Fi as the battery-powered W11 and runs Flower and the OpenAI calls. The W11 runs the camera, IMU, and a small local HTTP service. The sketch accepts no motor commands yet. This keeps the already validated camera format (`OBS1`) while we verify the wireless link and identify the actual motor wiring.

## Flash and provision once

Connect the W11 by USB temporarily. Build and flash `firmware/wifi_robot/wifi_robot.ino` using the same Arduino ESP32 core, SensorLib, board settings, and upload procedure in [CAMERA_SETUP.md](CAMERA_SETUP.md), substituting `firmware/wifi_robot` for `firmware/usb_camera`. The `wifi_robot` sketch preserves the USB camera commands.

On the Mac, run:

```sh
.venv/bin/python provision_robot.py --port /dev/cu.usbmodem101 --ssid 'YOUR_2.4_GHZ_WIFI_NAME'
```

Type the Wi-Fi password at the local prompt. The tool sends it only to the W11 over USB, generates a separate 32-character access token, and saves the board IP and token to the Git-ignored `runs/robot_config.json` with owner-only file permissions. The Wi-Fi password is stored in the W11's nonvolatile storage; the Mac does not save it in the project. The Mac must join the same network. Client isolation on guest Wi-Fi can prevent local connections.

Unplug USB, power the W11 from the battery, then run:

```sh
.venv/bin/python wireless_capture.py
```

This requests `/status` and one `/observation`, saving a JPEG and IMU metadata under `runs/`. It makes **zero OpenAI calls**. If the Wi-Fi network assigns a new IP later, use `wireless_capture.py --host NEW_IP`; for a repeatable demo, reserve the W11's address in the router. The HTTP endpoints require the token in the `X-Robot-Token` header. This is local-network authentication, not encryption; use a trusted Wi-Fi network.

## Two DC motors through TB6612FNG

The driver uses two separate H-bridge channels. Connect motor 1 to AO1/AO2 and motor 2 to BO1/BO2. The seven logic signals are `AIN1`, `AIN2`, `PWMA`, `BIN1`, `BIN2`, `PWMB`, and `STBY`. `VCC` goes to the W11's 3.3 V logic supply, `VM` to the motor battery supply, and driver `GND` to the same ground as the W11 and battery. `STBY` low disables the outputs. Do not connect the motors directly to ESP32 GPIOs.

The W11 exposes GPIO1, GPIO2, GPIO6, GPIO7, GPIO8, GPIO9, and GPIO21, which this camera/IMU sketch does not use. They are a **proposed wiring set**, not a claim about how an already assembled bot is wired. A possible assignment is:

| TB6612FNG | W11 GPIO |
| --- | ---: |
| AIN1 | 1 |
| AIN2 | 2 |
| PWMA | 6 |
| BIN1 | 7 |
| BIN2 | 8 |
| PWMB | 9 |
| STBY | 21 |

GPIO21 may also be used as an SD-card select on some W11 configurations; this sketch does not use the SD card. If the bot is already wired, its **actual** GPIO-to-driver connections must be identified before flashing firmware that drives the motors. A motor-driver part number cannot determine those connections. Also verify each N20 motor's stall current against the driver's low-voltage current rating before a loaded run. The motor control endpoint will use short bounded commands and a local watchdog once that wiring and power are confirmed.
