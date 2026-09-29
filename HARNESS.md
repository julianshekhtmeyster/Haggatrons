# Camera and perception harness

The ESP32 firmware in `firmware/usb_camera/usb_camera.ino` accepts USB commands `C` (JPEG frame), `R` (resolution), and `O` (paired JPEG and raw IMU sample). The IMU values are acceleration and angular rate, not a pose estimate. This is a bench transport, not motor control.

Install the `camera` optional dependencies as shown in `README.md`. Find your serial port (typically `/dev/cu.*` on macOS or `COM5` on Windows), then close any camera preview before capture:

```sh
.venv/bin/python perception_harness.py --port PORT --capture-only --count 3
```

For a single OpenAI observation, put your key after `OPENAI_API_KEY=` in the project root `.env` and run:

```sh
.venv/bin/python perception_harness.py --port PORT --count 1 --goal 'Find an open direction to explore'
```

Captures, metadata, model output, and API usage go into timestamped `runs/` folders, which Git ignores. A model proposal is never executed as a motor command. `.env` is also Git ignored, but it may still be copied by your computer's sync or backup service.
