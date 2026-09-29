# ESP32 USB camera

`firmware/usb_camera/usb_camera.ino` implements the bench camera protocol. `C` returns a `CAM1` header, a four byte little endian JPEG length, and JPEG bytes. `O` returns a camera frame with timestamped raw IMU readings. See `observation_protocol.py` for decoding.

The Python capture and viewer tools need Pillow and pyserial (`pip install -e '.[camera]'`). Pass the serial port for your device to `capture_once.py` or `perception_harness.py`. The `open_camera.cmd` launcher is for Windows; on macOS run `usb_camera_viewer.py` directly with Python and a port argument. Only one process may open the serial port at once.

On the test Mac, the W11 appears at `/dev/cu.usbmodem101`. Its original firmware identifies itself as `W11 Factory Test + Data Output`; it passed camera and QMI8658C IMU self-tests, but does not implement the `OBS1` capture protocol. A full 16 MB factory flash backup is saved locally at `firmware/backups/w11_factory_16mb_20260929.bin` and excluded from Git (SHA-256 `dcc13ec3e3d367e75035a00e97b476428ed185d5278f3c63fc01841d3e5abf59`).

The working firmware build uses Espressif Arduino ESP32 core 3.0.7, SensorLib 0.3.1, 16 MB flash, OPI PSRAM, and USB CDC on boot. Add `https://espressif.github.io/arduino-esp32/package_esp32_index.json` to Arduino CLI's board manager URLs, then run:

```sh
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.0.7
arduino-cli lib install SensorLib@0.3.1
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc' --build-path firmware/usb_camera/build firmware/usb_camera
arduino-cli upload --fqbn 'esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc,UploadSpeed=115200' --port /dev/cu.usbmodem101 --build-path firmware/usb_camera/build --verify firmware/usb_camera
```

After flashing, capture one frame with `perception_harness.py --port /dev/cu.usbmodem101 --capture-only --count 1`. Inspect its JPEG and IMU metadata before running without `--capture-only`, which sends a frame to the default `gpt-6-luna` visual observer and uses one OpenAI API call. The first hardware check produced a 640×480 JPEG with valid IMU data; one Luna request returned a structured observation. No motor commands were issued.
