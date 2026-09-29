# ESP32 USB camera

`firmware/usb_camera/usb_camera.ino` implements the bench camera protocol. `C` returns a `CAM1` header, a four byte little endian JPEG length, and JPEG bytes. `O` returns a camera frame with timestamped raw IMU readings. See `observation_protocol.py` for decoding.

The Python capture and viewer tools need Pillow and pyserial (`pip install -e '.[camera]'`). Pass the serial port for your device to `capture_once.py` or `perception_harness.py`. The `open_camera.cmd` launcher is for Windows; on macOS run `usb_camera_viewer.py` directly with Python and a port argument. Only one process may open the serial port at once.

The code cannot confirm that a board is connected or running this firmware. Verify the port and firmware on the target hardware before using a capture command.
