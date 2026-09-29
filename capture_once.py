"""Capture one JPEG from the W11 USB camera firmware for a quick hardware check."""

from __future__ import annotations

import io
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "tools"))

from PIL import Image
import serial

from usb_camera_viewer import RESOLUTIONS, read_frame


def main() -> None:
    port_name = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    resolution = sys.argv[2] if len(sys.argv) > 2 else "640×480 balanced"
    if resolution not in RESOLUTIONS:
        raise SystemExit(f"Resolution must be one of: {', '.join(RESOLUTIONS)}")
    port = serial.Serial(port=None, baudrate=921600, timeout=0.5, write_timeout=2)
    port.dtr = False
    port.rts = False
    port.port = port_name
    port.open()
    try:
        time.sleep(1)
        port.reset_input_buffer()
        port.write(b"R" + RESOLUTIONS[resolution])
        time.sleep(0.2)
        started = time.monotonic()
        data = read_frame(port)
        elapsed = time.monotonic() - started
    finally:
        port.close()
    image = Image.open(io.BytesIO(data))
    image.verify()
    output = Path(__file__).resolve().parent / "captures" / (
        "frame_" + resolution.split(" ")[0].replace("×", "x") + ".jpg"
    )
    output.parent.mkdir(exist_ok=True)
    output.write_bytes(data)
    print(
        f"Saved {len(data)}-byte JPEG ({image.width}x{image.height}) "
        f"in {elapsed:.2f}s to {output}"
    )


if __name__ == "__main__":
    main()
