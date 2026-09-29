"""Read a timestamped camera and IMU observation from the W11 over USB.

Protocol: send ``O``. The board replies with ``OBS1``, JPEG byte count (u32),
frame time (u32 ms), IMU time (u32 ms), IMU valid flag (u8), six little-endian
float32 values (acceleration in g, angular rate in degrees/second), and JPEG.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass


MAGIC = b"OBS1"
HEADER = struct.Struct("<4sIIIB6f")
MAX_FRAME = 2_000_000
RESOLUTIONS = {"320x240": b"0", "640x480": b"1", "800x600": b"2", "1024x768": b"3"}


@dataclass(frozen=True)
class Observation:
    jpeg: bytes
    frame_ms: int
    imu_ms: int
    imu_valid: bool
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]

    def metadata(self) -> dict:
        return {
            "board_frame_ms": self.frame_ms,
            "board_imu_ms": self.imu_ms,
            "imu_valid": self.imu_valid,
            "accel_g": dict(zip(("x", "y", "z"), self.accel_g)),
            "gyro_dps": dict(zip(("x", "y", "z"), self.gyro_dps)),
            "jpeg_bytes": len(self.jpeg),
        }


def _read_exact(port, count: int, deadline: float) -> bytes:
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(min(count - len(data), 8192)))
    if len(data) != count:
        raise TimeoutError(f"Serial observation ended after {len(data)}/{count} bytes")
    return bytes(data)


def read_observation(port, timeout: float = 15.0) -> Observation:
    port.write(b"O")
    deadline = time.monotonic() + timeout
    prefix = bytearray()
    while time.monotonic() < deadline:
        byte = port.read(1)
        if byte:
            prefix.extend(byte)
            if len(prefix) > 128:
                del prefix[:-8]
            if prefix.endswith(MAGIC):
                break
    else:
        raise TimeoutError("No OBS1 response from board; close the preview and check firmware")

    header = MAGIC + _read_exact(port, HEADER.size - len(MAGIC), deadline)
    _, length, frame_ms, imu_ms, valid, ax, ay, az, gx, gy, gz = HEADER.unpack(header)
    if length <= 0 or length > MAX_FRAME:
        raise ValueError(f"Board reported invalid JPEG length {length}")
    jpeg = _read_exact(port, length, deadline)
    if not (jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")):
        raise ValueError("Board returned an incomplete JPEG")
    return Observation(jpeg, frame_ms, imu_ms, bool(valid), (ax, ay, az), (gx, gy, gz))
