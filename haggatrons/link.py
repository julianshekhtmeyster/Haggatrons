"""Byte transport to the master ESP32: real USB serial or the in-process simulator."""

from __future__ import annotations

import threading
from typing import Callable, Protocol

from haggatrons.protocol import FrameReader, encode_frame

FrameHandler = Callable[[int, bytes], None]


class Link(Protocol):
    description: str

    def start(self, on_frame: FrameHandler, on_closed: Callable[[str], None]) -> None: ...
    def send(self, frame_type: int, payload: bytes = b"") -> None: ...
    def close(self) -> None: ...


class SerialLink:
    """USB CDC connection to the master. Only one process may hold the port."""

    def __init__(self, port: str, baudrate: int = 921600) -> None:
        import serial  # imported lazily so the simulator runs without pyserial

        self.description = f"serial:{port}"
        self._serial = serial.Serial(port=None, baudrate=baudrate, timeout=0.05, write_timeout=1)
        self._serial.dtr = False  # do not reset the board on open
        self._serial.rts = False
        self._serial.port = port
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self, on_frame: FrameHandler, on_closed: Callable[[str], None]) -> None:
        self._serial.open()
        self._serial.reset_input_buffer()
        reader = FrameReader()

        def run() -> None:
            reason = "closed"
            try:
                while not self._stop.is_set():
                    data = self._serial.read(4096)
                    if data:
                        for frame_type, payload in reader.feed(data):
                            on_frame(frame_type, payload)
            except Exception as exc:  # device unplugged, permission lost, ...
                reason = f"serial error: {exc}"
            finally:
                if not self._stop.is_set():
                    on_closed(reason)

        self._thread = threading.Thread(target=run, name="master-serial", daemon=True)
        self._thread.start()

    def send(self, frame_type: int, payload: bytes = b"") -> None:
        with self._write_lock:
            self._serial.write(encode_frame(frame_type, payload))

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        self._serial.close()


def list_serial_ports() -> list[dict]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    ports = []
    for port in list_ports.comports():
        if "Bluetooth" in port.device or "debug-console" in port.device:
            continue
        ports.append({"device": port.device, "description": port.description or "",
                      "vid": port.vid, "pid": port.pid,
                      "espressif": port.vid == 0x303A})
    return ports
