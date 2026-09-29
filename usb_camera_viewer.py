"""Preview JPEG frames from the Meshnology W11 over its USB serial port."""

from __future__ import annotations

import io
import queue
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

sys.path.insert(0, str(Path(__file__).resolve().parent / "tools"))

from PIL import Image, ImageTk
import serial


MAGIC = b"CAM1"
MAX_FRAME = 2_000_000
RESOLUTIONS = {
    "320×240 preview": b"0",
    "640×480 balanced": b"1",
    "800×600 detail": b"2",
    "1024×768 detail": b"3",
}


def read_frame(port: serial.Serial) -> bytes:
    port.write(b"C")
    deadline = time.monotonic() + 15
    prefix = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(1)
        if not chunk:
            continue
        prefix += chunk
        if len(prefix) > 100:
            del prefix[:-8]
        if prefix.endswith(MAGIC):
            break
    else:
        raise TimeoutError("No camera frame arrived from the board")

    size_bytes = port.read(4)
    if len(size_bytes) != 4:
        raise TimeoutError("Incomplete frame header")
    size = int.from_bytes(size_bytes, "little")
    if not 0 < size <= MAX_FRAME:
        raise ValueError(f"Invalid frame size: {size}")
    frame = bytearray()
    while len(frame) < size and time.monotonic() < deadline:
        frame.extend(port.read(min(size - len(frame), 8192)))
    if len(frame) != size or not frame.startswith(b"\xff\xd8"):
        raise ValueError("Incomplete JPEG frame")
    return bytes(frame)


class Viewer:
    def __init__(self, root: tk.Tk, port_name: str):
        self.root = root
        root.title("W11 USB Camera")
        root.geometry("900x720")
        self.port_name = tk.StringVar(value=port_name)
        self.status = tk.StringVar(value="Click Connect to capture a frame")
        self.stop = threading.Event()
        self.messages: queue.Queue[tuple[str, object]] = queue.Queue(maxsize=3)
        self.last_jpeg: bytes | None = None
        self.photo: ImageTk.PhotoImage | None = None
        self.frames = 0
        self.started = time.monotonic()
        self.requested_resolution = "640×480 balanced"

        bar = ttk.Frame(root, padding=10)
        bar.pack(fill="x")
        ttk.Label(bar, text="Serial port:").pack(side="left")
        ttk.Entry(bar, textvariable=self.port_name, width=12).pack(side="left", padx=6)
        ttk.Button(bar, text="Connect", command=self.connect).pack(side="left")
        resolution = ttk.Combobox(bar, values=list(RESOLUTIONS), width=20, state="readonly")
        resolution.set(self.requested_resolution)
        resolution.pack(side="left", padx=6)
        resolution.bind(
            "<<ComboboxSelected>>",
            lambda _event: setattr(self, "requested_resolution", resolution.get()),
        )
        ttk.Button(bar, text="Save frame", command=self.save_frame).pack(side="left", padx=6)
        ttk.Label(root, textvariable=self.status, padding=(10, 0)).pack(fill="x")
        self.picture = ttk.Label(root, text="Waiting for camera", anchor="center")
        self.picture.pack(fill="both", expand=True, padx=10, pady=10)
        root.after(50, self.poll)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.connect()

    def connect(self) -> None:
        self.stop.set()
        self.stop = threading.Event()
        self.frames = 0
        self.started = time.monotonic()
        name = self.port_name.get().strip()
        self.status.set(f"Opening {name} ...")
        threading.Thread(target=self.worker, args=(name, self.stop), daemon=True).start()

    def worker(self, name: str, stop: threading.Event) -> None:
        try:
            port = serial.Serial(port=None, baudrate=921600, timeout=0.5, write_timeout=2)
            port.dtr = False
            port.rts = False
            port.port = name
            port.open()
        except (serial.SerialException, ValueError) as exc:
            self.put(("error", str(exc)))
            return
        try:
            time.sleep(1.0)
            port.reset_input_buffer()
            current_resolution = None
            while not stop.is_set():
                try:
                    desired = self.requested_resolution
                    if desired != current_resolution:
                        port.write(b"R" + RESOLUTIONS[desired])
                        time.sleep(0.2)
                        current_resolution = desired
                    frame = read_frame(port)
                    self.put(("frame", frame))
                except (serial.SerialException, TimeoutError, ValueError) as exc:
                    self.put(("error", str(exc)))
                    stop.wait(1)
                else:
                    stop.wait(0.1)
        finally:
            port.close()

    def put(self, message: tuple[str, object]) -> None:
        try:
            self.messages.put_nowait(message)
        except queue.Full:
            try:
                self.messages.get_nowait()
            except queue.Empty:
                pass
            self.messages.put_nowait(message)

    def poll(self) -> None:
        try:
            while True:
                kind, payload = self.messages.get_nowait()
                if kind == "error":
                    self.status.set(str(payload))
                    continue
                data = payload
                assert isinstance(data, bytes)
                self.last_jpeg = data
                image = Image.open(io.BytesIO(data))
                dimensions = image.size
                image.thumbnail((860, 620))
                self.photo = ImageTk.PhotoImage(image)
                self.picture.configure(image=self.photo, text="")
                self.frames += 1
                elapsed = max(time.monotonic() - self.started, 0.1)
                self.status.set(
                    f"{self.port_name.get()} • {dimensions[0]}×{dimensions[1]} • "
                    f"{self.frames / elapsed:.1f} frames/s • frame {self.frames}"
                )
        except queue.Empty:
            pass
        self.root.after(50, self.poll)

    def save_frame(self) -> None:
        if self.last_jpeg is None:
            self.status.set("No frame to save yet")
            return
        name = filedialog.asksaveasfilename(
            parent=self.root,
            initialfile=time.strftime("w11_%Y%m%d_%H%M%S.jpg"),
            defaultextension=".jpg",
            filetypes=[("JPEG image", "*.jpg")],
        )
        if name:
            Path(name).write_bytes(self.last_jpeg)
            self.status.set(f"Saved {name}")

    def close(self) -> None:
        self.stop.set()
        self.root.destroy()


def main() -> None:
    port_name = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    root = tk.Tk()
    Viewer(root, port_name)
    root.mainloop()


if __name__ == "__main__":
    main()
