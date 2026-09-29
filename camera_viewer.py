"""Small desktop viewer for the W11 camera's HTTP JPEG endpoint.

Run with: python camera_viewer.py --host 192.168.x.x
The host can also be entered or changed in the window.
"""

from __future__ import annotations

import argparse
import io
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk
from urllib.error import URLError
from urllib.request import urlopen

from PIL import Image, ImageTk


class CameraViewer:
    def __init__(self, root: tk.Tk, host: str):
        self.root = root
        self.root.title("W11 Robot Camera")
        self.root.geometry("900x720")
        self.host = tk.StringVar(value=host)
        self.status = tk.StringVar(value="Enter the board IP address and click Connect")
        self.frames: queue.Queue[tuple[str, object]] = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.last_jpeg: bytes | None = None
        self.photo: ImageTk.PhotoImage | None = None
        self.frame_count = 0
        self.start_time = 0.0

        bar = ttk.Frame(root, padding=10)
        bar.pack(fill="x")
        ttk.Label(bar, text="Board IP:").pack(side="left")
        ttk.Entry(bar, textvariable=self.host, width=24).pack(side="left", padx=6)
        ttk.Button(bar, text="Connect", command=self.connect).pack(side="left")
        ttk.Button(bar, text="Save frame", command=self.save_frame).pack(side="left", padx=6)
        ttk.Label(root, textvariable=self.status, padding=(10, 0)).pack(fill="x")

        self.image_label = ttk.Label(root, text="Waiting for camera", anchor="center")
        self.image_label.pack(fill="both", expand=True, padx=10, pady=10)
        root.after(50, self.process_frames)
        root.protocol("WM_DELETE_WINDOW", self.close)
        if host:
            self.connect()

    def connect(self) -> None:
        host = self.host.get().strip().removeprefix("http://").removeprefix("https://")
        host = host.split("/")[0]
        if not host:
            self.status.set("Enter the IP address printed by the board on COM5")
            return
        self.host.set(host)
        self.stop_event.set()
        self.stop_event = threading.Event()
        self.frame_count = 0
        self.start_time = time.monotonic()
        self.status.set(f"Connecting to http://{host}/capture ...")
        self.worker = threading.Thread(
            target=self.capture_loop, args=(host, self.stop_event), daemon=True
        )
        self.worker.start()

    def capture_loop(self, host: str, stop: threading.Event) -> None:
        url = f"http://{host}/capture"
        while not stop.is_set():
            try:
                with urlopen(url, timeout=2) as response:
                    data = response.read(4_000_000)
                if not data.startswith(b"\xff\xd8"):
                    raise ValueError("Camera response was not a JPEG")
                self.put_latest(("frame", data))
            except (URLError, TimeoutError, OSError, ValueError) as exc:
                self.put_latest(("error", str(exc)))
                stop.wait(1.0)
            else:
                stop.wait(0.15)

    def put_latest(self, item: tuple[str, object]) -> None:
        try:
            self.frames.put_nowait(item)
        except queue.Full:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            self.frames.put_nowait(item)

    def process_frames(self) -> None:
        try:
            while True:
                kind, payload = self.frames.get_nowait()
                if kind == "error":
                    self.status.set(f"Camera unavailable: {payload}")
                    continue
                data = payload
                assert isinstance(data, bytes)
                self.last_jpeg = data
                image = Image.open(io.BytesIO(data))
                image.thumbnail((860, 620))
                self.photo = ImageTk.PhotoImage(image)
                self.image_label.configure(image=self.photo, text="")
                self.frame_count += 1
                elapsed = max(time.monotonic() - self.start_time, 0.1)
                self.status.set(
                    f"Connected • {image.width}×{image.height} • "
                    f"{self.frame_count / elapsed:.1f} frames/s • frame {self.frame_count}"
                )
        except queue.Empty:
            pass
        self.root.after(50, self.process_frames)

    def save_frame(self) -> None:
        if self.last_jpeg is None:
            self.status.set("No image to save yet")
            return
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save camera frame",
            initialfile=time.strftime("w11_%Y%m%d_%H%M%S.jpg"),
            defaultextension=".jpg",
            filetypes=[("JPEG image", "*.jpg")],
        )
        if path:
            Path(path).write_bytes(self.last_jpeg)
            self.status.set(f"Saved {path}")

    def close(self) -> None:
        self.stop_event.set()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="", help="W11 IP address on your Wi-Fi")
    args = parser.parse_args()
    root = tk.Tk()
    CameraViewer(root, args.host)
    root.mainloop()


if __name__ == "__main__":
    main()
