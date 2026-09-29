"""In-process stand-in for the master ESP32 and its robots.

SimLink speaks the same serial byte protocol as firmware/master, and each
SimRobot mirrors firmware/robot's safety rules (heartbeat watchdog, arming,
bounded moves, range-guarded forward, IMU-measured turns). The Mac-side code
cannot tell it apart from hardware, which keeps the full stack testable.
Simulated evidence is labelled "sim" everywhere it is logged.
"""

from __future__ import annotations

import io
import math
import queue
import random
import threading
import time
from dataclasses import dataclass, field

from haggatrons import protocol as p
from haggatrons.config import FleetConfig
from haggatrons.link import FrameHandler
from haggatrons.protocol import FrameReader, encode_frame

WHEEL_MPS = 0.3        # speed at 1000 permille, matches the default calibration
WHEELBASE_M = 0.09
ROBOT_RADIUS_M = 0.06
RANGE_MAX_MM = 2000
SIM_MASTER_MAC = bytes.fromhex("5a494d000001")

Segment = tuple[float, float, float, float]


def _box(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


@dataclass
class SimWorld:
    width: float = 4.0
    height: float = 3.0
    segments: list[Segment] = field(default_factory=list)

    @classmethod
    def default(cls) -> "SimWorld":
        world = cls()
        world.segments = (_box(0, 0, world.width, world.height) + _box(1.6, 1.6, 2.2, 2.0)
                          + _box(2.8, 0.4, 3.1, 1.3) + _box(1.2, 2.2, 1.3, 3.0))
        return world

    def raycast(self, x: float, y: float, angle: float, limit: float = 5.0) -> float:
        dx, dy = math.cos(angle), math.sin(angle)
        best = limit
        for x0, y0, x1, y1 in self.segments:
            sx, sy = x1 - x0, y1 - y0
            denom = dx * sy - dy * sx
            if abs(denom) < 1e-9:
                continue
            t = ((x0 - x) * sy - (y0 - y) * sx) / denom
            u = ((x0 - x) * dy - (y0 - y) * dx) / denom
            if 0 <= u <= 1 and 0 < t < best:
                best = t
        return best

    def clearance(self, x: float, y: float) -> float:
        best = math.inf
        for x0, y0, x1, y1 in self.segments:
            sx, sy = x1 - x0, y1 - y0
            length2 = sx * sx + sy * sy
            t = max(0.0, min(1.0, ((x - x0) * sx + (y - y0) * sy) / length2)) if length2 else 0.0
            best = min(best, math.hypot(x - (x0 + t * sx), y - (y0 + t * sy)))
        return best


def render_view(world: SimWorld, x: float, y: float, heading: float, robot_id: int,
                width: int = 320, height: int = 240, quality: int = 70) -> bytes:
    """Tiny raycaster so simulated frames look like a forward camera."""
    from PIL import Image, ImageDraw

    image = Image.new("RGB", (width, height), (38, 42, 52))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, height // 2, width, height), fill=(92, 84, 72))
    fov = math.radians(62)
    columns = 80
    step = width / columns
    for column in range(columns):
        offset = (column + 0.5) / columns - 0.5
        angle = heading - offset * fov
        distance = max(0.05, world.raycast(x, y, angle) * math.cos(offset * fov))
        wall = min(height, int(height * 0.35 / distance))
        shade = max(40, min(230, int(230 / (1 + distance))))
        top = (height - wall) // 2
        draw.rectangle((int(column * step), top, int((column + 1) * step), top + wall),
                       fill=(shade, shade, min(255, shade + 20)))
    draw.text((6, 6), f"SIM bot{robot_id}", fill=(255, 200, 0))
    buffer = io.BytesIO()
    image.save(buffer, "JPEG", quality=quality)
    return buffer.getvalue()


class SimRobot:
    def __init__(self, robot_id: int, mac: bytes, pose: tuple[float, float, float],
                 world: SimWorld, send: "callable", rng: random.Random, chunk_drop: float = 0.0) -> None:
        self.robot_id = robot_id
        self.mac = mac
        self.x, self.y = pose[0], pose[1]
        self.heading = math.radians(pose[2])
        self.world = world
        self.send_to_master = send
        self.rng = rng
        self.chunk_drop = chunk_drop
        self.seq = 0
        self.hb_ms: float | None = None
        self.hb_flags = p.HB_ESTOP
        self.left = self.right = 0
        self.move: dict | None = None
        self.last_outcome = p.OUT_COMPLETED
        self.frame = b""
        self.frame_req = 0
        self.last_telemetry = 0.0
        self.gyro_z = 0.0
        self.bumped = False

    # -------------------------------------------------------------- state
    def now_ms(self) -> float:
        return time.monotonic() * 1000

    def link_ok(self) -> bool:
        return self.hb_ms is not None and self.now_ms() - self.hb_ms < p.LINK_TIMEOUT_MS

    def armed(self) -> bool:
        return self.link_ok() and bool(self.hb_flags & p.HB_ARMED) and not self.hb_flags & p.HB_ESTOP

    def range_mm(self) -> int:
        front_x = self.x + math.cos(self.heading) * 0.05
        front_y = self.y + math.sin(self.heading) * 0.05
        distance = self.world.raycast(front_x, front_y, self.heading) * 1000
        return int(min(RANGE_MAX_MM, max(0, distance + self.rng.gauss(0, 4))))

    # -------------------------------------------------------------- outgoing
    def _send(self, msg_type: int, body: bytes) -> None:
        self.seq += 1
        self.send_to_master(self.mac, p.pack_message(msg_type, self.robot_id, self.seq, body))

    def _reject(self, req_id: int, ref: int, code: int) -> None:
        self._send(p.MSG_REJECT, p.REJECT.pack(req_id, ref, code))

    def _telemetry(self) -> None:
        flags = p.TEL_IMU_OK | p.TEL_RANGE_OK | p.TEL_CAMERA_OK
        if self.armed():
            flags |= p.TEL_ARMED
        if self.hb_flags & p.HB_ESTOP:
            flags |= p.TEL_ESTOP
        if self.link_ok():
            flags |= p.TEL_LINK_OK
        if self.move:
            flags |= p.TEL_MOVING
        age = 0xFFFF if self.hb_ms is None else min(0xFFFF, int(self.now_ms() - self.hb_ms))
        telemetry = p.Telemetry(int(self.now_ms()) & 0xFFFFFFFF, flags, age, (0.0, 0.0, 1.0),
                                (0.0, 0.0, self.gyro_z), self.range_mm(),
                                self.move["req_id"] if self.move else 0, self.last_outcome, "1.0")
        self._send(p.MSG_TELEMETRY, telemetry.pack())

    # -------------------------------------------------------------- incoming
    def receive(self, data: bytes) -> None:
        try:
            header, body = p.unpack_message(data)
        except p.ProtocolError:
            return
        if header.robot_id != self.robot_id:
            return
        if header.type == p.MSG_HEARTBEAT and len(body) == p.HEARTBEAT.size:
            self.hb_flags, _ = p.HEARTBEAT.unpack(body)
            self.hb_ms = self.now_ms()
        elif header.type == p.MSG_STOP and len(body) == p.STOP.size:
            (req_id,) = p.STOP.unpack(body)
            if self.move and req_id in (0, self.move["req_id"]):
                self._finish(p.OUT_ESTOP if self.hb_flags & p.HB_ESTOP else p.OUT_HOST_STOP)
            self.left = self.right = 0
        elif header.type == p.MSG_MOVE and len(body) == p.MOVE.size:
            self._start_move(body)
        elif header.type == p.MSG_CAPTURE and len(body) == p.CAPTURE.size:
            self._capture(body)
        elif header.type == p.MSG_RESEND and len(body) >= p.RESEND.size:
            req_id, count = p.RESEND.unpack_from(body)
            if req_id != self.frame_req or not self.frame:
                return self._reject(req_id, p.MSG_RESEND, p.REJ_FRAME_GONE)
            for i in range(count):
                (index,) = p.struct.unpack_from("<H", body, p.RESEND.size + 2 * i)
                self._send_chunk(index, drop=False)

    def _start_move(self, body: bytes) -> None:
        req_id, cmd = p.MoveCommand.unpack(body)
        if self.hb_flags & p.HB_ESTOP:
            return self._reject(req_id, p.MSG_MOVE, p.REJ_ESTOP)
        if not self.link_ok():
            return self._reject(req_id, p.MSG_MOVE, p.REJ_LINK_DOWN)
        if not self.armed():
            return self._reject(req_id, p.MSG_MOVE, p.REJ_NOT_ARMED)
        if self.move:
            return self._reject(req_id, p.MSG_MOVE, p.REJ_BUSY)
        try:
            cmd.validate()
        except ValueError:
            return self._reject(req_id, p.MSG_MOVE, p.REJ_BAD_PARAMS)
        clear = cmd.min_clear_mm or 150
        if cmd.kind == p.MOVE_FORWARD and self.range_mm() < clear:
            return self._reject(req_id, p.MSG_MOVE, p.REJ_OBSTACLE)
        if cmd.kind == p.MOVE_DRIVE and cmd.left > 0 and cmd.right > 0 and self.range_mm() < clear:
            return self._reject(req_id, p.MSG_MOVE, p.REJ_OBSTACLE)
        self.move = {"req_id": req_id, "cmd": cmd, "start": self.now_ms(), "yaw": 0.0,
                     "min_range": 0xFFFF, "clear": clear, "applied": (0, 0)}
        self._control(0.0)

    def _finish(self, outcome: int) -> None:
        self.left = self.right = 0
        if not self.move:
            return
        move, self.move = self.move, None
        self.last_outcome = outcome
        cmd: p.MoveCommand = move["cmd"]
        result = p.MoveResult(move["req_id"], outcome, cmd.kind,
                              min(0xFFFF, int(self.now_ms() - move["start"])), move["yaw"],
                              None if move["min_range"] == 0xFFFF else move["min_range"],
                              *move["applied"])
        self._send(p.MSG_MOVE_RESULT, result.pack())

    def _drive(self, left: float, right: float, cap: int) -> None:
        self.left = int(max(-cap, min(cap, left)))
        self.right = int(max(-cap, min(cap, right)))
        self.move["applied"] = (self.left, self.right)

    def _control(self, dt: float) -> None:
        move = self.move
        if not move:
            return
        if not self.armed():
            if self.hb_flags & p.HB_ESTOP:
                return self._finish(p.OUT_ESTOP)
            return self._finish(p.OUT_LINK_LOST if not self.link_ok() else p.OUT_DISARMED)
        cmd: p.MoveCommand = move["cmd"]
        move["yaw"] += self.gyro_z * dt
        reading = self.range_mm()
        move["min_range"] = min(move["min_range"], reading)
        elapsed = self.now_ms() - move["start"]
        cap = 1000
        if cmd.kind == p.MOVE_DRIVE:
            if cmd.left > 0 and cmd.right > 0 and reading < move["clear"]:
                return self._finish(p.OUT_OBSTACLE)
            self._drive(cmd.left, cmd.right, cap)
        elif cmd.kind == p.MOVE_FORWARD:
            if reading < move["clear"]:
                return self._finish(p.OUT_OBSTACLE)
            trim = max(-150, min(150, move["yaw"] * 8))
            self._drive(cmd.left + trim, cmd.right - trim, cap)
        elif cmd.kind == p.MOVE_TURN:
            remaining = abs(cmd.turn_deg) - abs(move["yaw"])
            if remaining <= 2.0:
                return self._finish(p.OUT_COMPLETED)
            speed = max(abs(cmd.left), 300)
            if remaining < 20:
                speed = max(speed * 0.6, 300)
            sign = 1 if cmd.turn_deg > 0 else -1
            self._drive(-sign * speed, sign * speed, cap)
        if elapsed >= cmd.duration_ms:
            self._finish(p.OUT_TURN_TIMEOUT if cmd.kind == p.MOVE_TURN else p.OUT_COMPLETED)

    def _capture(self, body: bytes) -> None:
        req_id, framesize, quality = p.CAPTURE.unpack(body)
        if self.move:
            return self._reject(req_id, p.MSG_CAPTURE, p.REJ_BUSY)
        size = {0: (320, 240), 1: (640, 480), 2: (800, 600), 3: (1024, 768)}.get(framesize)
        if size is None:
            return self._reject(req_id, p.MSG_CAPTURE, p.REJ_CAMERA_FAILED)
        frame_ms = int(self.now_ms()) & 0xFFFFFFFF
        self.frame = render_view(self.world, self.x, self.y, self.heading, self.robot_id, *size,
                                 quality=max(20, 100 - 2 * quality))
        self.frame_req = req_id
        chunk = p.CHUNK_DATA
        count = (len(self.frame) + chunk - 1) // chunk
        reading = self.range_mm()
        meta = p.FrameMeta(req_id, len(self.frame), count, chunk, size[0], size[1], frame_ms,
                           frame_ms, True, True, reading, (0.0, 0.0, 1.0), (0.0, 0.0, self.gyro_z))
        self._send(p.MSG_FRAME_META, meta.pack())
        for index in range(count):
            self._send_chunk(index, drop=True)

    def _send_chunk(self, index: int, drop: bool) -> None:
        chunk = p.CHUNK_DATA
        data = self.frame[index * chunk:(index + 1) * chunk]
        if not data or (drop and self.rng.random() < self.chunk_drop):
            return
        self._send(p.MSG_FRAME_CHUNK, p.FRAME_CHUNK.pack(self.frame_req, index) + data)

    # -------------------------------------------------------------- physics
    def step(self, dt: float) -> None:
        if self.move:
            self._control(dt)
        vl = self.left / 1000 * WHEEL_MPS
        vr = self.right / 1000 * WHEEL_MPS
        omega = (vr - vl) / WHEELBASE_M
        self.gyro_z = math.degrees(omega) + (self.rng.gauss(0, 0.3) if omega else 0.0)
        speed = (vl + vr) / 2
        heading = self.heading + omega * dt
        x = self.x + math.cos(heading) * speed * dt
        y = self.y + math.sin(heading) * speed * dt
        self.heading = math.atan2(math.sin(heading), math.cos(heading))
        if self.world.clearance(x, y) > ROBOT_RADIUS_M:
            self.x, self.y = x, y
            self.bumped = False
        else:
            self.bumped = True
        if self.now_ms() - self.last_telemetry >= p.TELEMETRY_PERIOD_MS:
            self.last_telemetry = self.now_ms()
            self._telemetry()


class SimLink:
    """Simulated master + robots behind the serial frame protocol."""

    def __init__(self, fleet: FleetConfig, seed: int = 7, chunk_drop: float = 0.0,
                 world: SimWorld | None = None) -> None:
        self.description = "sim"
        self.fleet = fleet
        self.world = world or SimWorld.default()
        self.rng = random.Random(seed)
        self._inbox: queue.Queue[bytes] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._host_reader = FrameReader(limit=16384)
        self._on_frame: FrameHandler | None = None
        self.robots = {
            robot.id: SimRobot(robot.id, robot.mac, robot.start_pose, self.world, self._robot_sent,
                               self.rng, chunk_drop)
            for robot in fleet.robots
        }
        self.configured: set[int] = set()
        self.host_flags = p.HB_ESTOP
        self.last_host_ms: float | None = None
        self.stats = {robot_id: {"last_rx": None, "tx_ok": 0, "tx_fail": 0} for robot_id in self.robots}
        self._outbox: list[tuple[int, bytes]] = []
        self._lock = threading.Lock()

    # -------------------------------------------------------------- Link API
    def start(self, on_frame: FrameHandler, on_closed) -> None:
        self._on_frame = on_frame
        self._thread = threading.Thread(target=self._run, name="sim-master", daemon=True)
        self._thread.start()
        self._emit(p.SER_HELLO, p.SER_HELLO_S.pack(p.PROTOCOL_VERSION, 1, 0, SIM_MASTER_MAC,
                                                   p.ESPNOW_MAX_PAYLOAD))

    def send(self, frame_type: int, payload: bytes = b"") -> None:
        self._inbox.put(encode_frame(frame_type, payload))

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)

    def pose(self, robot_id: int) -> tuple[float, float, float]:
        robot = self.robots[robot_id]
        return robot.x, robot.y, math.degrees(robot.heading)

    # -------------------------------------------------------------- master logic
    def _now(self) -> float:
        return time.monotonic() * 1000

    def _host_ok(self) -> bool:
        return self.last_host_ms is not None and self._now() - self.last_host_ms < p.HOST_TIMEOUT_MS

    def _flags(self) -> int:
        if not self._host_ok():
            return p.HB_ESTOP
        flags = (self.host_flags & (p.HB_ARMED | p.HB_ESTOP)) | p.HB_HOST_OK
        return flags & ~p.HB_ARMED if flags & p.HB_ESTOP else flags

    def _emit(self, frame_type: int, payload: bytes) -> None:
        with self._lock:
            self._outbox.append((frame_type, payload))

    def _robot_sent(self, mac: bytes, data: bytes) -> None:
        for robot_id, robot in self.robots.items():
            if robot.mac == mac and robot_id in self.configured:
                self.stats[robot_id]["last_rx"] = self._now()
                self._emit(p.SER_RECV, p.SER_RECV_HEADER.pack(robot_id, -42) + data)

    def _to_robot(self, robot_id: int, data: bytes) -> None:
        if robot_id in self.configured:
            self.stats[robot_id]["tx_ok"] += 1
            self.robots[robot_id].receive(data)

    def _handle_host(self, frame_type: int, payload: bytes) -> None:
        self.last_host_ms = self._now()
        if frame_type == p.SER_HELLO_REQ:
            self._emit(p.SER_HELLO, p.SER_HELLO_S.pack(p.PROTOCOL_VERSION, 1, 0, SIM_MASTER_MAC,
                                                       p.ESPNOW_MAX_PAYLOAD))
        elif frame_type == p.SER_CONFIG:
            try:
                _, _, peers = p.unpack_config(payload)
            except (p.ProtocolError, ValueError):
                return self._emit(p.SER_CONFIG_ACK, p.SER_CONFIG_ACK_S.pack(0, 0))
            known = {robot.mac: robot_id for robot_id, robot in self.robots.items()}
            self.configured = {known[peer.mac] for peer in peers if peer.mac in known}
            self._emit(p.SER_CONFIG_ACK, p.SER_CONFIG_ACK_S.pack(1, len(peers)))
        elif frame_type == p.SER_KEEPALIVE and len(payload) == 1:
            flags = payload[0] & (p.HB_ARMED | p.HB_ESTOP)
            changed, self.host_flags = flags != self.host_flags, flags
            if changed:
                self._heartbeats()
        elif frame_type == p.SER_STOP_ALL:
            self.host_flags = p.HB_ESTOP
            self._heartbeats()
            for robot_id in self.configured:
                self._to_robot(robot_id, p.pack_message(p.MSG_STOP, robot_id, 0, p.STOP.pack(0)))
        elif frame_type == p.SER_SEND and payload:
            robot_id, message = payload[0], payload[1:]
            try:
                header, _ = p.unpack_message(message)
            except p.ProtocolError:
                return
            if header.type == p.MSG_MOVE and not self._flags() & p.HB_ARMED:
                return self._emit(p.SER_LOG, b"dropped move while disarmed")
            targets = self.configured if robot_id == p.BROADCAST else {robot_id}
            for target in targets:
                self._to_robot(target, message)

    def _heartbeats(self) -> None:
        body = p.HEARTBEAT.pack(self._flags(), int(self._now()) & 0xFFFFFFFF)
        for robot_id in self.configured:
            self._to_robot(robot_id, p.pack_message(p.MSG_HEARTBEAT, robot_id, 0, body))

    def _status(self) -> None:
        now = self._now()
        status = p.MasterStatus(int(now) & 0xFFFFFFFF, self._flags(), self.fleet.channel, {
            robot_id: {"last_rx_age_ms": None if s["last_rx"] is None else int(now - s["last_rx"]),
                       "tx_ok": s["tx_ok"], "tx_fail": s["tx_fail"]}
            for robot_id, s in self.stats.items() if robot_id in self.configured})
        self._emit(p.SER_STATUS, status.pack())

    def _run(self) -> None:
        last = time.monotonic()
        last_beat = last_status = 0.0
        while not self._stop.is_set():
            while True:
                try:
                    data = self._inbox.get_nowait()
                except queue.Empty:
                    break
                for frame_type, payload in self._host_reader.feed(data):
                    self._handle_host(frame_type, payload)
            now = time.monotonic()
            dt, last = now - last, now
            if self.configured and (now - last_beat) * 1000 >= p.HEARTBEAT_PERIOD_MS:
                last_beat = now
                self._heartbeats()
            for robot_id, robot in self.robots.items():
                if robot_id in self.configured:
                    robot.step(dt)
            if now - last_status >= 1.0:
                last_status = now
                self._status()
            with self._lock:
                outbox, self._outbox = self._outbox, []
            for frame_type, payload in outbox:
                # Round-trip through the real framing so codec bugs surface here too.
                encoded = encode_frame(frame_type, payload)
                for decoded_type, decoded_payload in FrameReader(limit=16384).feed(encoded):
                    if self._on_frame:
                        self._on_frame(decoded_type, decoded_payload)
            time.sleep(0.005)
