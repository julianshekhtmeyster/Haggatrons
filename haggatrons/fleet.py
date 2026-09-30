"""Mac-side fleet controller: the only code that talks to the master ESP32.

It owns arming and e-stop, streams keepalives so the master (and through it
each robot) knows the Mac is alive, reassembles camera frames, runs move
requests with host-side checks on top of the firmware's own, and keeps a
dead-reckoned pose estimate per robot with an explicit uncertainty.
"""

from __future__ import annotations

import itertools
import math
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from haggatrons import protocol as p
from haggatrons.config import RUNS, FleetConfig, FleetKeys
from haggatrons.events import EventBus
from haggatrons.link import Link

TELEMETRY_STALE_S = 1.0
RESEND_AFTER_S = 0.6
CAPTURE_TIMEOUT_S = 8.0


class FleetError(RuntimeError):
    """A request the fleet refused or could not complete; the message is user-facing."""


@dataclass
class Pose:
    x: float
    y: float
    heading_deg: float
    sigma_m: float = 0.02
    sigma_deg: float = 2.0

    def as_dict(self) -> dict:
        return {"x": round(self.x, 3), "y": round(self.y, 3),
                "heading_deg": round(self.heading_deg, 1),
                "sigma_m": round(self.sigma_m, 3), "sigma_deg": round(self.sigma_deg, 1)}


@dataclass
class Capture:
    robot_id: int
    frame_id: str
    path: Path
    jpeg: bytes
    meta: p.FrameMeta
    captured_at: float
    pose: Pose
    simulated: bool

    def as_dict(self) -> dict:
        m = self.meta
        return {
            "robot_id": self.robot_id, "frame_id": self.frame_id, "path": str(self.path),
            "width": m.width, "height": m.height, "jpeg_bytes": len(self.jpeg),
            "captured_at": self.captured_at, "imu_valid": m.imu_valid,
            "accel_g": list(m.accel_g), "gyro_dps": list(m.gyro_dps),
            "range_mm": m.range_mm if m.range_valid else None,
            "pose": self.pose.as_dict(), "simulated": self.simulated,
        }


@dataclass
class _PendingCapture:
    req_id: int
    robot_id: int
    done: threading.Event = field(default_factory=threading.Event)
    meta: p.FrameMeta | None = None
    chunks: dict[int, bytes] = field(default_factory=dict)
    last_rx: float = field(default_factory=time.monotonic)
    error: str | None = None


@dataclass
class _PendingMove:
    req_id: int
    robot_id: int
    command: p.MoveCommand
    done: threading.Event = field(default_factory=threading.Event)
    result: p.MoveResult | None = None
    error: str | None = None


@dataclass
class RobotState:
    id: int
    name: str
    mac: str
    pose: Pose
    telemetry: dict | None = None
    telemetry_at: float | None = None
    rssi: int | None = None
    radio: dict | None = None
    last_capture: Capture | None = None
    last_move: dict | None = None
    moving_req: int | None = None

    def online(self) -> bool:
        return self.telemetry_at is not None and time.monotonic() - self.telemetry_at < TELEMETRY_STALE_S

    def as_dict(self) -> dict:
        return {
            "id": self.id, "name": self.name, "mac": self.mac, "online": self.online(),
            "pose": self.pose.as_dict(), "telemetry": self.telemetry, "rssi": self.rssi,
            "radio": self.radio,
            "last_capture": self.last_capture.as_dict() if self.last_capture else None,
            "last_move": self.last_move, "moving": self.moving_req is not None,
        }


class FleetController:
    def __init__(self, fleet: FleetConfig, keys: FleetKeys, events: EventBus,
                 frames_dir: Path | None = None) -> None:
        self.fleet = fleet
        self.keys = keys
        self.events = events
        self.frames_dir = frames_dir or RUNS / "frames" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.robots = {r.id: RobotState(r.id, r.name, p.format_mac(r.mac), Pose(*r.start_pose))
                       for r in fleet.robots}
        self.link: Link | None = None
        self.master: dict = {"connected": False, "configured": False}
        self.armed = False
        self.estop = False
        # Returns a refusal reason, or None when mission policy allows a move.
        self.mission_gate: Callable[[int, str], str | None] = lambda robot_id, source: None
        self._lock = threading.RLock()
        self._req_ids = itertools.count(int(time.time()) & 0xFFFF)
        self._seq = itertools.count(1)
        self._captures: dict[int, _PendingCapture] = {}
        self._moves: dict[int, _PendingMove] = {}
        self._stop = threading.Event()
        self._keepalive: threading.Thread | None = None

    # ------------------------------------------------------------ connection
    @property
    def simulated(self) -> bool:
        return self.link is not None and self.link.description == "sim"

    def connect(self, link: Link) -> None:
        self.disconnect()
        with self._lock:
            self.link = link
            self.armed = False
            self.master = {"connected": True, "configured": False, "link": link.description}
        link.start(self._on_frame, self._on_closed)
        self._stop.clear()
        self._keepalive = threading.Thread(target=self._keepalive_loop, name="keepalive", daemon=True)
        self._keepalive.start()
        link.send(p.SER_HELLO_REQ)
        self._configure()
        self.events.publish("link", state="connected", link=link.description)

    def disconnect(self) -> None:
        link = self.link
        if not link:
            return
        try:
            link.send(p.SER_STOP_ALL)
        except Exception:
            pass
        self._stop.set()
        if self._keepalive:
            self._keepalive.join(timeout=1)
        link.close()
        with self._lock:
            self.link = None
            self.armed = False
            self.master = {"connected": False, "configured": False}
            self._fail_pending("link disconnected")
        self.events.publish("link", state="disconnected")

    def _configure(self) -> None:
        payload = p.pack_config(self.fleet.channel, self.keys.pmk, self.keys.peers(self.fleet))
        self.link.send(p.SER_CONFIG, payload)

    def _on_closed(self, reason: str) -> None:
        with self._lock:
            self.link = None
            self.armed = False
            self.master = {"connected": False, "configured": False, "error": reason}
            self._fail_pending(reason)
        self.events.publish("link", state="lost", reason=reason)

    def _send_keepalive(self) -> bool:
        link = self.link
        if not link:
            return False
        flags = (p.HB_ESTOP if self.estop else 0) | (p.HB_ARMED if self.armed and not self.estop else 0)
        try:
            link.send(p.SER_KEEPALIVE, p.SER_KEEPALIVE_S.pack(flags))
        except Exception as exc:
            self._on_closed(f"keepalive failed: {exc}")
            return False
        return True

    def _keepalive_loop(self) -> None:
        while not self._stop.wait(p.HEARTBEAT_PERIOD_MS / 1000):
            if not self._send_keepalive():
                return

    # ------------------------------------------------------------ safety state
    def arm(self) -> None:
        with self._lock:
            if self.estop:
                raise FleetError("Clear the e-stop before arming")
            if not self.master.get("configured"):
                raise FleetError("Master is not connected and configured")
            self.armed = True
        # Push the new state now; the master relays it to robots at once, so a
        # move sent right after arming is not dropped as "disarmed".
        self._send_keepalive()
        time.sleep(0.05)
        self.events.publish("safety", state="armed")

    def disarm(self, reason: str = "operator") -> None:
        with self._lock:
            was = self.armed
            self.armed = False
        self._send_keepalive()
        if self.link:
            self._send_robot(p.BROADCAST, p.MSG_STOP, p.STOP.pack(0))
        if was:
            self.events.publish("safety", state="disarmed", reason=reason)

    def emergency_stop(self, reason: str = "operator") -> None:
        with self._lock:
            self.estop = True
            self.armed = False
            link = self.link
        if link:
            try:
                link.send(p.SER_STOP_ALL)
            except Exception:
                pass
        self.events.publish("safety", state="estop", reason=reason)

    def clear_estop(self) -> None:
        with self._lock:
            self.estop = False
            self.armed = False
        self._send_keepalive()
        self.events.publish("safety", state="estop_cleared")

    # ------------------------------------------------------------ incoming
    def _on_frame(self, frame_type: int, payload: bytes) -> None:
        try:
            if frame_type == p.SER_RECV:
                robot_id, rssi = p.SER_RECV_HEADER.unpack_from(payload)
                self._on_robot(robot_id, rssi, payload[p.SER_RECV_HEADER.size:])
            elif frame_type == p.SER_HELLO:
                _, major, minor, mac, max_payload = p.SER_HELLO_S.unpack(payload)
                with self._lock:
                    self.master.update(mac=p.format_mac(mac), firmware=f"{major}.{minor}",
                                       max_payload=max_payload)
                self.events.publish("master", state="hello", mac=p.format_mac(mac),
                                    firmware=f"{major}.{minor}")
            elif frame_type == p.SER_CONFIG_ACK:
                ok, count = p.SER_CONFIG_ACK_S.unpack(payload)
                with self._lock:
                    self.master["configured"] = bool(ok)
                self.events.publish("master", state="configured" if ok else "config_failed", peers=count)
            elif frame_type == p.SER_STATUS:
                status = p.MasterStatus.unpack(payload)
                with self._lock:
                    self.master.update(uptime_ms=status.uptime_ms, channel=status.channel,
                                       broadcast_armed=bool(status.flags & p.HB_ARMED),
                                       broadcast_estop=bool(status.flags & p.HB_ESTOP),
                                       host_ok=bool(status.flags & p.HB_HOST_OK))
                    for robot_id, radio in status.peers.items():
                        if robot_id in self.robots:
                            self.robots[robot_id].radio = radio
                    if not self.master.get("configured") and self.link:
                        self._configure()  # the master rebooted and forgot its peers
            elif frame_type == p.SER_LOG:
                self.events.publish("master", state="log", text=payload.decode("utf-8", "replace"))
        except Exception as exc:  # a malformed frame must never kill the reader thread
            self.events.publish("error", source="master", error=f"bad frame 0x{frame_type:02x}: {exc}")

    def _on_robot(self, robot_id: int, rssi: int, data: bytes) -> None:
        header, body = p.unpack_message(data)
        robot = self.robots.get(robot_id)
        if robot is None or header.robot_id != robot_id:
            return
        robot.rssi = rssi
        if header.type == p.MSG_TELEMETRY:
            telemetry = p.Telemetry.unpack(body)
            robot.telemetry = telemetry.as_dict()
            robot.telemetry_at = time.monotonic()
            self.events.publish("telemetry", robot_id=robot_id, telemetry=robot.telemetry, rssi=rssi)
        elif header.type == p.MSG_FRAME_META:
            meta = p.FrameMeta.unpack(body)
            with self._lock:
                pending = self._captures.get(meta.req_id)
                if pending:
                    pending.meta = meta
                    pending.last_rx = time.monotonic()
                    self._check_capture(pending)
        elif header.type == p.MSG_FRAME_CHUNK:
            req_id, index = p.FRAME_CHUNK.unpack_from(body)
            with self._lock:
                pending = self._captures.get(req_id)
                if pending:
                    pending.chunks[index] = body[p.FRAME_CHUNK.size:]
                    pending.last_rx = time.monotonic()
                    self._check_capture(pending)
        elif header.type == p.MSG_MOVE_RESULT:
            result = p.MoveResult.unpack(body)
            with self._lock:
                pending = self._moves.get(result.req_id)
                if pending:
                    pending.result = result
                    pending.done.set()
        elif header.type == p.MSG_REJECT:
            req_id, ref, code = p.REJECT.unpack(body)
            reason = p.REJECTS.get(code, f"code {code}")
            with self._lock:
                target = self._moves.get(req_id) if ref == p.MSG_MOVE else self._captures.get(req_id)
                if target:
                    target.error = f"robot rejected: {reason}"
                    target.done.set()
            self.events.publish("robot_reject", robot_id=robot_id, request=ref, reason=reason)

    def _check_capture(self, pending: _PendingCapture) -> None:
        meta = pending.meta
        if meta and len(pending.chunks) >= meta.chunk_count:
            pending.done.set()

    def _fail_pending(self, reason: str) -> None:
        for pending in [*self._captures.values(), *self._moves.values()]:
            pending.error = reason
            pending.done.set()

    # ------------------------------------------------------------ outgoing
    def _send_robot(self, robot_id: int, msg_type: int, body: bytes) -> None:
        link = self.link
        if not link:
            raise FleetError("Master is not connected")
        # Robots check the id in the header, so a "broadcast" is one message per robot.
        targets = list(self.robots) if robot_id == p.BROADCAST else [robot_id]
        for target in targets:
            link.send(p.SER_SEND, bytes([target]) + p.pack_message(msg_type, target, next(self._seq), body))

    def _robot(self, robot_id: int) -> RobotState:
        robot = self.robots.get(robot_id)
        if robot is None:
            raise FleetError(f"Unknown robot {robot_id}")
        return robot

    def capture(self, robot_id: int, framesize: str = "320x240", quality: int = 12,
                timeout: float = CAPTURE_TIMEOUT_S) -> Capture:
        robot = self._robot(robot_id)
        if framesize not in p.FRAMESIZES:
            raise FleetError(f"Unsupported frame size {framesize}")
        if not self.master.get("configured"):
            raise FleetError("Master is not connected and configured")
        req_id = next(self._req_ids) & 0xFFFFFFFF
        pending = _PendingCapture(req_id, robot_id)
        with self._lock:
            self._captures[req_id] = pending
        started = time.monotonic()
        try:
            self._send_robot(robot_id, p.MSG_CAPTURE, p.CAPTURE.pack(req_id, p.FRAMESIZES[framesize], quality))
            resends = 0
            while not pending.done.wait(0.1):
                now = time.monotonic()
                if now - started > timeout:
                    raise FleetError(f"Robot {robot_id} capture timed out")
                with self._lock:
                    meta = pending.meta
                    missing = ([i for i in range(meta.chunk_count) if i not in pending.chunks]
                               if meta else [])
                if meta and missing and now - pending.last_rx > RESEND_AFTER_S and resends < 4:
                    resends += 1
                    pending.last_rx = now
                    batch = missing[:64]
                    self._send_robot(robot_id, p.MSG_RESEND,
                                     p.RESEND.pack(req_id, len(batch)) + p.struct.pack(f"<{len(batch)}H", *batch))
            if pending.error:
                raise FleetError(pending.error)
            meta = pending.meta
            jpeg = b"".join(pending.chunks[i] for i in range(meta.chunk_count))
            if len(jpeg) != meta.jpeg_len or not (jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")):
                raise FleetError(f"Robot {robot_id} sent an incomplete JPEG")
        finally:
            with self._lock:
                self._captures.pop(req_id, None)

        frame_id = f"bot{robot_id}_{req_id:08x}"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        path = self.frames_dir / f"{frame_id}.jpg"
        path.write_bytes(jpeg)
        capture = Capture(robot_id, frame_id, path, jpeg, meta, time.time(),
                          Pose(**vars(robot.pose)), self.simulated)
        robot.last_capture = capture
        self.events.publish("capture", robot_id=robot_id, frame_id=frame_id,
                            seconds=round(time.monotonic() - started, 3),
                            range_mm=capture.as_dict()["range_mm"], simulated=self.simulated)
        return capture

    def move(self, robot_id: int, command: p.MoveCommand, source: str = "manual",
             reason: str = "", tick: int | None = None) -> dict:
        robot = self._robot(robot_id)
        refusal = self._refuse_move(robot, source)
        cal = self.fleet.calibration
        cap = cal.max_speed
        command = p.MoveCommand(command.kind, max(-cap, min(cap, command.left)),
                                max(-cap, min(cap, command.right)), command.duration_ms,
                                command.turn_deg, command.min_clear_mm or cal.min_clear_mm)
        try:
            command.validate()
        except ValueError as exc:
            refusal = refusal or str(exc)
        if refusal:
            self.events.publish("move_refused", robot_id=robot_id, source=source, tick=tick,
                                command=command.describe(), reason=refusal)
            raise FleetError(refusal)

        req_id = next(self._req_ids) & 0xFFFFFFFF
        pending = _PendingMove(req_id, robot_id, command)
        with self._lock:
            self._moves[req_id] = pending
            robot.moving_req = req_id
        self.events.publish("move_sent", robot_id=robot_id, req_id=req_id, source=source, tick=tick,
                            command=command.describe(), reason=reason)
        try:
            self._send_robot(robot_id, p.MSG_MOVE, command.pack(req_id))
            if not pending.done.wait(command.duration_ms / 1000 + 2.0):
                self._send_robot(robot_id, p.MSG_STOP, p.STOP.pack(req_id))
                raise FleetError(f"Robot {robot_id} did not report the move result in time")
            if pending.error:
                raise FleetError(pending.error)
        except FleetError as exc:
            robot.last_move = {"req_id": req_id, "command": command.describe(), "outcome": "failed",
                               "error": str(exc), "source": source}
            self.events.publish("move_failed", robot_id=robot_id, req_id=req_id, tick=tick, error=str(exc))
            raise
        finally:
            with self._lock:
                self._moves.pop(req_id, None)
                robot.moving_req = None

        result = pending.result
        before = Pose(**vars(robot.pose))
        self._update_pose(robot, command, result)
        outcome = {**result.as_dict(), "command": command.describe(), "source": source, "tick": tick,
                   "pose_before": before.as_dict(), "pose_after": robot.pose.as_dict()}
        robot.last_move = outcome
        self.events.publish("move_result", robot_id=robot_id, **outcome)
        return outcome

    def _refuse_move(self, robot: RobotState, source: str) -> str | None:
        if self.estop:
            return "E-stop is active"
        if not self.armed:
            return "Fleet is not armed"
        if not self.master.get("configured"):
            return "Master is not connected"
        if not robot.online():
            return f"{robot.name} has no recent telemetry"
        if robot.moving_req is not None:
            return f"{robot.name} is already moving"
        return self.mission_gate(robot.id, source)

    def stop_robot(self, robot_id: int) -> None:
        self._robot(robot_id)
        self._send_robot(robot_id, p.MSG_STOP, p.STOP.pack(0))
        self.events.publish("stop", robot_id=robot_id)

    def _update_pose(self, robot: RobotState, command: p.MoveCommand, result: p.MoveResult) -> None:
        """Dead reckoning: gyro yaw for heading, commanded speed × time for distance."""
        pose = robot.pose
        imu_ok = bool(robot.telemetry and robot.telemetry.get("imu_ok"))
        yaw = result.yaw_deg
        if not imu_ok:
            yaw = command.turn_deg if command.kind == p.MOVE_TURN and result.outcome == p.OUT_COMPLETED else 0.0
        mean_speed = (result.left + result.right) / 2 / 1000
        distance = mean_speed * self.fleet.calibration.forward_mps_at_full_speed * result.elapsed_ms / 1000
        mid_heading = math.radians(pose.heading_deg + yaw / 2)
        pose.x += distance * math.cos(mid_heading)
        pose.y += distance * math.sin(mid_heading)
        pose.heading_deg = (pose.heading_deg + yaw + 180) % 360 - 180
        pose.sigma_m += 0.15 * abs(distance) + (0.005 if distance else 0)
        pose.sigma_deg += 0.05 * abs(yaw) + (1.0 if yaw else 0)

    def reset_poses(self) -> None:
        for config in self.fleet.robots:
            self.robots[config.id].pose = Pose(*config.start_pose)
        self.events.publish("poses_reset")

    # ------------------------------------------------------------ snapshot
    def snapshot(self) -> dict:
        with self._lock:
            return {
                "master": dict(self.master), "armed": self.armed, "estop": self.estop,
                "simulated": self.simulated,
                "calibration": vars(self.fleet.calibration),
                "robots": [robot.as_dict() for robot in self.robots.values()],
            }

    def close(self) -> None:
        self.disconnect()
