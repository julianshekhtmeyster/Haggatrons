"""Local control API (127.0.0.1 only, bearer token) plus a server-sent event stream."""

from __future__ import annotations

import hmac
import json
import queue
import re
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable
from urllib.parse import parse_qs, urlparse

from haggatrons import protocol as p
from haggatrons.client import BACKEND_FILE
from haggatrons.config import ROOT, write_private
from haggatrons.events import EventBus
from haggatrons.fleet import FleetController, FleetError
from haggatrons.link import SerialLink, list_serial_ports
from haggatrons.mission import Mission, MissionError


STATIC_DIR = ROOT / "app" / "dist"
STATIC_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript", ".css": "text/css",
                ".svg": "image/svg+xml", ".png": "image/png", ".ico": "image/x-icon", ".json": "application/json"}
UI_GRACE_S = 3.0  # disarm when no control page has been connected for this long


class ApiError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


class Backend:
    """Everything the HTTP handlers operate on."""

    def __init__(self, fleet: FleetController, events: EventBus, token: str,
                 make_sim_link: Callable[[], object]) -> None:
        self.fleet = fleet
        self.events = events
        self.token = token
        self.make_sim_link = make_sim_link
        self.mission: Mission | None = None  # set once the server knows its URL
        self.coordinator: dict | None = None
        self.ui_required = False  # set by `serve`; headless missions and tests have no page
        self.ui_streams = 0
        self.ui_last_seen = time.monotonic()

    # ------------------------------------------------------------ routes
    def route(self, method: str, path: str, body: dict) -> object:
        fleet, mission = self.fleet, self.mission
        if method == "GET" and path == "/api/state":
            return {"fleet": fleet.snapshot(), "mission": mission.snapshot(),
                    "coordinator": self.coordinator, "events": self.events.recent(150)}
        if method == "GET" and path == "/api/ports":
            return {"ports": list_serial_ports()}
        if method == "GET" and path == "/api/mission/control":
            return mission.control()
        if method == "POST" and path == "/api/link/connect":
            return self._connect(body)
        if method == "POST" and path == "/api/link/disconnect":
            mission.stop("link disconnected")
            fleet.disconnect()
            return fleet.snapshot()
        if method == "POST" and path == "/api/arm":
            fleet.arm()
            return fleet.snapshot()
        if method == "POST" and path == "/api/disarm":
            fleet.disarm()
            return fleet.snapshot()
        if method == "POST" and path == "/api/estop":
            fleet.emergency_stop(body.get("reason", "operator"))
            mission.stop("e-stop")
            return fleet.snapshot()
        if method == "POST" and path == "/api/estop/clear":
            fleet.clear_estop()
            return fleet.snapshot()
        if method == "POST" and path == "/api/poses/reset":
            if mission.state in ("running", "paused"):
                raise ApiError(409, "Cannot reset poses during a mission")
            fleet.reset_poses()
            self.coordinator = None
            return fleet.snapshot()
        if method == "POST" and path == "/api/mission/start":
            return mission.start(int(body.get("steps", 12)), bool(body.get("vision", False)),
                                 int(body.get("vision_budget", 4)), str(body.get("runner", "flower")))
        if method == "POST" and path == "/api/mission/pause":
            mission.pause()
            return mission.snapshot()
        if method == "POST" and path == "/api/mission/resume":
            mission.resume()
            return mission.snapshot()
        if method == "POST" and path == "/api/mission/stop":
            mission.stop()
            return mission.snapshot()
        if method == "POST" and path == "/api/vision/claim":
            return {"granted": mission.claim_vision(), "used": mission.vision_used,
                    "budget": mission.vision_budget}
        if method == "POST" and path == "/api/coordinator":
            self.coordinator = body
            mission.coordinator = {k: body.get(k) for k in ("tick", "phase", "goals", "summary")}
            self.events.publish("coordinator", tick=body.get("tick"), phase=body.get("phase"),
                                goals=body.get("goals"), coverage={k: body.get("map", {}).get(k) for k in
                                                                   ("free_cells", "blocked_cells", "area_m2")})
            return {"ok": True}
        if method == "POST" and path == "/api/log":
            kind = str(body.pop("kind", "log"))[:40]
            self.events.publish(kind, **body)
            return {"ok": True}

        match = re.fullmatch(r"/api/frames/([A-Za-z0-9_]+)\.jpg", path)
        if method == "GET" and match:
            frame = self._frame(match.group(1))
            if frame is None:
                raise ApiError(404, "Unknown frame")
            return frame
        match = re.fullmatch(r"/api/robots/(\d+)/(capture|observe|analyze|move|act|stop)", path)
        if method == "POST" and match:
            return self._robot_action(int(match.group(1)), match.group(2), body)
        raise ApiError(404, f"No route for {method} {path}")

    def _connect(self, body: dict) -> dict:
        kind = body.get("kind", "serial")
        if self.mission.state in ("running", "paused"):
            raise ApiError(409, "Stop the mission before changing the link")
        if kind == "sim":
            self.fleet.connect(self.make_sim_link())
        elif kind == "serial":
            port = str(body.get("port", ""))
            if not port:
                raise ApiError(400, "Choose the master's serial port")
            try:
                link = SerialLink(port)
                self.fleet.connect(link)
            except Exception as exc:
                raise ApiError(409, f"Cannot open {port}: {exc}") from None
        else:
            raise ApiError(400, "kind must be serial or sim")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not self.fleet.master.get("configured"):
            time.sleep(0.05)
        return self.fleet.snapshot()

    def _frame(self, frame_id: str) -> bytes | None:
        for robot in self.fleet.robots.values():
            capture = robot.last_capture
            if capture and capture.frame_id == frame_id:
                return capture.jpeg
        path = self.fleet.frames_dir / f"{frame_id}.jpg"
        return path.read_bytes() if path.exists() else None

    def _robot_action(self, robot_id: int, action: str, body: dict) -> dict:
        fleet = self.fleet
        if action == "stop":
            fleet.stop_robot(robot_id)
            return {"ok": True}
        if action == "analyze":
            return self._analyze(robot_id, body)
        if action in ("capture", "observe"):
            if action == "capture" and self.mission.state in ("running", "paused"):
                raise ApiError(409, "Manual captures are disabled while a mission is active")
            capture = fleet.capture(robot_id, str(body.get("framesize", "320x240")),
                                    int(body.get("quality", 12)))
            robot = fleet.robots[robot_id]
            data = capture.as_dict()
            data.update(online=robot.online(), telemetry=robot.telemetry, tick=body.get("tick"))
            if action == "observe":
                self.events.publish("observation", robot_id=robot_id, tick=body.get("tick"),
                                    frame_id=capture.frame_id, range_mm=data["range_mm"],
                                    pose=data["pose"], simulated=capture.simulated,
                                    imu_valid=capture.meta.imu_valid)
            return data
        if action == "move":
            kind = {"drive": p.MOVE_DRIVE, "turn": p.MOVE_TURN, "forward": p.MOVE_FORWARD}.get(body.get("kind"))
            if kind is None:
                raise ApiError(400, "kind must be drive, turn, or forward")
            command = p.MoveCommand(kind, int(body.get("left", 0)), int(body.get("right", 0)),
                                    int(body.get("duration_ms", 400)), float(body.get("turn_deg", 0)),
                                    int(body.get("min_clear_mm", 0)))
            return fleet.move(robot_id, command, source="manual", reason=str(body.get("reason", "operator jog")))
        # act: a coordinator-approved proposal from a mission worker
        command = self._proposal_to_command(body)
        outcome = fleet.move(robot_id, command, source="mission", reason=str(body.get("reason", "")),
                             tick=body.get("tick"))
        return {"robot_id": robot_id, "tick": body.get("tick"), "proposal": body, **outcome}

    def _analyze(self, robot_id: int, body: dict) -> dict:
        """Operator-requested look: one fresh frame through the visual model. Never moves."""
        from haggatrons.loop import VISION_GOAL
        from haggatrons.observer import MODEL, load_api_key, observe_jpeg

        if self.mission.state in ("running", "paused"):
            raise ApiError(409, "The mission is using the cameras; analyze from the trace instead")
        key = load_api_key()
        if not key:
            raise ApiError(409, "No OpenAI key: put OPENAI_API_KEY=... in the project .env, then retry")
        capture = self.fleet.capture(robot_id, str(body.get("framesize", "640x480")), 12)
        goal = str(body.get("goal") or VISION_GOAL)[:500]
        started = time.monotonic()
        try:
            decision, usage = observe_jpeg(capture.jpeg, goal, key)
        except Exception as exc:
            message = str(exc).replace(key, "[REDACTED]")[:500]
            self.events.publish("vision_failed", robot_id=robot_id, frame_id=capture.frame_id, message=message,
                                source="manual")
            raise ApiError(502, f"Vision model failed: {message}") from None
        event = self.events.publish("vision", robot_id=robot_id, frame_id=capture.frame_id, model=MODEL,
                                    decision=decision, usage=usage, goal=goal, source="manual",
                                    seconds=round(time.monotonic() - started, 2), motor_action_executed=False)
        return event

    def _proposal_to_command(self, proposal: dict) -> p.MoveCommand:
        cal = self.fleet.fleet.calibration
        if proposal.get("action") == "turn":
            degrees = max(-180.0, min(180.0, float(proposal["turn_deg"])))
            timeout = int(min(p.MAX_MOVE_MS, 600 + abs(degrees) / 90 * 1200))
            return p.MoveCommand(p.MOVE_TURN, cal.turn_speed, cal.turn_speed, timeout, degrees)
        if proposal.get("action") == "forward":
            mps = cal.forward_mps_at_full_speed * cal.cruise_speed / 1000
            duration = int(max(100, min(1500, float(proposal["distance_m"]) / mps * 1000)))
            return p.MoveCommand(p.MOVE_FORWARD, cal.cruise_speed, cal.cruise_speed, duration,
                                 min_clear_mm=cal.min_clear_mm)
        raise ApiError(400, "action must be turn or forward")


def make_handler(backend: Backend):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "Haggatrons/1.0"

        def log_message(self, *args) -> None:  # keep stdout for the READY line and errors
            pass

        def _authorized(self, query: dict) -> bool:
            supplied = self.headers.get("Authorization", "").removeprefix("Bearer ").strip()
            supplied = supplied or (query.get("token") or [""])[0]
            return hmac.compare_digest(supplied.encode(), backend.token.encode())

        def _static(self, path: str) -> None:
            # The control page itself is public; everything under /api needs the token.
            relative = "index.html" if path in ("", "/") else path.lstrip("/")
            target = (STATIC_DIR / relative).resolve()
            if not target.is_relative_to(STATIC_DIR.resolve()) or not target.is_file():
                if not (STATIC_DIR / "index.html").exists():
                    return self._send(404, {"error": "UI not built: run `npm install && npm run build` in app/"})
                target = STATIC_DIR / "index.html"
            body = target.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", STATIC_TYPES.get(target.suffix, "application/octet-stream"))
            self.send_header("Content-Length", str(len(body)))
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def _send(self, status: int, payload: object) -> None:
            if isinstance(payload, bytes):
                body, content_type = payload, "image/jpeg"
            else:
                body, content_type = json.dumps(payload, default=str).encode(), "application/json"
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _handle(self, method: str) -> None:
            url = urlparse(self.path)
            query = parse_qs(url.query)
            if method == "GET" and not url.path.startswith("/api/"):
                return self._static(url.path)
            if not self._authorized(query):
                return self._send(401, {"error": "unauthorized"})
            if method == "GET" and url.path == "/api/events":
                return self._stream()
            body: dict = {}
            if method == "POST":
                length = int(self.headers.get("Content-Length", "0") or 0)
                if length > 1_000_000:
                    return self._send(413, {"error": "request too large"})
                try:
                    body = json.loads(self.rfile.read(length) or b"{}")
                except ValueError:
                    return self._send(400, {"error": "invalid JSON"})
                if not isinstance(body, dict):
                    return self._send(400, {"error": "JSON body must be an object"})
            try:
                self._send(200, backend.route(method, url.path, body))
            except ApiError as exc:
                self._send(exc.status, {"error": str(exc)})
            except (FleetError, MissionError) as exc:
                self._send(409, {"error": str(exc)})
            except (KeyError, ValueError, TypeError) as exc:
                self._send(400, {"error": f"bad request: {exc}"})
            except Exception as exc:
                backend.events.publish("error", source="api", error=f"{type(exc).__name__}: {exc}")
                self._send(500, {"error": f"{type(exc).__name__}: {exc}"})

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            subscriber = backend.events.subscribe()
            backend.ui_streams += 1
            try:
                while True:
                    try:
                        event = subscriber.get(timeout=10)
                        data = json.dumps(event, default=str)
                        self.wfile.write(f"data: {data}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                backend.ui_streams -= 1
                backend.ui_last_seen = time.monotonic()
                backend.events.unsubscribe(subscriber)
                self.close_connection = True

        def do_GET(self) -> None:
            self._handle("GET")

        def do_POST(self) -> None:
            self._handle("POST")

    return Handler


class ApiServer:
    def __init__(self, backend: Backend, host: str = "127.0.0.1", port: int = 0) -> None:
        if host not in ("127.0.0.1", "localhost", "::1"):
            raise ValueError("The control API only binds to localhost")
        self.backend = backend
        self.httpd = ThreadingHTTPServer((host, port), make_handler(backend))
        self.httpd.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        backend.mission = Mission(backend.fleet, backend.events, self.url, backend.token)
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self.httpd.serve_forever, name="api", daemon=True)
        self._thread.start()
        threading.Thread(target=self._ui_watchdog, name="ui-watchdog", daemon=True).start()
        write_private(BACKEND_FILE, json.dumps({"url": self.url, "token": self.backend.token}))

    def _ui_watchdog(self) -> None:
        """A closed browser tab must not leave robots armed with nobody watching."""
        backend = self.backend
        while True:
            time.sleep(0.5)
            if backend.ui_streams > 0:
                backend.ui_last_seen = time.monotonic()
            elif backend.ui_required and time.monotonic() - backend.ui_last_seen > UI_GRACE_S:
                if backend.mission and backend.mission.state in ("running", "paused"):
                    backend.mission.stop("control page closed")
                if backend.fleet.armed:
                    backend.fleet.disarm(reason="control page closed")

    def close(self) -> None:
        if self.backend.mission:
            self.backend.mission.stop("backend shutting down")
        self.backend.fleet.close()
        self.httpd.shutdown()
        self.httpd.server_close()
        try:
            if json.loads(BACKEND_FILE.read_text())["url"] == self.url:
                BACKEND_FILE.unlink()
        except (OSError, ValueError, KeyError):
            pass
