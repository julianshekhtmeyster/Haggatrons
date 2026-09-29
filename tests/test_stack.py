"""Full Mac-side stack against the simulated master and robots (no hardware, no API key)."""

import time

import pytest

from haggatrons import protocol as p
from haggatrons.client import BackendClient, BackendError
from haggatrons.config import FleetKeys, load_fleet
from haggatrons.events import EventBus
from haggatrons.fleet import FleetController, FleetError
from haggatrons.server import ApiServer, Backend
from haggatrons.simlink import SimLink


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def stack(tmp_path, monkeypatch):
    monkeypatch.setattr("haggatrons.mission.RUNS", tmp_path)
    monkeypatch.setattr("haggatrons.server.BACKEND_FILE", tmp_path / "backend.json")
    fleet = load_fleet()
    keys = FleetKeys(bytes(range(16)), {r.id: bytes([r.id] * 16) for r in fleet.robots})
    events = EventBus()
    controller = FleetController(fleet, keys, events, frames_dir=tmp_path / "frames")
    links = []

    def make_link(drop=0.0):
        link = SimLink(fleet, chunk_drop=drop)
        links.append(link)
        return link

    backend = Backend(controller, events, "test-token", make_link)
    server = ApiServer(backend)
    server.start()
    api = BackendClient(server.url, "test-token")
    yield api, controller, backend, make_link, links
    server.close()


def connect(api, controller):
    api.post("/api/link/connect", {"kind": "sim"})
    assert wait_for(lambda: all(r.online() for r in controller.robots.values()))


def test_api_requires_token(stack):
    api, *_ = stack
    with pytest.raises(BackendError) as info:
        BackendClient(api.url, "wrong").get("/api/state")
    assert info.value.status == 401


def test_capture_reassembles_jpeg(stack):
    api, controller, *_ = stack
    connect(api, controller)
    data = api.post("/api/robots/1/capture", {"framesize": "640x480"})
    assert data["width"] == 640 and data["range_mm"] > 0
    jpeg = api.get(f"/api/frames/{data['frame_id']}.jpg", raw=True)
    assert jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")


def test_capture_recovers_dropped_chunks(stack):
    api, controller, backend, make_link, _ = stack
    controller.connect(make_link(drop=0.3))
    assert wait_for(lambda: all(r.online() for r in controller.robots.values()))
    capture = controller.capture(1, "1024x768", quality=8)
    assert capture.meta.chunk_count > 3
    assert len(capture.jpeg) == capture.meta.jpeg_len


def test_moves_refused_until_armed(stack):
    api, controller, *_ = stack
    connect(api, controller)
    with pytest.raises(BackendError, match="not armed"):
        api.post("/api/robots/1/move", {"kind": "turn", "left": 400, "turn_deg": 30, "duration_ms": 1500})
    api.post("/api/arm")
    result = api.post("/api/robots/1/move", {"kind": "turn", "left": 400, "turn_deg": 30, "duration_ms": 1500})
    assert result["outcome"] == "completed"
    assert 25 <= result["yaw_deg"] <= 35
    assert 20 <= result["pose_after"]["heading_deg"] <= 40


def test_estop_interrupts_move_and_blocks_arming(stack):
    api, controller, *_ = stack
    connect(api, controller)
    api.post("/api/arm")
    import threading

    results = {}
    thread = threading.Thread(target=lambda: results.setdefault("r", controller.move(
        2, p.MoveCommand(p.MOVE_DRIVE, 300, -300, 2000))))
    thread.start()
    time.sleep(0.3)
    api.post("/api/estop")
    thread.join(5)
    assert results["r"]["outcome"] == "stopped_estop"
    assert results["r"]["elapsed_ms"] < 1000
    with pytest.raises(BackendError, match="e-stop"):
        api.post("/api/arm")
    api.post("/api/estop/clear")
    api.post("/api/arm")


def test_robot_stops_when_host_goes_silent(stack):
    api, controller, *_ = stack
    connect(api, controller)
    api.post("/api/arm")
    import threading

    results = {}
    thread = threading.Thread(target=lambda: results.setdefault("r", controller.move(
        1, p.MoveCommand(p.MOVE_DRIVE, -300, 300, 2000))))
    thread.start()
    time.sleep(0.3)
    controller._stop.set()  # simulate the Mac hanging: keepalives stop
    thread.join(5)
    # The master drops to e-stop after HOST_TIMEOUT_MS; the robot then stops locally.
    assert results["r"]["outcome"] in ("stopped_estop", "stopped_link_lost")
    assert results["r"]["elapsed_ms"] < 1500


def test_forward_is_refused_near_a_wall(stack):
    api, controller, backend, _, links = stack
    connect(api, controller)
    api.post("/api/arm")
    api.post("/api/robots/1/move", {"kind": "turn", "left": 450, "turn_deg": 180, "duration_ms": 2000})
    # Robot 1 now faces the west wall ~0.35 m away; a 1.5 s forward must stop at the clearance.
    result = api.post("/api/robots/1/move", {"kind": "forward", "left": 450, "right": 450, "duration_ms": 1500,
                                             "min_clear_mm": 180})
    assert result["outcome"] == "stopped_obstacle"
    assert links[0].world.clearance(links[0].robots[1].x, links[0].robots[1].y) > 0.1


def test_local_mission_explores_and_logs(stack):
    api, controller, backend, *_ = stack
    connect(api, controller)
    with pytest.raises(BackendError, match="Arm"):
        api.post("/api/mission/start", {"steps": 3, "runner": "local"})
    api.post("/api/arm")
    api.post("/api/mission/start", {"steps": 6, "runner": "local"})
    with pytest.raises(BackendError, match="manual"):
        api.post("/api/robots/1/move", {"kind": "turn", "left": 400, "turn_deg": 30, "duration_ms": 1500})
    assert wait_for(lambda: backend.mission.state in ("finished", "failed"), timeout=90)
    mission = backend.mission.snapshot()
    assert mission["state"] == "finished", backend.events.recent(20)
    assert mission["summary"]["free_cells"] > 20
    assert not controller.armed  # missions end disarmed
    kinds = {e["kind"] for e in backend.events.recent(400)}
    assert {"observation", "coordinator", "gate", "move_result"} <= kinds
    log = (backend.mission.log_dir / "events.jsonl").read_text()
    assert "move_result" in log and "telemetry" not in log


def test_pause_blocks_mission_moves(stack):
    api, controller, backend, *_ = stack
    connect(api, controller)
    api.post("/api/arm")
    api.post("/api/mission/start", {"steps": 20, "runner": "local"})
    time.sleep(1.5)
    api.post("/api/mission/pause")
    time.sleep(0.5)
    count = sum(e["kind"] == "move_result" for e in backend.events.recent(400))
    time.sleep(2.0)
    assert sum(e["kind"] == "move_result" for e in backend.events.recent(400)) == count
    api.post("/api/mission/stop")
    assert wait_for(lambda: backend.mission.state in ("stopped", "finished"), timeout=10)


def test_move_refused_for_offline_robot(stack):
    _, controller, *_ = stack
    controller.master["configured"] = True
    controller.armed = True
    with pytest.raises(FleetError, match="no recent telemetry"):
        controller.move(1, p.MoveCommand(p.MOVE_TURN, 400, 400, 1000, 30))


def test_telemetry_does_not_evict_event_history():
    from haggatrons.events import EventBus

    bus = EventBus(history=10)
    bus.publish("link", state="lost", reason="unplugged")
    for _ in range(100):
        bus.publish("telemetry", robot_id=1)
    assert [e["kind"] for e in bus.recent()] == ["link"]
