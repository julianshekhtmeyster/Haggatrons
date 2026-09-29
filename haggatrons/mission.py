"""Mission lifecycle under human supervision.

The operator starts, pauses, resumes, and stops a mission. The mission runs
either as a Flower federation (one ClientApp per robot, the ServerApp as
coordinator) or, for tests, as an in-process loop with the same logic. Either
way every move goes back through the backend, where the mission gate refuses
it unless the mission is running.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from haggatrons.config import ROOT, RUNS
from haggatrons.events import EventBus
from haggatrons.fleet import FleetController, FleetError

ACTIVE = {"running", "paused"}


class MissionError(RuntimeError):
    pass


class Mission:
    def __init__(self, fleet: FleetController, events: EventBus, url: str, token: str) -> None:
        self.fleet = fleet
        self.events = events
        self.url = url
        self.token = token
        self.state = "idle"
        self.id: str | None = None
        self.steps = 0
        self.vision = False
        self.vision_budget = 0
        self.vision_used = 0
        self.runner = "flower"
        self.started_at: float | None = None
        self.summary: dict | None = None
        self.coordinator: dict | None = None
        self.log_dir: Path | None = None
        self._process: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        fleet.mission_gate = self._gate

    # ------------------------------------------------------------ policy
    def _gate(self, robot_id: int, source: str) -> str | None:
        if source == "mission":
            return None if self.state == "running" else f"mission is {self.state}"
        if self.state in ACTIVE:
            return "manual moves are disabled while a mission is active"
        return None

    def claim_vision(self) -> bool:
        with self._lock:
            if not self.vision or self.state != "running" or self.vision_used >= self.vision_budget:
                return False
            self.vision_used += 1
            return True

    # ------------------------------------------------------------ lifecycle
    def start(self, steps: int, vision: bool, vision_budget: int, runner: str = "flower") -> dict:
        with self._lock:
            if self.state in ACTIVE:
                raise MissionError("A mission is already active")
            if not 1 <= steps <= 200:
                raise MissionError("steps must be 1-200")
            if runner not in ("flower", "local"):
                raise MissionError("runner must be flower or local")
            if not self.fleet.armed or self.fleet.estop:
                raise MissionError("Arm the fleet (and clear any e-stop) before starting")
            if vision and not 0 < vision_budget <= 50:
                raise MissionError("vision budget must be 1-50 calls")
            self.id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            self.log_dir = RUNS / "missions" / self.id
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.events.log_to(self.log_dir / "events.jsonl")
            self.state, self.steps, self.runner = "running", steps, runner
            self.vision, self.vision_budget, self.vision_used = vision, vision_budget if vision else 0, 0
            self.started_at, self.summary, self.coordinator = time.time(), None, None
        (self.log_dir / "mission.json").write_text(json.dumps({
            "id": self.id, "steps": steps, "vision": vision, "vision_budget": vision_budget,
            "runner": runner, "simulated": self.fleet.simulated,
            "robots": [r.id for r in self.fleet.fleet.robots], "started_utc": self.id,
        }, indent=2) + "\n")
        self.events.publish("mission", state="running", mission_id=self.id, steps=steps, vision=vision,
                            runner=runner, simulated=self.fleet.simulated)
        target = self._run_flower if runner == "flower" else self._run_local
        self._thread = threading.Thread(target=target, name="mission", daemon=True)
        self._thread.start()
        return self.snapshot()

    def pause(self) -> None:
        with self._lock:
            if self.state != "running":
                raise MissionError("Mission is not running")
            self.state = "paused"
        for robot_id in self.fleet.robots:
            try:
                self.fleet.stop_robot(robot_id)
            except FleetError:
                pass
        self.events.publish("mission", state="paused", mission_id=self.id)

    def resume(self) -> None:
        with self._lock:
            if self.state != "paused":
                raise MissionError("Mission is not paused")
            if not self.fleet.armed or self.fleet.estop:
                raise MissionError("Re-arm the fleet before resuming")
            self.state = "running"
        self.events.publish("mission", state="running", mission_id=self.id, resumed=True)

    def stop(self, reason: str = "operator") -> None:
        with self._lock:
            if self.state not in ACTIVE:
                return
            self.state = "stopping"
        self.fleet.disarm(reason=f"mission stopped: {reason}")
        self.events.publish("mission", state="stopping", mission_id=self.id, reason=reason)
        process = self._process
        if process and process.poll() is None:
            threading.Timer(15, lambda: process.poll() is None and process.terminate()).start()

    def _finish(self, state: str, **data) -> None:
        with self._lock:
            if self.state == "stopping" and state == "finished":
                state = "stopped"
            self.state = state
            self._process = None
        self.fleet.disarm(reason=f"mission {state}")
        self.events.publish("mission", state=state, mission_id=self.id, summary=self.summary, **data)
        if self.log_dir:
            (self.log_dir / "summary.json").write_text(json.dumps({
                "state": state, "summary": self.summary, "vision_calls": self.vision_used,
                "coordinator": self.coordinator, **data}, indent=2, default=str) + "\n")
        self.events.log_to(None)

    # ------------------------------------------------------------ runners
    def _run_local(self) -> None:
        from haggatrons.client import BackendClient
        from haggatrons.loop import headless_mission

        try:
            api = BackendClient(self.url, self.token)
            self.summary = headless_mission(api, list(self.fleet.robots), self.steps,
                                            self.fleet.fleet.calibration.min_clear_mm / 1000, self.vision)
        except Exception as exc:
            return self._finish("failed", error=str(exc))
        self._finish("finished")

    def _run_flower(self) -> None:
        executable = shutil.which("flwr", path=str(Path(sys.executable).parent)) or shutil.which("flwr")
        if not executable:
            return self._finish("failed", error="Flower CLI (flwr) is not installed in this environment")
        env = os.environ.copy()
        env.update({
            "HAGGATRONS_URL": self.url, "HAGGATRONS_TOKEN": self.token,
            "FLWR_HOME": str(ROOT / ".flwr"), "FLWR_DISABLE_RUNTIME_DEPENDENCY_INSTALLATION": "1",
            "PYTHONIOENCODING": "utf-8", "PYTHONPATH": str(ROOT),
            "PATH": str(Path(executable).parent) + os.pathsep + env.get("PATH", ""),
        })
        robots = len(self.fleet.robots)
        command = [executable, "run", ".", "--stream",
                   "--federation-config", f"num-supernodes={robots} client-resources-num-cpus=1",
                   "--run-config", f"steps={self.steps} vision={'true' if self.vision else 'false'} "
                                   f"clearance-mm={self.fleet.fleet.calibration.min_clear_mm}"]
        self.events.publish("flower", line=f"$ flwr run . --federation-config num-supernodes={robots}")
        completed = False
        try:
            self._process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                             stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                             errors="replace", bufsize=1)
            for line in self._process.stdout:
                line = line.rstrip()
                if not line:
                    continue
                if line.startswith("HAGGATRONS_SUMMARY "):
                    completed = True
                    self.summary = json.loads(line.split(" ", 1)[1])
                self.events.publish("flower", line=line[:1000])
            code = self._process.wait()
        except Exception as exc:
            return self._finish("failed", error=str(exc))
        # Flower can report a failed run while its CLI still exits zero.
        if code == 0 and completed:
            self._finish("finished")
        elif self.state == "stopping":
            self._finish("stopped")
        else:
            self._finish("failed", error=f"flwr exited with code {code}" + ("" if completed else
                                                                            " before the coordinator finished"))

    # ------------------------------------------------------------ views
    def control(self) -> dict:
        return {"state": self.state, "mission_id": self.id}

    def snapshot(self) -> dict:
        return {"state": self.state, "id": self.id, "steps": self.steps, "runner": self.runner,
                "vision": self.vision, "vision_budget": self.vision_budget, "vision_used": self.vision_used,
                "started_at": self.started_at, "summary": self.summary,
                "log_dir": str(self.log_dir) if self.log_dir else None}
