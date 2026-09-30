"""Operator scan tasks: every robot photographs each position around a circle and logs it.

An instruction such as "8 rotations looking for a dog" is interpreted into a
plan the operator can edit: 8 photo positions around one full circle. Running
it sends the plan to every online robot at once. At every position a robot
captures a frame over ESP-NOW, asks the visual model whether the target is in
view, logs the result, then turns 360/steps degrees through the same fleet
move path (and firmware limits) as everything else. Every position is always
visited; finding the target or a person is logged, never a reason to stop.
Only an operator stop, an e-stop, or a failed turn ends a robot's scan.

Logs: runs/tasks/<id>/events.jsonl (everything) and robot-<id>.csv (one clean
row per position per robot).
"""

from __future__ import annotations

import csv
import json
import threading
import time
from datetime import datetime, timezone

from haggatrons import protocol as p
from haggatrons.config import RUNS
from haggatrons.events import EventBus
from haggatrons.fleet import FleetController, FleetError

SPIN_CHUNK_MS = 450  # under the firmware's 500 ms cap for drives without a range reading

CSV_FIELDS = ["position", "planned_heading_deg", "gyro_turn_deg", "target", "target_visible", "where",
              "confidence", "description", "scene", "person_visible", "frame", "vision_seconds", "turn_outcome"]


class TaskError(RuntimeError):
    pass


def validate_plan(plan: dict) -> dict:
    steps = int(plan.get("steps_per_rotation", 8))
    rotations = int(plan.get("rotations", 1))
    target = str(plan.get("target", "")).strip()
    if not 2 <= steps <= 36:
        raise TaskError("Steps per rotation must be 2-36")
    if not 1 <= rotations <= 3:
        raise TaskError("Rotations must be 1-3")
    if not 0 < len(target) <= 100:
        raise TaskError("Say what to look for (1-100 characters)")
    return {"steps_per_rotation": steps, "rotations": rotations, "target": target,
            "understood_as": str(plan.get("understood_as", ""))[:500]}


class ScanTask:
    def __init__(self, fleet: FleetController, events: EventBus, mission_active) -> None:
        self.fleet = fleet
        self.events = events
        self.mission_active = mission_active
        self.state = "idle"
        self.id: str | None = None
        self.plan: dict | None = None
        self.instruction = ""
        self.robots: dict[int, dict] = {}
        self.log_dir = None
        self._lock = threading.Lock()

    @property
    def running(self) -> bool:
        return self.state in ("running", "stopping")

    def start(self, plan: dict, instruction: str = "") -> dict:
        from haggatrons.observer import load_api_key

        plan = validate_plan(plan)
        with self._lock:
            if self.running:
                raise TaskError("A task is already running")
            if self.mission_active():
                raise TaskError("Stop the mission before running a task")
            if self.fleet.estop or not self.fleet.armed:
                raise TaskError("Arm the fleet (and clear any e-stop) before running a task")
            if not load_api_key():
                raise TaskError("No OpenAI key: put OPENAI_API_KEY=... in the project .env")
            online = [r.id for r in self.fleet.robots.values() if r.online()]
            if not online:
                raise TaskError("No robot is online")
            self.id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            self.plan, self.instruction, self.state = plan, instruction[:1000], "running"
            self.robots = {rid: {"state": "running", "step": 0, "found": []} for rid in online}
        self.log_dir = log_dir = RUNS / "tasks" / self.id
        log_dir.mkdir(parents=True, exist_ok=True)
        (log_dir / "task.json").write_text(json.dumps({"id": self.id, "instruction": self.instruction,
                                                       "plan": plan, "robots": online}, indent=2) + "\n")
        self.events.log_to(log_dir / "events.jsonl")
        offline = [r.id for r in self.fleet.robots.values() if r.id not in online]
        self.events.publish("task", state="running", task_id=self.id, plan=plan, instruction=self.instruction,
                            robots=online, skipped_offline=offline)
        threads = [threading.Thread(target=self._run_robot, args=(rid,), name=f"task-bot{rid}", daemon=True)
                   for rid in online]
        for thread in threads:
            thread.start()
        threading.Thread(target=self._join, args=(threads,), name="task", daemon=True).start()
        return self.snapshot()

    def stop(self, reason: str = "operator") -> None:
        with self._lock:
            if self.state != "running":
                return
            self.state = "stopping"
        for robot_id in self.robots:
            try:
                self.fleet.stop_robot(robot_id)
            except FleetError:
                pass
        self.events.publish("task", state="stopping", task_id=self.id, reason=reason)

    def _join(self, threads: list[threading.Thread]) -> None:
        for thread in threads:
            thread.join()
        with self._lock:
            final = "stopped" if self.state == "stopping" else "finished"
            self.state = final
        found = {rid: [f["position"] for f in r["found"]] for rid, r in self.robots.items()}
        self.events.publish("task", state=final, task_id=self.id, found=found, log_dir=str(self.log_dir),
                            robots={rid: r["state"] for rid, r in self.robots.items()})
        self.events.log_to(None)

    def _done(self, robot_id: int, outcome: str, **data) -> None:
        self.robots[robot_id]["state"] = outcome
        self.events.publish("task_robot", robot_id=robot_id, task_id=self.id, outcome=outcome, **data)

    def _run_robot(self, robot_id: int) -> None:
        from haggatrons.observer import MODEL, detect_jpeg

        plan = self.plan
        steps = plan["steps_per_rotation"] * plan["rotations"]
        angle = 360.0 / plan["steps_per_rotation"]
        target = plan["target"]
        cal = self.fleet.fleet.calibration
        turned = 0.0
        rows = self.log_dir / f"robot-{robot_id}.csv"
        with rows.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for step in range(steps):
                if self.state != "running":
                    return self._done(robot_id, "stopped", step=step)
                self.robots[robot_id]["step"] = step + 1
                planned = round((step * angle) % 360, 1)
                try:
                    capture = self.fleet.capture(robot_id, "640x480")
                except FleetError as exc:
                    return self._done(robot_id, "failed", step=step, error=f"photo at position {step + 1}: {exc}")
                started = time.monotonic()
                try:
                    detection, usage = detect_jpeg(capture.jpeg, target)
                except Exception as exc:
                    return self._done(robot_id, "failed", step=step, error=f"vision at position {step + 1}: {str(exc)[:300]}")
                seconds = round(time.monotonic() - started, 2)
                self.events.publish("task_step", robot_id=robot_id, task_id=self.id, step=step, of=steps,
                                    planned_heading_deg=planned, heading_turned_deg=round(turned, 1),
                                    frame_id=capture.frame_id, model=MODEL, target=target, detection=detection,
                                    usage=usage, seconds=seconds, simulated=capture.simulated)
                if detection["target_visible"]:
                    self.robots[robot_id]["found"].append({
                        "position": step + 1, "heading_deg": planned, "frame_id": capture.frame_id,
                        "where": detection["target_location"], "confidence": detection["confidence"]})
                turn_outcome = "last position"
                if step < steps - 1:
                    turn_outcome, spun = self._spin(robot_id, angle, step, steps, cal)
                    turned += spun
                writer.writerow({
                    "position": step + 1, "planned_heading_deg": planned, "gyro_turn_deg": round(turned, 1),
                    "target": target, "target_visible": detection["target_visible"],
                    "where": detection["target_location"], "confidence": detection["confidence"],
                    "description": detection["target_description"], "scene": detection["scene_summary"],
                    "person_visible": detection["person_visible"], "frame": str(capture.path),
                    "vision_seconds": seconds, "turn_outcome": turn_outcome})
                stream.flush()
                if turn_outcome not in ("completed", "last position"):
                    if turn_outcome == "stopped":
                        return self._done(robot_id, "stopped", step=step)
                    return self._done(robot_id, "failed", step=step,
                                      error=f"turn after position {step + 1}: {turn_outcome}")
        found = self.robots[robot_id]["found"]
        self._done(robot_id, "completed", step=steps - 1, turned_deg=round(turned, 1),
                   found_at=[f["position"] for f in found], log=str(rows))

    def _spin(self, robot_id: int, angle: float, step: int, steps: int, cal) -> tuple[str, float]:
        """Timed spin in place: left wheel back, right wheel forward, for angle / spin rate."""
        total_ms = int(angle / max(1.0, cal.spin_deg_per_s) * 1000)
        chunks = max(1, -(-total_ms // SPIN_CHUNK_MS))
        chunk_ms = max(50, total_ms // chunks)
        yaw = 0.0
        for index in range(chunks):
            if self.state != "running":
                return "stopped", yaw
            try:
                result = self.fleet.move(
                    robot_id, p.MoveCommand(p.MOVE_DRIVE, -cal.turn_speed, cal.turn_speed, chunk_ms),
                    source="task", tick=step,
                    reason=f"scan position {step + 1}/{steps}: spin ~{angle:.0f}° ({index + 1}/{chunks})")
            except FleetError as exc:
                return f"failed: {exc}", yaw
            yaw += result["yaw_deg"]  # informational only
            if result["outcome"] != "completed":
                return result["outcome"], yaw
        self.events.publish("task_spin", robot_id=robot_id, task_id=self.id, step=step, target_deg=round(angle, 1),
                            spin_ms=chunk_ms * chunks, moves=chunks, gyro_deg=round(yaw, 1))
        return "completed", yaw

    def snapshot(self) -> dict:
        return {"state": self.state, "id": self.id, "plan": self.plan, "instruction": self.instruction,
                "robots": self.robots}
