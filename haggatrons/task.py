"""Operator scan tasks: every robot looks, checks for a target, turns, and reports each step.

An instruction such as "turn a full circle in 8 steps and look for a dog" is
interpreted into a plan the operator can edit. Running it sends the plan to
every online robot at once. Each step captures a frame over ESP-NOW, asks the
visual model whether the target is in view, logs the result, then turns
360/steps degrees through the same fleet move path (and firmware limits) as
everything else. A person in view stops that robot unless a person is the
target. Each task writes runs/tasks/<id>/events.jsonl.
"""

from __future__ import annotations

import json
import threading
import time
from datetime import datetime, timezone

from haggatrons import protocol as p
from haggatrons.config import RUNS
from haggatrons.events import EventBus
from haggatrons.fleet import FleetController, FleetError

PERSON_WORDS = {"person", "people", "human", "humans", "man", "woman", "child", "kid", "someone", "face"}


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
            "stop_when_found": bool(plan.get("stop_when_found", True)),
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
            self.robots = {rid: {"state": "running", "step": 0, "found": None} for rid in online}
        log_dir = RUNS / "tasks" / self.id
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
        found = {rid: r["found"] for rid, r in self.robots.items() if r["found"]}
        self.events.publish("task", state=final, task_id=self.id, found=found,
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
        looking_for_person = any(word in PERSON_WORDS for word in target.lower().replace(",", " ").split())
        cal = self.fleet.fleet.calibration
        turned = 0.0
        for step in range(steps):
            if self.state != "running":
                return self._done(robot_id, "stopped", step=step)
            self.robots[robot_id]["step"] = step + 1
            try:
                capture = self.fleet.capture(robot_id, "640x480")
            except FleetError as exc:
                return self._done(robot_id, "failed", step=step, error=f"capture: {exc}")
            started = time.monotonic()
            try:
                detection, usage = detect_jpeg(capture.jpeg, target)
            except Exception as exc:
                return self._done(robot_id, "failed", step=step, error=f"vision: {str(exc)[:300]}")
            found = detection["target_visible"]
            self.events.publish("task_step", robot_id=robot_id, task_id=self.id, step=step, of=steps,
                                heading_turned_deg=round(turned, 1), frame_id=capture.frame_id, model=MODEL,
                                target=target, detection=detection, usage=usage,
                                seconds=round(time.monotonic() - started, 2), simulated=capture.simulated)
            if found:
                self.robots[robot_id]["found"] = {"step": step, "frame_id": capture.frame_id,
                                                  "where": detection["target_location"],
                                                  "confidence": detection["confidence"],
                                                  "description": detection["target_description"]}
                if plan["stop_when_found"]:
                    return self._done(robot_id, "found", step=step, frame_id=capture.frame_id,
                                      where=detection["target_location"], confidence=detection["confidence"])
            if detection["person_visible"] and not looking_for_person:
                return self._done(robot_id, "stopped_person", step=step, frame_id=capture.frame_id,
                                  reason="a person is in view; stopped turning for safety")
            if step == steps - 1:
                break
            if self.state != "running":
                return self._done(robot_id, "stopped", step=step)
            timeout = int(min(p.MAX_MOVE_MS, 600 + angle / 90 * 1200))
            try:
                result = self.fleet.move(robot_id, p.MoveCommand(p.MOVE_TURN, cal.turn_speed, cal.turn_speed,
                                                                 timeout, angle),
                                         source="task", reason=f"scan step {step + 1}/{steps}: turn {angle:.0f}°",
                                         tick=step)
            except FleetError as exc:
                return self._done(robot_id, "failed", step=step, error=f"turn: {exc}")
            if result["outcome"] not in ("completed",):
                return self._done(robot_id, "failed", step=step, error=f"turn {result['outcome']}")
            turned += result["yaw_deg"]
        found = self.robots[robot_id]["found"]
        self._done(robot_id, "found" if found else "not_found", step=steps - 1, turned_deg=round(turned, 1))

    def snapshot(self) -> dict:
        return {"state": self.state, "id": self.id, "plan": self.plan, "instruction": self.instruction,
                "robots": self.robots}
