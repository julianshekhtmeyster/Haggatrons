"""One exploration tick, split the way Flower carries it.

Worker side (one per robot, a Flower ClientApp in a mission):
    observe()  capture a frame + IMU + range, optionally ask the visual model
    decide()   turn the coordinator's goal into one bounded proposal
    execute()  ask the backend to run an approved move and report the outcome

Coordinator side (the Flower ServerApp):
    Coordinator.merge()   fold reports into the shared map
    Coordinator.assign()  distinct goals, with reasons
    Coordinator.approve() gate proposals against clearance and each other

All robot access goes through the backend API, which applies the fleet's
safety state (armed, e-stop, pause) to every move.
"""

from __future__ import annotations

import time

from haggatrons.client import BackendClient, BackendError
from haggatrons.worldmap import ExplorationMap, Goal, Proposal, Report, assign_goals, gate, propose

VISION_GOAL = "Explore this room: judge whether left, center, and right look open for a small floor robot."


# ------------------------------------------------------------------ worker
def observe(api: BackendClient, robot_id: int, tick: int, use_vision: bool) -> dict:
    """Returns a Report as a dict (plain values so Flower can carry it)."""
    try:
        obs = api.post(f"/api/robots/{robot_id}/observe", {"tick": tick}, timeout=20)
    except BackendError as exc:
        api.post("/api/log", {"kind": "worker", "robot_id": robot_id, "tick": tick,
                              "message": f"observation failed: {exc}"})
        return {"robot_id": robot_id, "tick": tick, "online": False, "x": 0, "y": 0,
                "heading_deg": 0, "sigma_m": 1, "range_mm": None, "summary": str(exc)}
    pose = obs["pose"]
    report = {
        "robot_id": robot_id, "tick": tick, "online": True, "x": pose["x"], "y": pose["y"],
        "heading_deg": pose["heading_deg"], "sigma_m": pose["sigma_m"], "range_mm": obs["range_mm"],
        "frame_id": obs["frame_id"], "simulated": obs["simulated"], "directions": None,
        "hazards": [], "summary": "", "confidence": 0.5,
    }
    if use_vision and api.post("/api/vision/claim", {"robot_id": robot_id}).get("granted"):
        from haggatrons.observer import MODEL, observe_jpeg

        started = time.monotonic()
        try:
            jpeg = api.get(f"/api/frames/{obs['frame_id']}.jpg", raw=True)
            decision, usage = observe_jpeg(jpeg, VISION_GOAL)
        except Exception as exc:  # the visual model is advisory; carry on without it
            api.post("/api/log", {"kind": "vision_failed", "robot_id": robot_id, "tick": tick,
                                  "message": str(exc)[:500]})
        else:
            report.update(directions=decision["directions"], hazards=decision["visual_hazards"],
                          summary=decision["scene_summary"],
                          confidence=0.3 if decision["uncertainty"] else 0.5)
            api.post("/api/log", {"kind": "vision", "robot_id": robot_id, "tick": tick, "model": MODEL,
                                  "source": "mission", "goal": VISION_GOAL,
                                  "decision": decision, "usage": usage, "frame_id": obs["frame_id"],
                                  "seconds": round(time.monotonic() - started, 2),
                                  "motor_action_executed": False})
    return report


def decide(report: dict, goal: dict, clearance_m: float) -> dict:
    """Pure function of the report and goal, so it runs on the worker without robot I/O."""
    target = goal.get("target")
    proposal = propose(_report(report), Goal(goal["robot_id"], goal["kind"],
                                             tuple(target) if target else None, goal["reason"]),
                       clearance_m)
    return proposal.as_dict()


def execute(api: BackendClient, robot_id: int, approved: dict, tick: int) -> dict:
    if approved["action"] == "hold":
        api.post("/api/log", {"kind": "held", "robot_id": robot_id, "tick": tick, "reason": approved["reason"]})
        return {"robot_id": robot_id, "outcome": "held", "reason": approved["reason"]}
    try:
        return api.post(f"/api/robots/{robot_id}/act", {**approved, "tick": tick}, timeout=10)
    except BackendError as exc:
        return {"robot_id": robot_id, "outcome": "refused", "reason": str(exc)}


def _report(data: dict) -> Report:
    return Report(robot_id=int(data["robot_id"]), tick=int(data["tick"]), x=float(data["x"]),
                  y=float(data["y"]), heading_deg=float(data["heading_deg"]),
                  sigma_m=float(data["sigma_m"]),
                  range_mm=None if data.get("range_mm") is None else int(data["range_mm"]),
                  directions=data.get("directions"), hazards=list(data.get("hazards") or []),
                  summary=str(data.get("summary") or ""), confidence=float(data.get("confidence", 0.5)),
                  online=bool(data.get("online", True)))


# ------------------------------------------------------------------ coordinator
class Coordinator:
    def __init__(self, clearance_m: float) -> None:
        self.map = ExplorationMap()
        self.clearance_m = clearance_m
        self.reports: list[Report] = []
        self.goals: list[Goal] = []

    def merge(self, reports: list[dict]) -> None:
        self.reports = [_report(r) for r in sorted(reports, key=lambda r: r["robot_id"])]
        for report in self.reports:
            self.map.integrate(report)

    def assign(self) -> list[dict]:
        previous = {g.robot_id: g.target for g in self.goals if g.kind == "frontier" and g.target}
        self.goals = assign_goals(self.map, self.reports, previous)
        return [goal.as_dict() for goal in self.goals]

    def approve(self, proposals: list[dict]) -> list[tuple[dict, str | None]]:
        decisions = gate([Proposal(p["robot_id"], p["action"], p.get("turn_deg", 0.0),
                                   p.get("distance_m", 0.0), p.get("reason", "")) for p in proposals],
                         self.reports, self.clearance_m)
        return [(proposal.as_dict(), refusal) for proposal, refusal in decisions]

    def state(self, tick: int) -> dict:
        return {"tick": tick, "map": self.map.snapshot(),
                "goals": [goal.as_dict() for goal in self.goals],
                "reports": [vars(report) for report in self.reports]}


def wait_while_paused(api: BackendClient) -> str:
    """Blocks during a pause. Returns the mission state that ended the wait."""
    while True:
        state = api.get("/api/mission/control")["state"]
        if state != "paused":
            return state
        time.sleep(0.25)


def run_tick(api: BackendClient, coordinator: Coordinator, robot_ids: list[int], tick: int,
             use_vision: bool, observe_fn=None, decide_fn=None, execute_fn=None) -> str:
    """Local (non-Flower) version of one tick, used by tests and the headless runner.

    The Flower ServerApp performs the same steps, sending each worker call as a message.
    """
    observe_fn = observe_fn or (lambda rid: observe(api, rid, tick, use_vision))
    reports = [observe_fn(rid) for rid in robot_ids]
    coordinator.merge(reports)
    goals = coordinator.assign()
    api.post("/api/coordinator", {**coordinator.state(tick), "phase": "assigned"})
    by_id = {r["robot_id"]: r for r in reports}
    decide_fn = decide_fn or (lambda goal: decide(by_id[goal["robot_id"]], goal, coordinator.clearance_m))
    proposals = [decide_fn(goal) for goal in goals]
    decisions = coordinator.approve(proposals)
    api.post("/api/log", {"kind": "gate", "tick": tick, "decisions": [
        {"proposal": prop, "approved": refusal is None, "refusal": refusal} for prop, refusal in decisions]})
    if wait_while_paused(api) != "running":
        return "stopped"
    execute_fn = execute_fn or (lambda approved: execute(api, approved["robot_id"], approved, tick))
    for proposal, refusal in decisions:
        approved = proposal if refusal is None else {**proposal, "action": "hold", "reason": refusal}
        execute_fn(approved)
    return "running"


def headless_mission(api: BackendClient, robot_ids: list[int], steps: int, clearance_m: float,
                     use_vision: bool = False) -> dict:
    coordinator = Coordinator(clearance_m)
    tick = 0
    for tick in range(steps):
        if wait_while_paused(api) != "running":
            break
        if run_tick(api, coordinator, robot_ids, tick, use_vision) != "running":
            break
    summary = {"ticks": tick + 1, **coordinator.map.coverage()}
    api.post("/api/coordinator", {**coordinator.state(tick), "phase": "finished", "summary": summary})
    return summary

