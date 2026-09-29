"""Shared exploration map, distinct goal assignment, and the coordinator's move gate.

The map is an evidence grid in metres with the robots' shared start frame as
origin. Each cell accumulates log-odds from two sources of different quality:

- the REV 2m range sensor: a measured ray, moderate weight;
- the visual model's left/center/right judgement: a short, weak ray.

One image alone can never push a cell past the "free" threshold; it needs the
range sensor or repeated agreement. Dead-reckoned pose uncertainty is carried
alongside, and the coordinator re-observes after every short move.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

CELL_M = 0.1
L_MAX = 4.0
FREE_T = -0.6
BLOCKED_T = 0.6
RANGE_MAX_M = 2.0
RANGE_TRUST_M = 1.5
VISION_RAY_M = 0.5
VISION_ANGLES = {"left": 35.0, "center": 0.0, "right": -35.0}
GOAL_SEPARATION_M = 0.6
ROBOT_SEPARATION_M = 0.35
MIN_GOAL_M = 0.3          # nearer frontiers are revealed by scanning in place
BEAM_HALF_ANGLE = 8.0     # the VL53L0X cone is ~25°; free space is marked across it
UNKNOWN_RADIUS = 2        # cells around a frontier counted as unexplored area
KEEP_GOAL_BONUS = 3.0     # hysteresis: robots stick with a goal unless a clearly better one appears

Cell = tuple[int, int]


def to_cell(x: float, y: float) -> Cell:
    return (math.floor(x / CELL_M), math.floor(y / CELL_M))


def cell_center(cell: Cell) -> tuple[float, float]:
    return ((cell[0] + 0.5) * CELL_M, (cell[1] + 0.5) * CELL_M)


def ray_cells(x: float, y: float, angle_deg: float, length: float) -> list[Cell]:
    steps = max(1, int(length / (CELL_M / 2)))
    angle = math.radians(angle_deg)
    cells: list[Cell] = []
    for i in range(steps + 1):
        d = length * i / steps
        cell = to_cell(x + d * math.cos(angle), y + d * math.sin(angle))
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return cells


@dataclass
class Report:
    """One robot's local observation for a tick (what a worker sends the coordinator)."""

    robot_id: int
    tick: int
    x: float
    y: float
    heading_deg: float
    sigma_m: float
    range_mm: int | None
    directions: dict[str, str] | None = None
    hazards: list[str] = field(default_factory=list)
    summary: str = ""
    confidence: float = 0.5
    online: bool = True


@dataclass
class Goal:
    robot_id: int
    kind: str  # "frontier" | "scan" | "hold"
    target: tuple[float, float] | None
    reason: str
    score: float = 0.0

    def as_dict(self) -> dict:
        return {"robot_id": self.robot_id, "kind": self.kind,
                "target": None if self.target is None else [round(v, 2) for v in self.target],
                "reason": self.reason, "score": round(self.score, 2)}


@dataclass
class Proposal:
    robot_id: int
    action: str  # "turn" | "forward" | "hold"
    turn_deg: float = 0.0
    distance_m: float = 0.0
    reason: str = ""

    def as_dict(self) -> dict:
        return {"robot_id": self.robot_id, "action": self.action, "turn_deg": round(self.turn_deg, 1),
                "distance_m": round(self.distance_m, 3), "reason": self.reason}


class ExplorationMap:
    def __init__(self) -> None:
        self.logodds: dict[Cell, float] = {}
        self.visits: dict[Cell, set[int]] = {}

    # -------------------------------------------------------------- updates
    def _add(self, cell: Cell, delta: float) -> None:
        self.logodds[cell] = max(-L_MAX, min(L_MAX, self.logodds.get(cell, 0.0) + delta))

    def state(self, cell: Cell) -> str:
        value = self.logodds.get(cell)
        if value is None or FREE_T < value < BLOCKED_T:
            return "unknown"
        return "free" if value <= FREE_T else "blocked"

    def integrate(self, report: Report) -> None:
        if not report.online:
            return
        here = to_cell(report.x, report.y)
        self._add(here, -2.0)  # the robot is standing on it
        self.visits.setdefault(here, set()).add(report.robot_id)
        # Evidence weakens as dead reckoning drifts.
        trust = max(0.25, 1.0 - report.sigma_m * 2)
        if report.range_mm is not None:
            measured = report.range_mm / 1000
            length = min(measured, RANGE_TRUST_M)
            hit = measured < RANGE_MAX_M - 0.05 and measured <= RANGE_TRUST_M
            center = ray_cells(report.x, report.y, report.heading_deg, length)
            end = center[-1] if hit else None
            free: set[Cell] = set()
            for offset in (-BEAM_HALF_ANGLE, 0.0, BEAM_HALF_ANGLE):
                # Off-axis rays stop a little short: the cone may clip an edge the center misses.
                ray = ray_cells(report.x, report.y, report.heading_deg + offset,
                                length if offset == 0 else max(0.0, length - CELL_M))
                free.update(ray[:-1] if hit and offset == 0 else ray)
            free.discard(end)
            for cell in free:
                self._add(cell, -0.7 * trust)
            if end is not None:
                self._add(end, 1.2 * trust)
        if report.directions:
            for name, offset in VISION_ANGLES.items():
                verdict = report.directions.get(name)
                cells = ray_cells(report.x, report.y, report.heading_deg + offset, VISION_RAY_M)[1:]
                if verdict == "open":
                    for cell in cells:
                        self._add(cell, -0.25 * trust * report.confidence)
                elif verdict == "blocked" and cells:
                    self._add(cells[-1], 0.5 * trust * report.confidence)

    # -------------------------------------------------------------- queries
    def neighbors(self, cell: Cell) -> list[Cell]:
        x, y = cell
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def frontiers(self) -> list[Cell]:
        return [cell for cell in self.logodds if self.state(cell) == "free"
                and any(self.state(n) == "unknown" for n in self.neighbors(cell))]

    def path(self, start: Cell, goal: Cell) -> list[Cell] | None:
        if start == goal:
            return [start]
        previous: dict[Cell, Cell | None] = {start: None}
        queue = deque([start])
        while queue:
            current = queue.popleft()
            for n in self.neighbors(current):
                if n in previous or self.state(n) != "free":
                    continue
                previous[n] = current
                if n == goal:
                    route = [n]
                    while previous[route[-1]] is not None:
                        route.append(previous[route[-1]])
                    return route[::-1]
                queue.append(n)
        return None

    def unknown_around(self, cell: Cell, radius: int = UNKNOWN_RADIUS) -> int:
        x, y = cell
        return sum(self.state((x + dx, y + dy)) == "unknown"
                   for dx in range(-radius, radius + 1) for dy in range(-radius, radius + 1))

    def coverage(self) -> dict:
        free = sum(self.state(c) == "free" for c in self.logodds)
        blocked = sum(self.state(c) == "blocked" for c in self.logodds)
        return {"free_cells": free, "blocked_cells": blocked, "area_m2": round(free * CELL_M ** 2, 2)}

    def snapshot(self) -> dict:
        cells = [[c[0], c[1], round(v, 2)] for c, v in self.logodds.items() if abs(v) >= 0.2]
        return {"cell_m": CELL_M, "free_t": FREE_T, "blocked_t": BLOCKED_T, "cells": cells,
                "frontiers": [list(c) for c in self.frontiers()], **self.coverage()}


def assign_goals(world: ExplorationMap, reports: list[Report],
                 previous: dict[int, tuple[float, float]] | None = None) -> list[Goal]:
    """Give each online robot a distinct frontier; otherwise ask it to scan."""
    previous = previous or {}
    choices: list[tuple[float, int, Cell, float, int]] = []
    frontiers = world.frontiers()
    for report in reports:
        if not report.online:
            continue
        start = to_cell(report.x, report.y)
        for target in frontiers:
            if math.dist(cell_center(target), (report.x, report.y)) < MIN_GOAL_M:
                continue
            route = world.path(start, target)
            if route is None:
                continue
            unknown = world.unknown_around(target)
            distance = (len(route) - 1) * CELL_M
            visited_by_other = any(r != report.robot_id for r in world.visits.get(target, ()))
            score = 0.5 * unknown - 2.5 * distance - (1.5 if visited_by_other else 0.0)
            kept = previous.get(report.robot_id)
            if kept and math.dist(cell_center(target), kept) < CELL_M * 1.5:
                score += KEEP_GOAL_BONUS
            choices.append((score, report.robot_id, target, distance, unknown))
    choices.sort(key=lambda c: (-c[0], c[1], c[2]))
    goals: dict[int, Goal] = {}
    for score, robot_id, target, distance, unknown in choices:
        if robot_id in goals:
            continue
        center = cell_center(target)
        if any(g.target and math.dist(center, g.target) < GOAL_SEPARATION_M for g in goals.values()):
            continue
        goals[robot_id] = Goal(robot_id, "frontier", center,
                               f"frontier bordering {unknown} unknown cells, {distance:.1f} m away, "
                               f"≥{GOAL_SEPARATION_M} m from other goals", score)
    result = []
    for report in reports:
        if not report.online:
            result.append(Goal(report.robot_id, "hold", None, "robot offline"))
        elif report.robot_id in goals:
            result.append(goals[report.robot_id])
        else:
            result.append(Goal(report.robot_id, "scan", None,
                               "no distinct reachable frontier; rotate to observe"))
    return result


def propose(report: Report, goal: Goal, clearance_m: float, step_m: float = 0.3) -> Proposal:
    """What a robot worker would do next to pursue its goal (runs on the worker)."""
    rid = report.robot_id
    if goal.kind == "hold" or not report.online:
        return Proposal(rid, "hold", reason=goal.reason)
    if goal.kind == "scan" or goal.target is None:
        return Proposal(rid, "turn", turn_deg=60.0, reason="scan: rotate 60° to look for open space")
    dx, dy = goal.target[0] - report.x, goal.target[1] - report.y
    if math.hypot(dx, dy) < 0.15:
        return Proposal(rid, "turn", turn_deg=60.0, reason="at goal: rotate 60° to observe around it")
    bearing = math.degrees(math.atan2(dy, dx)) - report.heading_deg
    bearing = (bearing + 180) % 360 - 180
    if abs(bearing) > 15:
        turn = max(-90.0, min(90.0, bearing))
        return Proposal(rid, "turn", turn_deg=turn, reason=f"face goal ({bearing:+.0f}° off)")
    if report.range_mm is None:
        return Proposal(rid, "turn", turn_deg=45.0, reason="range sensor unavailable; forward needs it")
    if report.directions and report.directions.get("center") == "blocked":
        side = "left" if report.directions.get("left") == "open" else "right"
        return Proposal(rid, "turn", turn_deg=45.0 if side == "left" else -45.0,
                        reason=f"vision says center blocked; turning {side}")
    free_m = report.range_mm / 1000 - clearance_m
    distance = min(step_m, math.hypot(dx, dy), free_m)
    if distance < 0.05:
        return Proposal(rid, "turn", turn_deg=60.0, reason=f"only {max(free_m, 0):.2f} m clear ahead")
    return Proposal(rid, "forward", distance_m=distance,
                    reason=f"advance {distance:.2f} m toward goal; {report.range_mm} mm clear")


def gate(proposals: list[Proposal], reports: list[Report], clearance_m: float) -> list[tuple[Proposal, str | None]]:
    """Coordinator-side approval. Returns (proposal, refusal reason or None)."""
    by_id = {r.robot_id: r for r in reports}
    predicted: dict[int, tuple[float, float]] = {r.robot_id: (r.x, r.y) for r in reports}
    decisions: list[tuple[Proposal, str | None]] = []
    # Forward moves are approved in robot-id order; later ones must avoid earlier ones.
    for proposal in sorted(proposals, key=lambda item: item.robot_id):
        report = by_id.get(proposal.robot_id)
        refusal = None
        if report is None or not report.online:
            refusal = "robot offline"
        elif proposal.action == "forward":
            if report.range_mm is None:
                refusal = "no range reading"
            elif report.range_mm / 1000 - proposal.distance_m < clearance_m:
                refusal = "would end inside the clearance margin"
            else:
                h = math.radians(report.heading_deg)
                end = (report.x + proposal.distance_m * math.cos(h), report.y + proposal.distance_m * math.sin(h))
                for other_id, other in predicted.items():
                    if other_id != proposal.robot_id and math.dist(end, other) < ROBOT_SEPARATION_M:
                        refusal = f"would come within {ROBOT_SEPARATION_M} m of robot {other_id}"
                        break
                if refusal is None:
                    predicted[proposal.robot_id] = end
        elif proposal.action == "turn" and abs(proposal.turn_deg) > 180:
            refusal = "turn larger than 180°"
        decisions.append((proposal, refusal))
    return decisions
