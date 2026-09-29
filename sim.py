"""Small, deterministic two-robot exploration harness with replaceable boundaries."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, dataclass
import json
from pathlib import Path


Point = tuple[int, int]
UNKNOWN, FREE, WALL = "?", ".", "#"
WORLD = (
    "####################",
    "#..................#",
    "#....##............#",
    "#....##....###.....#",
    "#..........#.......#",
    "#..####....#.......#",
    "#..........#.......#",
    "#..........#.......#",
    "#.....###..........#",
    "#..................#",
    "####################",
)
STARTS = ((2, 2), (17, 8))
RANGE = 3


def neighbors(point: Point) -> tuple[Point, ...]:
    x, y = point
    return ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1))


def line(a: Point, b: Point) -> list[Point]:
    """Integer grid cells crossed by a camera ray, including both endpoints."""
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    cells = []
    while True:
        cells.append((x0, y0))
        if (x0, y0) == (x1, y1):
            return cells
        twice = 2 * err
        if twice > -dy:
            err -= dy
            x0 += sx
        if twice < dx:
            err += dx
            y0 += sy


@dataclass(frozen=True)
class Observation:
    robot_id: int
    tick: int
    pose: Point
    cells: dict[Point, str]


@dataclass(frozen=True)
class Assignment:
    robot_id: int
    target: Point | None


@dataclass(frozen=True)
class Intent:
    robot_id: int
    next_cell: Point


class World:
    def __init__(self) -> None:
        self.rows = WORLD
        self.width = len(WORLD[0])
        self.height = len(WORLD)
        assert all(len(row) == self.width for row in WORLD)
        assert all(self.cell(p) == FREE for p in STARTS)

    def cell(self, point: Point) -> str:
        x, y = point
        if not (0 <= x < self.width and 0 <= y < self.height):
            return WALL
        return self.rows[y][x]

    def observe(self, robot_id: int, tick: int, pose: Point) -> Observation:
        visible: dict[Point, str] = {}
        for y in range(max(0, pose[1] - RANGE), min(self.height, pose[1] + RANGE + 1)):
            for x in range(max(0, pose[0] - RANGE), min(self.width, pose[0] + RANGE + 1)):
                target = (x, y)
                if (x - pose[0]) ** 2 + (y - pose[1]) ** 2 > RANGE**2:
                    continue
                for point in line(pose, target):
                    visible[point] = self.cell(point)
                    if self.cell(point) == WALL:
                        break
        return Observation(robot_id, tick, pose, visible)


class Coordinator:
    def __init__(self) -> None:
        self.known: dict[Point, str] = {}

    def merge(self, observations: list[Observation]) -> None:
        for observation in observations:
            self.known.update(observation.cells)

    def route(self, start: Point, target: Point) -> list[Point] | None:
        if self.known.get(start) != FREE or self.known.get(target) != FREE:
            return None
        queue = deque([start])
        previous: dict[Point, Point | None] = {start: None}
        while queue:
            current = queue.popleft()
            if current == target:
                path = [current]
                while previous[current] is not None:
                    current = previous[current]  # type: ignore[assignment]
                    path.append(current)
                return list(reversed(path))
            for point in neighbors(current):
                if self.known.get(point) == FREE and point not in previous:
                    previous[point] = current
                    queue.append(point)
        return None

    def assign(self, poses: dict[int, Point]) -> list[Assignment]:
        frontiers = [
            point for point, kind in self.known.items()
            if kind == FREE and any(neighbor not in self.known for neighbor in neighbors(point))
        ]
        choices: list[tuple[float, int, Point]] = []
        for robot_id, pose in poses.items():
            for target in frontiers:
                path = self.route(pose, target)
                if path is None:
                    continue
                unknown_edges = sum(point not in self.known for point in neighbors(target))
                score = 2.0 * unknown_edges - 0.5 * (len(path) - 1)
                choices.append((score, robot_id, target))
        choices.sort(key=lambda choice: (-choice[0], choice[1], choice[2]))
        assigned: dict[int, Point] = {}
        for _, robot_id, target in choices:
            if robot_id in assigned:
                continue
            # Keep the robots from exploring the same small patch.
            if any(abs(target[0] - other[0]) + abs(target[1] - other[1]) <= RANGE
                   for other in assigned.values()):
                continue
            assigned[robot_id] = target
        return [Assignment(robot_id, assigned.get(robot_id)) for robot_id in poses]


class RobotWorker:
    def __init__(self, robot_id: int) -> None:
        self.robot_id = robot_id

    def propose(self, pose: Point, assignment: Assignment, map_state: Coordinator) -> Intent:
        if assignment.target is None:
            return Intent(self.robot_id, pose)
        route = map_state.route(pose, assignment.target)
        return Intent(self.robot_id, route[1] if route and len(route) > 1 else pose)


class SafetyGate:
    def __init__(self) -> None:
        self.enabled = True
        self.blocked = 0

    def apply(self, world: World, poses: dict[int, Point], intents: list[Intent]) -> dict[int, Point]:
        proposed = {intent.robot_id: intent.next_cell for intent in intents}
        if set(proposed) != set(poses):
            raise ValueError("Every robot must submit exactly one intent")
        if not self.enabled:
            return dict(poses)
        accepted = dict(proposed)
        for robot_id, target in proposed.items():
            current = poses[robot_id]
            if target == current:
                continue
            if target not in neighbors(current) or world.cell(target) != FREE:
                accepted[robot_id] = current
        # Resolve simultaneous commands until no robot moves into a cell whose
        # occupant was forced to stop. This also forbids position swaps.
        while True:
            rejected: set[int] = set()
            for robot_id, target in accepted.items():
                if target == poses[robot_id]:
                    continue
                for other_id, other_target in accepted.items():
                    if other_id == robot_id:
                        continue
                    if target == other_target:
                        rejected.add(robot_id)
                    if target == poses[other_id] and other_target == poses[robot_id]:
                        rejected.update((robot_id, other_id))
            if not rejected:
                break
            for robot_id in rejected:
                accepted[robot_id] = poses[robot_id]
        self.blocked += sum(proposed[robot_id] != poses[robot_id]
                            and accepted[robot_id] == poses[robot_id] for robot_id in poses)
        assert len(set(accepted.values())) == len(accepted)
        return accepted


def render(known: dict[Point, str], poses: dict[int, Point], world: World) -> str:
    symbols = {pose: str(robot_id) for robot_id, pose in poses.items()}
    return "\n".join(
        "".join(symbols.get((x, y), known.get((x, y), UNKNOWN)) for x in range(world.width))
        for y in range(world.height)
    )


def run(max_steps: int) -> tuple[dict, list[dict]]:
    world = World()
    coordinator = Coordinator()
    workers = {robot_id: RobotWorker(robot_id) for robot_id in range(2)}
    safety = SafetyGate()
    poses = dict(enumerate(STARTS))
    distance = {robot_id: 0 for robot_id in poses}
    seen_by: dict[Point, set[int]] = {}
    trace: list[dict] = []
    total_free = sum(row.count(FREE) for row in WORLD)
    for tick in range(max_steps + 1):
        observations = [world.observe(robot_id, tick, pose) for robot_id, pose in poses.items()]
        for observation in observations:
            for point, kind in observation.cells.items():
                if kind == FREE:
                    seen_by.setdefault(point, set()).add(observation.robot_id)
        coordinator.merge(observations)
        covered = sum(kind == FREE for kind in coordinator.known.values())
        if tick == max_steps or covered == total_free:
            break
        assignments = coordinator.assign(poses)
        intents = [workers[item.robot_id].propose(poses[item.robot_id], item, coordinator)
                   for item in assignments]
        next_poses = safety.apply(world, poses, intents)
        for robot_id in poses:
            distance[robot_id] += int(poses[robot_id] != next_poses[robot_id])
        trace.append({
            "tick": tick,
            "poses": {str(k): list(v) for k, v in poses.items()},
            "assignments": [asdict(item) for item in assignments],
            "intents": [asdict(item) for item in intents],
            "coverage": covered / total_free,
        })
        if next_poses == poses and all(item.target is None for item in assignments):
            break
        poses = next_poses
    summary = {
        "ticks": tick,
        "covered_free_cells": covered,
        "total_free_cells": total_free,
        "coverage": round(covered / total_free, 3),
        "distance_by_robot": distance,
        "overlap_cells": sum(len(robots) > 1 for robots in seen_by.values()),
        "blocked_moves": safety.blocked,
        "map": render(coordinator.known, poses, world),
    }
    return summary, trace


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=150)
    parser.add_argument("--trace", type=Path)
    args = parser.parse_args()
    if args.steps < 0:
        parser.error("--steps must be nonnegative")
    summary, trace = run(args.steps)
    if args.trace:
        args.trace.write_text(json.dumps({"summary": summary, "trace": trace}, indent=2))
    print(json.dumps({key: value for key, value in summary.items() if key != "map"}, indent=2))
    print(summary["map"])


if __name__ == "__main__":
    main()
