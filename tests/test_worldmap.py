import math

from haggatrons.worldmap import (
    ExplorationMap, Goal, Proposal, Report, assign_goals, gate, propose, to_cell,
)


def report(robot_id=1, x=0.45, y=0.45, heading=0.0, range_mm=1200, directions=None, sigma=0.02, online=True):
    return Report(robot_id, 0, x, y, heading, sigma, range_mm, directions, online=online)


def test_range_ray_marks_free_cells_and_a_hit():
    world = ExplorationMap()
    for _ in range(2):
        world.integrate(report(range_mm=800))
    assert world.state(to_cell(0.85, 0.45)) == "free"
    assert world.state(to_cell(1.25, 0.45)) == "blocked"
    assert world.state(to_cell(0.45, 1.5)) == "unknown"


def test_single_image_cannot_mark_cells_free():
    world = ExplorationMap()
    world.integrate(report(range_mm=None, directions={"left": "open", "center": "open", "right": "open"}))
    free = [cell for cell in world.logodds if world.state(cell) == "free"]
    assert free == [to_cell(0.45, 0.45)]  # only the cell the robot stands on


def test_goals_are_distinct_and_separated():
    world = ExplorationMap()
    robots = [report(1, 0.45, 0.45, 0, 1500), report(2, 0.45, 1.05, 0, 1500)]
    for _ in range(2):
        for r in robots:
            world.integrate(r)
    goals = assign_goals(world, robots)
    targets = [g.target for g in goals if g.kind == "frontier"]
    assert len(targets) == 2
    assert math.dist(*targets) >= 0.6


def test_offline_robot_holds():
    world = ExplorationMap()
    goals = assign_goals(world, [report(1, online=False)])
    assert goals[0].kind == "hold"


def test_propose_turns_toward_goal_then_advances():
    r = report(heading=90)
    assert propose(r, Goal(1, "frontier", (1.5, 0.45), ""), 0.18).action == "turn"
    step = propose(report(heading=0, range_mm=1000), Goal(1, "frontier", (1.5, 0.45), ""), 0.18)
    assert step.action == "forward" and 0.05 <= step.distance_m <= 0.3


def test_propose_needs_range_for_forward():
    proposal = propose(report(range_mm=None), Goal(1, "frontier", (1.5, 0.45), ""), 0.18)
    assert proposal.action == "turn"


def test_gate_blocks_collisions_and_clearance():
    # Facing each other 0.8 m apart: the first 0.3 m advance is fine, the second would close to 0.2 m.
    reports = [report(1, 0.45, 0.45, 0, 1000), report(2, 1.25, 0.45, 180, 1000)]
    decisions = gate([Proposal(1, "forward", distance_m=0.3), Proposal(2, "forward", distance_m=0.3)], reports, 0.18)
    assert decisions[0][1] is None
    assert decisions[1][1] and "robot 1" in decisions[1][1]
    tight = gate([Proposal(1, "forward", distance_m=0.3)], [report(range_mm=400)], 0.18)
    assert tight[0][1] == "would end inside the clearance margin"
