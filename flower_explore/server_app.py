"""Flower ServerApp: the mission coordinator.

Each tick, for every robot's worker (one Flower SuperNode per robot):
  1. query.observe  → the worker captures a frame, IMU and range sample and may
                      ask the visual model; it returns a local report.
  2. The coordinator merges reports into the shared map and assigns distinct goals.
  3. query.decide   → each worker turns its goal into one bounded proposal.
  4. The coordinator gates proposals (clearance, robot-robot separation).
  5. query.execute  → workers run approved moves through the backend, which
                      re-checks arming, e-stop and the mission state, and
                      report what actually happened.
The operator's pause/stop is honoured between every phase.
"""

from __future__ import annotations

import json

from flwr.app import ConfigRecord, Context, RecordDict
from flwr.serverapp import Grid, ServerApp

from haggatrons.client import BackendClient
from haggatrons.loop import Coordinator, wait_while_paused

app = ServerApp()
TIMEOUTS = {"observe": 100, "decide": 30, "execute": 30}


def exchange(grid: Grid, node_for: dict[int, int], action: str, payloads: dict[int, dict],
             tick: int) -> dict[int, dict]:
    messages = [grid.create_message(
        content=RecordDict({"request": ConfigRecord({"payload": json.dumps(payloads[robot_id])})}),
        message_type=f"query.{action}", dst_node_id=node_for[robot_id], group_id=str(tick),
    ) for robot_id in payloads]
    replies = list(grid.send_and_receive(messages, timeout=TIMEOUTS[action]))
    errors = [f"node={r.metadata.src_node_id} {r.error.reason!s:.300}" for r in replies if r.has_error()]
    if len(replies) != len(messages) or errors:
        raise RuntimeError(f"{action}: {len(replies)}/{len(messages)} replies, errors={errors}")
    robot_for = {node: robot_id for robot_id, node in node_for.items()}
    results = {}
    for reply in replies:
        result = json.loads(reply.content["result"]["payload"])
        robot_id = robot_for[reply.metadata.src_node_id]
        if result.get("robot_id") != robot_id:
            raise RuntimeError(f"{action}: reply from node {reply.metadata.src_node_id} claims "
                               f"robot {result.get('robot_id')}, expected {robot_id}")
        results[robot_id] = result
    return results


@app.main()
def main(grid: Grid, context: Context) -> None:
    api = BackendClient()
    steps = int(context.run_config.get("steps", 8))
    vision = str(context.run_config.get("vision", "false")).lower() == "true"
    clearance = int(context.run_config.get("clearance-mm", 180)) / 1000
    # Only the robots that were online when the operator started the mission take part.
    robot_ids = sorted(api.get("/api/mission/control")["robot_ids"])
    nodes = sorted(grid.get_node_ids())
    if len(nodes) != len(robot_ids):
        raise RuntimeError(f"Expected one Flower SuperNode per robot ({len(robot_ids)}), got {len(nodes)}")
    node_for = dict(zip(robot_ids, nodes))
    print(f"HAGGATRONS_START robots={robot_ids} nodes={nodes} steps={steps} vision={vision}", flush=True)

    coordinator = Coordinator(clearance)
    tick = 0
    for tick in range(steps):
        if wait_while_paused(api) != "running":
            break
        reports = exchange(grid, node_for, "observe",
                           {rid: {"robot_id": rid, "tick": tick, "vision": vision} for rid in robot_ids}, tick)
        coordinator.merge(list(reports.values()))
        goals = {goal["robot_id"]: goal for goal in coordinator.assign()}
        api.post("/api/coordinator", {**coordinator.state(tick), "phase": "assigned"})

        proposals = exchange(grid, node_for, "decide", {
            rid: {"robot_id": rid, "report": reports[rid], "goal": goals[rid], "clearance_m": clearance}
            for rid in robot_ids}, tick)
        decisions = coordinator.approve(list(proposals.values()))
        api.post("/api/log", {"kind": "gate", "tick": tick, "decisions": [
            {"proposal": prop, "approved": refusal is None, "refusal": refusal} for prop, refusal in decisions]})

        if wait_while_paused(api) != "running":
            break
        approved = {prop["robot_id"]: prop if refusal is None else {**prop, "action": "hold", "reason": refusal}
                    for prop, refusal in decisions}
        outcomes = exchange(grid, node_for, "execute",
                            {rid: {"robot_id": rid, "tick": tick, "approved": approved[rid]} for rid in robot_ids},
                            tick)
        print("HAGGATRONS_TICK " + json.dumps({
            "tick": tick, "coverage": coordinator.map.coverage(),
            "outcomes": {rid: o.get("outcome") for rid, o in outcomes.items()}}), flush=True)

    summary = {"ticks": tick + 1, "robots": robot_ids, **coordinator.map.coverage()}
    api.post("/api/coordinator", {**coordinator.state(tick), "phase": "finished", "summary": summary})
    print("HAGGATRONS_SUMMARY " + json.dumps(summary), flush=True)
