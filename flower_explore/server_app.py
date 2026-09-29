"""Flower controller: share observations, assign distinct work, gate proposals."""

from __future__ import annotations

import json
from pathlib import Path

from flwr.app import ConfigRecord, Context, RecordDict
from flwr.serverapp import Grid, ServerApp

from sim import Coordinator, Intent, SafetyGate, STARTS, WORLD, World, render
from flower_explore.wire import decode_observation, encode_cells


app = ServerApp()


def exchange(grid: Grid, nodes: list[int], action: str,
             payloads: dict[int, dict], tick: int) -> dict[int, dict]:
    messages = [grid.create_message(
        content=RecordDict({"request": ConfigRecord(payloads[robot_id])}),
        message_type=f"query.{action}",
        dst_node_id=nodes[robot_id],
        group_id=str(tick),
    ) for robot_id in range(2)]
    # A camera observation can include a bounded VLM request (60s HTTP timeout).
    replies = list(grid.send_and_receive(messages, timeout=90 if action == "observe" else 30))
    if len(replies) != 2 or any(reply.has_error() for reply in replies):
        errors = [f"node={reply.metadata.src_node_id} code={reply.error.code} "
                  f"reason={str(reply.error.reason)[:400]}"
                  for reply in replies if reply.has_error()]
        raise RuntimeError(f"Flower {action} received {len(replies)}/2 replies; "
                           f"errors={errors}")
    by_node = {reply.metadata.src_node_id: dict(reply.content["result"])
               for reply in replies}
    if set(by_node) != set(nodes):
        raise RuntimeError(f"Flower {action} reply from unexpected node")
    return {robot_id: by_node[nodes[robot_id]] for robot_id in range(2)}


@app.main()
def main(grid: Grid, context: Context) -> None:
    nodes = sorted(grid.get_node_ids())
    if len(nodes) != 2:
        raise RuntimeError(f"Expected exactly two Flower SuperNodes, got {len(nodes)}")
    max_steps = int(context.run_config.get("steps", 30))
    if not 1 <= max_steps <= 150:
        raise ValueError("steps must be between 1 and 150")
    vlm_frame = str(context.run_config.get("vlm-frame", ""))
    jpeg = Path(vlm_frame).read_bytes() if vlm_frame else None

    world = World()
    coordinator = Coordinator()
    gate = SafetyGate()
    poses = dict(enumerate(STARTS))
    distance = {0: 0, 1: 0}
    total_free = sum(row.count(".") for row in WORLD)
    print(f"FLOWER_START nodes={nodes} steps={max_steps} motor_commands_enabled=false")

    for tick in range(max_steps + 1):
        requests = {robot_id: {
            "robot_id": robot_id, "tick": tick,
            "x": pose[0], "y": pose[1],
        } for robot_id, pose in poses.items()}
        if tick == 0 and jpeg is not None:
            requests[0]["jpeg"] = jpeg
            requests[0]["goal"] = "Describe this real ESP32 camera view and suggest a safe next look"
        received = exchange(grid, nodes, "observe", requests, tick)
        if "visual_decision_json" in received[0]:
            visual = json.loads(received[0]["visual_decision_json"])
            print("FLOWER_VLM " + json.dumps({
                "robot_id": 0, "model": "gpt-6-luna",
                "decision": visual,
                "usage": json.loads(received[0]["visual_usage_json"]),
                "motor_action_executed": False,
            }))
        observations = [decode_observation(received[robot_id], robot_id, tick)
                        for robot_id in range(2)]
        if any(observation.pose != poses[observation.robot_id]
               for observation in observations):
            raise RuntimeError("Bot observation pose mismatch")
        coordinator.merge(observations)
        covered = sum(kind == "." for kind in coordinator.known.values())
        if tick == max_steps or covered == total_free:
            break

        assignments = coordinator.assign(poses)
        known_cells = encode_cells(coordinator.known)
        requests = {}
        for assignment in assignments:
            robot_id = assignment.robot_id
            target = assignment.target or (-1, -1)
            requests[robot_id] = {
                "robot_id": robot_id,
                "x": poses[robot_id][0], "y": poses[robot_id][1],
                "target_x": target[0], "target_y": target[1],
                "known_cells": known_cells,
            }
        received = exchange(grid, nodes, "propose", requests, tick)
        intents = [Intent(robot_id, (received[robot_id]["x"], received[robot_id]["y"]))
                   for robot_id in range(2)]
        if any(received[robot_id]["robot_id"] != robot_id for robot_id in range(2)):
            raise RuntimeError("Bot proposal identity mismatch")
        next_poses = gate.apply(world, poses, intents)
        for robot_id in range(2):
            distance[robot_id] += int(next_poses[robot_id] != poses[robot_id])
        print(f"FLOWER_TICK {tick} coverage={covered / total_free:.3f} "
              f"targets={[a.target for a in assignments]} "
              f"accepted={next_poses}")
        if next_poses == poses and all(item.target is None for item in assignments):
            break
        poses = next_poses

    summary = {
        "ticks": tick,
        "coverage": round(covered / total_free, 3),
        "covered_free_cells": covered,
        "total_free_cells": total_free,
        "distance_by_robot": distance,
        "blocked_moves": gate.blocked,
        "motor_commands_enabled": False,
        "map": render(coordinator.known, poses, world),
    }
    print("FLOWER_SUMMARY " + json.dumps(summary))
