"""One Flower ClientApp runs as each simulated bot worker."""

from __future__ import annotations

import json

from flwr.app import ConfigRecord, Context, Message, RecordDict
from flwr.clientapp import ClientApp

from sim import Assignment, Coordinator, RobotWorker, World
from flower_explore.wire import decode_cells, encode_cells
from flower_explore.openai_observer import observe_jpeg


app = ClientApp()


@app.query("observe")
def observe(message: Message, context: Context) -> Message:
    request = message.content["request"]
    robot_id = int(request["robot_id"])
    tick = int(request["tick"])
    pose = (int(request["x"]), int(request["y"]))
    observation = World().observe(robot_id, tick, pose)
    result = {
        "robot_id": robot_id,
        "tick": tick,
        "x": pose[0],
        "y": pose[1],
        "cells": encode_cells(observation.cells),
    }
    if "jpeg" in request:
        decision, usage = observe_jpeg(bytes(request["jpeg"]), str(request["goal"]))
        result["visual_decision_json"] = json.dumps(decision)
        result["visual_usage_json"] = json.dumps(usage)
    content = RecordDict({"result": ConfigRecord(result)})
    return Message(content=content, reply_to=message)


@app.query("propose")
def propose(message: Message, context: Context) -> Message:
    request = message.content["request"]
    robot_id = int(request["robot_id"])
    pose = (int(request["x"]), int(request["y"]))
    target = (int(request["target_x"]), int(request["target_y"]))
    assignment = Assignment(robot_id, None if target == (-1, -1) else target)
    map_state = Coordinator()
    map_state.known = decode_cells(str(request["known_cells"]))
    intent = RobotWorker(robot_id).propose(pose, assignment, map_state)
    content = RecordDict({"result": ConfigRecord({
        "robot_id": robot_id,
        "x": intent.next_cell[0],
        "y": intent.next_cell[1],
    })})
    return Message(content=content, reply_to=message)
