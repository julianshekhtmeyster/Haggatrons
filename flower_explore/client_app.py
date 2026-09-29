"""Flower ClientApp: one robot worker per SuperNode.

The worker is stateless between messages: the coordinator names the robot in
every request, and all robot I/O goes through the backend API.
"""

from __future__ import annotations

import json

from flwr.app import ConfigRecord, Context, Message, RecordDict
from flwr.clientapp import ClientApp

from haggatrons.client import BackendClient
from haggatrons.loop import decide, execute, observe

app = ClientApp()


def _request(message: Message) -> dict:
    return json.loads(message.content["request"]["payload"])


def _reply(message: Message, result: dict) -> Message:
    content = RecordDict({"result": ConfigRecord({"payload": json.dumps(result, default=str)})})
    return Message(content=content, reply_to=message)


@app.query("observe")
def observe_query(message: Message, context: Context) -> Message:
    request = _request(message)
    report = observe(BackendClient(), int(request["robot_id"]), int(request["tick"]), bool(request["vision"]))
    return _reply(message, report)


@app.query("decide")
def decide_query(message: Message, context: Context) -> Message:
    request = _request(message)
    return _reply(message, decide(request["report"], request["goal"], float(request["clearance_m"])))


@app.query("execute")
def execute_query(message: Message, context: Context) -> Message:
    request = _request(message)
    robot_id = int(request["robot_id"])
    outcome = execute(BackendClient(), robot_id, request["approved"], int(request["tick"]))
    return _reply(message, {"robot_id": robot_id, **outcome})
