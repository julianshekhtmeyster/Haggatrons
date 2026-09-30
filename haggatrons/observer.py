"""Bounded GPT-6 Luna visual observation; never executes a motor command."""

from __future__ import annotations

import base64
import json
import os
import urllib.error
import urllib.request
from pathlib import Path


MODEL = "gpt-6-luna"
ACTIONS = {"look_left", "look_right", "look_forward", "advance_short", "stop"}
STATES = {"open", "blocked", "uncertain"}
PERSON_LOCATIONS = {"none", "left", "center", "right", "multiple"}
SCHEMA = {
    "type": "object",
    "properties": {
        "scene_summary": {"type": "string"},
        "visible_landmarks": {"type": "array", "items": {"type": "string"}},
        "directions": {
            "type": "object",
            "properties": {key: {"type": "string", "enum": sorted(STATES)}
                           for key in ("left", "center", "right")},
            "required": ["left", "center", "right"],
            "additionalProperties": False,
        },
        "visual_hazards": {"type": "array", "items": {"type": "string"}},
        "uncertainty": {"type": "string"},
        "person_visible": {"type": "boolean"},
        "person_location": {"type": "string", "enum": sorted(PERSON_LOCATIONS)},
        "proposed_action": {"type": "string", "enum": sorted(ACTIONS)},
        "action_reason": {"type": "string"},
    },
    "required": [
        "scene_summary", "visible_landmarks", "directions", "visual_hazards",
        "uncertainty", "person_visible", "person_location", "proposed_action", "action_reason",
    ],
    "additionalProperties": False,
}

INSTRUCTIONS = (
    "You are a camera observer for a small wheeled robot. Describe only what the "
    "provided image supports. You have no depth sensor, wheel odometry, or reliable "
    "global pose in this request. A visible floor does not prove it is traversable. "
    "If the view is dark, blurry, pointed upward, or otherwise insufficient, mark "
    "directions uncertain and propose a look or stop. Propose exactly one bounded "
    "action; another component will decide whether to allow it. Never claim that "
    "an action has been executed. Set person_visible to true if any person or part "
    "of a person (face, hand, leg, foot) is in the image, and give where it is; "
    "otherwise false and \"none\". A visible person is always a hazard: never "
    "propose advance_short toward one. Return JSON matching the schema."
)


def validate(value: object) -> dict:
    if not isinstance(value, dict) or set(value) != set(SCHEMA["required"]):
        raise ValueError("Invalid visual observation fields")
    if not isinstance(value["directions"], dict) or set(value["directions"]) != {"left", "center", "right"}:
        raise ValueError("Invalid direction fields")
    if any(state not in STATES for state in value["directions"].values()):
        raise ValueError("Invalid direction state")
    if not isinstance(value["person_visible"], bool) or value["person_location"] not in PERSON_LOCATIONS:
        raise ValueError("Invalid person fields")
    if value["person_visible"] != (value["person_location"] != "none"):
        raise ValueError("person_visible conflicts with person_location")
    if value["proposed_action"] not in ACTIONS:
        raise ValueError("Invalid proposed action")
    if value["proposed_action"] == "advance_short" and value["directions"]["center"] != "open":
        raise ValueError("Advance conflicts with center direction")
    for name in ("scene_summary", "uncertainty", "action_reason"):
        if not isinstance(value[name], str) or len(value[name]) > 1000:
            raise ValueError(f"Invalid {name}")
    for name in ("visible_landmarks", "visual_hazards"):
        if (not isinstance(value[name], list) or len(value[name]) > 5
                or any(not isinstance(item, str) or len(item) > 200 for item in value[name])):
            raise ValueError(f"Invalid {name}")
    return value


def load_api_key() -> str:
    key = os.environ.get("OPENAI_API_KEY", "").strip()
    if key:
        return key
    path = Path(__file__).resolve().parents[1] / ".env"
    if path.exists():
        for line in path.read_text(encoding="utf-8-sig").splitlines():
            if line.startswith("OPENAI_API_KEY="):
                return line.partition("=")[2].strip().strip('"').strip("'")
    return ""


def observe_jpeg(jpeg: bytes, goal: str = "Describe the scene and suggest a safe next look",
                 api_key: str | None = None, model: str = MODEL) -> tuple[dict, dict]:
    key = (api_key if api_key is not None else load_api_key()).strip()
    if not key:
        raise RuntimeError("OPENAI_API_KEY is missing from the Flower worker environment")
    if not jpeg.startswith(b"\xff\xd8"):
        raise ValueError("Expected JPEG image bytes")
    if len(jpeg) > 2_000_000:
        raise ValueError("Image exceeds the 2 MB bench-test limit")
    if len(goal) > 500:
        raise ValueError("Goal is too long")
    body = {
        "model": model,
        "store": False,
        "reasoning": {"effort": "low"},
        "max_output_tokens": 1000,
        "instructions": INSTRUCTIONS,
        "input": [{"role": "user", "content": [
            {"type": "input_text", "text": f"Goal: {goal}"},
            {"type": "input_image", "image_url": "data:image/jpeg;base64," +
             base64.b64encode(jpeg).decode("ascii"), "detail": "low"},
        ]}],
        "text": {"format": {
            "type": "json_schema", "name": "bot_visual_observation",
            "strict": True, "schema": SCHEMA,
        }},
    }
    request = urllib.request.Request(
        "https://api.openai.com/v1/responses",
        data=json.dumps(body).encode("utf-8"),
        headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            result = json.load(response)
    except urllib.error.HTTPError as exc:
        detail = exc.read(1000).decode("utf-8", errors="replace").replace(key, "[REDACTED]")
        raise RuntimeError(f"OpenAI HTTP {exc.code}: {detail}") from None
    if result.get("status") != "completed":
        raise RuntimeError(f"OpenAI response did not complete: {result.get('status')}")
    output_text = "".join(
        part.get("text", "")
        for item in result.get("output", []) if item.get("type") == "message"
        for part in item.get("content", []) if part.get("type") == "output_text"
    )
    if not output_text:
        raise RuntimeError("OpenAI response contained no visual decision text")
    return validate(json.loads(output_text)), result.get("usage", {})
