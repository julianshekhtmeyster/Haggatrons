"""Small, explicit message codec shared by Flower server and workers."""

from __future__ import annotations

import json

from sim import Observation, Point


def encode_cells(cells: dict[Point, str]) -> str:
    return json.dumps([[x, y, kind] for (x, y), kind in sorted(cells.items())])


def decode_cells(raw: str) -> dict[Point, str]:
    cells = json.loads(raw)
    if not isinstance(cells, list):
        raise ValueError("Cells must be a list")
    result: dict[Point, str] = {}
    for row in cells:
        if (not isinstance(row, list) or len(row) != 3
                or type(row[0]) is not int or type(row[1]) is not int
                or row[2] not in (".", "#")):
            raise ValueError("Invalid cell entry")
        result[(row[0], row[1])] = row[2]
    return result


def decode_observation(record: dict, expected_robot: int, expected_tick: int) -> Observation:
    if record["robot_id"] != expected_robot or record["tick"] != expected_tick:
        raise ValueError("Stale or misrouted bot observation")
    return Observation(
        robot_id=expected_robot,
        tick=expected_tick,
        pose=(record["x"], record["y"]),
        cells=decode_cells(record["cells"]),
    )
