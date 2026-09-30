"""Fleet configuration: public facts in config/fleet.json, keys in runs/fleet_keys.json."""

from __future__ import annotations

import json
import os
import secrets
from dataclasses import dataclass, field
from pathlib import Path

from haggatrons.protocol import KEY_LEN, MAX_ROBOTS, PeerConfig, parse_mac

ROOT = Path(__file__).resolve().parents[1]
FLEET_PATH = ROOT / "config" / "fleet.json"
KEYS_PATH = ROOT / "runs" / "fleet_keys.json"
RUNS = ROOT / "runs"


@dataclass(frozen=True)
class Calibration:
    forward_mps_at_full_speed: float = 0.3
    cruise_speed: int = 450
    turn_speed: int = 420
    min_clear_mm: int = 180
    max_speed: int = 600
    # Timed in-place spin rate at turn_speed (scan tasks spin by time, not gyro).
    spin_deg_per_s: float = 21.0


@dataclass(frozen=True)
class RobotConfig:
    id: int
    name: str
    mac: bytes
    motor_flags: int = 0
    yaw_axis: int = 2
    yaw_sign: int = 1
    start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class FleetConfig:
    channel: int
    robots: list[RobotConfig]
    calibration: Calibration = field(default_factory=Calibration)
    master_mac: bytes | None = None

    def robot(self, robot_id: int) -> RobotConfig:
        for robot in self.robots:
            if robot.id == robot_id:
                return robot
        raise KeyError(f"Unknown robot {robot_id}")

    @property
    def ids(self) -> list[int]:
        return [robot.id for robot in self.robots]


def load_fleet(path: Path = FLEET_PATH) -> FleetConfig:
    raw = json.loads(path.read_text(encoding="utf-8"))
    robots = [RobotConfig(
        id=int(item["id"]), name=str(item.get("name", f"Robot {item['id']}")),
        mac=parse_mac(item["mac"]), motor_flags=int(item.get("motor_flags", 0)),
        yaw_axis=int(item.get("yaw_axis", 2)), yaw_sign=int(item.get("yaw_sign", 1)),
        start_pose=tuple(float(v) for v in item.get("start_pose", (0, 0, 0))),
    ) for item in raw["robots"]]
    ids = [robot.id for robot in robots]
    if len(set(ids)) != len(ids) or not all(1 <= i <= MAX_ROBOTS for i in ids):
        raise ValueError(f"Robot ids must be unique and within 1-{MAX_ROBOTS}")
    if len({robot.mac for robot in robots}) != len(robots):
        raise ValueError("Robot MAC addresses must be unique")
    channel = int(raw.get("channel", 1))
    if not 1 <= channel <= 13:
        raise ValueError("Channel must be 1-13")
    master = (raw.get("master") or {}).get("mac")
    return FleetConfig(channel, robots, Calibration(**raw.get("calibration", {})),
                       parse_mac(master) if master else None)


def save_master_mac(mac: str, path: Path = FLEET_PATH) -> None:
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw.setdefault("master", {})["mac"] = mac
    path.write_text(json.dumps(raw, indent=2) + "\n", encoding="utf-8")


@dataclass(frozen=True)
class FleetKeys:
    pmk: bytes
    lmk: dict[int, bytes]

    def peers(self, fleet: FleetConfig) -> list[PeerConfig]:
        return [PeerConfig(robot.id, robot.mac, self.lmk[robot.id]) for robot in fleet.robots]


def write_private(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(text)


def load_keys(fleet: FleetConfig, path: Path = KEYS_PATH) -> FleetKeys:
    """Load ESP-NOW keys, generating any that are missing. Never logged."""
    raw = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    changed = False
    if len(bytes.fromhex(raw.get("pmk", ""))) != KEY_LEN:
        raw["pmk"] = secrets.token_hex(KEY_LEN)
        changed = True
    lmks = raw.setdefault("lmk", {})
    for robot in fleet.robots:
        if len(bytes.fromhex(lmks.get(str(robot.id), ""))) != KEY_LEN:
            lmks[str(robot.id)] = secrets.token_hex(KEY_LEN)
            changed = True
    if changed:
        write_private(path, json.dumps(raw, indent=2) + "\n")
    return FleetKeys(bytes.fromhex(raw["pmk"]),
                     {int(k): bytes.fromhex(v) for k, v in lmks.items()})
