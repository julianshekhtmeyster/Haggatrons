"""Python mirror of firmware/libraries/HaggatronsProtocol/src/HaggatronsProtocol.h.

Robot messages travel over ESP-NOW between the master and each robot. Serial
frames travel over USB between the Mac and the master:
``COBS([type][payload][crc16 LE]) + b"\\x00"``. Keep this file in step with the
header; tests/test_protocol.py compares the constants.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

PROTOCOL_VERSION = 1
MAGIC = 0x48
KEY_LEN = 16
MAX_ROBOTS = 4
ESPNOW_MAX_PAYLOAD = 1470
CHUNK_DATA = 1400
MAX_MOVE_MS = 2000
MAX_JOG_MS_WITHOUT_RANGE = 500
MAX_TURN_DDEG = 1800
LINK_TIMEOUT_MS = 400
HOST_TIMEOUT_MS = 500
HEARTBEAT_PERIOD_MS = 100
TELEMETRY_PERIOD_MS = 200

# Robot message types
MSG_HEARTBEAT = 0x01
MSG_CAPTURE = 0x02
MSG_MOVE = 0x03
MSG_STOP = 0x04
MSG_RESEND = 0x05
MSG_TELEMETRY = 0x10
MSG_FRAME_META = 0x11
MSG_FRAME_CHUNK = 0x12
MSG_MOVE_RESULT = 0x13
MSG_REJECT = 0x14

HB_ARMED = 0x01
HB_ESTOP = 0x02
HB_HOST_OK = 0x04

MOVE_DRIVE = 1
MOVE_TURN = 2
MOVE_FORWARD = 3
MOVE_KINDS = {MOVE_DRIVE: "drive", MOVE_TURN: "turn", MOVE_FORWARD: "forward"}

OUT_COMPLETED = 0
OUT_ESTOP = 1
OUT_LINK_LOST = 2
OUT_OBSTACLE = 3
OUT_HOST_STOP = 4
OUT_TURN_TIMEOUT = 5
OUT_DISARMED = 6
OUT_RANGE_LOST = 7
OUTCOMES = {
    OUT_COMPLETED: "completed",
    OUT_ESTOP: "stopped_estop",
    OUT_LINK_LOST: "stopped_link_lost",
    OUT_OBSTACLE: "stopped_obstacle",
    OUT_HOST_STOP: "stopped_host",
    OUT_TURN_TIMEOUT: "turn_timeout",
    OUT_DISARMED: "stopped_disarmed",
    OUT_RANGE_LOST: "stopped_range_lost",
}

REJ_NOT_ARMED = 1
REJ_ESTOP = 2
REJ_BUSY = 3
REJ_BAD_PARAMS = 4
REJ_IMU_UNAVAILABLE = 5
REJ_RANGE_UNAVAILABLE = 6
REJ_OBSTACLE = 7
REJ_CAMERA_FAILED = 8
REJ_FRAME_GONE = 9
REJ_LINK_DOWN = 10
REJECTS = {
    REJ_NOT_ARMED: "not_armed",
    REJ_ESTOP: "estop",
    REJ_BUSY: "busy",
    REJ_BAD_PARAMS: "bad_params",
    REJ_IMU_UNAVAILABLE: "imu_unavailable",
    REJ_RANGE_UNAVAILABLE: "range_unavailable",
    REJ_OBSTACLE: "obstacle",
    REJ_CAMERA_FAILED: "camera_failed",
    REJ_FRAME_GONE: "frame_gone",
    REJ_LINK_DOWN: "link_down",
}

TEL_ARMED = 0x0001
TEL_ESTOP = 0x0002
TEL_LINK_OK = 0x0004
TEL_IMU_OK = 0x0008
TEL_RANGE_OK = 0x0010
TEL_MOVING = 0x0020
TEL_CAMERA_OK = 0x0040

MOTOR_INVERT_LEFT = 0x01
MOTOR_INVERT_RIGHT = 0x02
MOTOR_SWAP_SIDES = 0x04

# Serial frame types
SER_HELLO = 0x01
SER_RECV = 0x02
SER_STATUS = 0x03
SER_LOG = 0x04
SER_CONFIG_ACK = 0x05
SER_CONFIG = 0x81
SER_SEND = 0x82
SER_KEEPALIVE = 0x83
SER_STOP_ALL = 0x84
SER_HELLO_REQ = 0x85

BROADCAST = 0xFF
FRAMESIZES = {"320x240": 0, "640x480": 1, "800x600": 2, "1024x768": 3}

HEADER = struct.Struct("<BBBBH")
HEARTBEAT = struct.Struct("<BI")
CAPTURE = struct.Struct("<IBB")
MOVE = struct.Struct("<IBBhhHhH")
STOP = struct.Struct("<I")
RESEND = struct.Struct("<IB")
TELEMETRY = struct.Struct("<IHH3f3fHIBBB")
FRAME_META = struct.Struct("<IIHHHHIIBBH3f3f")
FRAME_CHUNK = struct.Struct("<IH")
MOVE_RESULT = struct.Struct("<IBBHfHhh")
REJECT = struct.Struct("<IBB")
PROVISION = struct.Struct(f"<BB6sB{KEY_LEN}s{KEY_LEN}sBBbH")

SER_HELLO_S = struct.Struct("<BBB6sH")
SER_PEER = struct.Struct(f"<B6s{KEY_LEN}s")
SER_CONFIG_S = struct.Struct(f"<B{KEY_LEN}sB")
SER_RECV_HEADER = struct.Struct("<Bb")
SER_KEEPALIVE_S = struct.Struct("<B")
SER_PEER_STATUS = struct.Struct("<BIII")
SER_STATUS_S = struct.Struct("<IBBB")
SER_CONFIG_ACK_S = struct.Struct("<BB")


class ProtocolError(ValueError):
    pass


# ---------------------------------------------------------------- framing
def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, matching hgCrc16."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray(b"\x00")
    code_at, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_at] = code
            code_at, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_at] = code
                code_at, code = len(out), 1
                out.append(0)
    out[code_at] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0 or index + code > len(data):
            raise ProtocolError("Malformed COBS frame")
        out += data[index + 1:index + code]
        index += code
        if code < 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def encode_frame(frame_type: int, payload: bytes = b"") -> bytes:
    raw = bytes([frame_type]) + payload
    return cobs_encode(raw + struct.pack("<H", crc16(raw))) + b"\x00"


def decode_frame(encoded: bytes) -> tuple[int, bytes]:
    """Decode one frame without its 0x00 delimiter."""
    raw = cobs_decode(encoded)
    if len(raw) < 3:
        raise ProtocolError("Serial frame too short")
    body, (crc,) = raw[:-2], struct.unpack("<H", raw[-2:])
    if crc16(body) != crc:
        raise ProtocolError("Serial frame CRC mismatch")
    return body[0], body[1:]


class FrameReader:
    """Splits a byte stream into decoded serial frames; drops corrupt ones."""

    def __init__(self, limit: int = 8192) -> None:
        self.buffer = bytearray()
        self.limit = limit
        self.dropped = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        frames = []
        for byte in data:
            if byte == 0:
                if self.buffer:
                    try:
                        frames.append(decode_frame(bytes(self.buffer)))
                    except ProtocolError:
                        self.dropped += 1
                self.buffer.clear()
            elif len(self.buffer) < self.limit:
                self.buffer.append(byte)
            else:
                self.buffer.clear()
                self.dropped += 1
        return frames


# ---------------------------------------------------------------- robot messages
@dataclass(frozen=True)
class Header:
    type: int
    robot_id: int
    seq: int


def pack_message(msg_type: int, robot_id: int, seq: int, body: bytes) -> bytes:
    return HEADER.pack(MAGIC, PROTOCOL_VERSION, msg_type, robot_id, seq & 0xFFFF) + body


def unpack_message(data: bytes) -> tuple[Header, bytes]:
    if len(data) < HEADER.size:
        raise ProtocolError("Robot message too short")
    magic, version, msg_type, robot_id, seq = HEADER.unpack_from(data)
    if magic != MAGIC or version != PROTOCOL_VERSION:
        raise ProtocolError("Robot message magic or version mismatch")
    return Header(msg_type, robot_id, seq), data[HEADER.size:]


@dataclass(frozen=True)
class MoveCommand:
    kind: int
    left: int = 0
    right: int = 0
    duration_ms: int = 500
    turn_deg: float = 0.0
    min_clear_mm: int = 0

    def validate(self) -> None:
        if self.kind not in MOVE_KINDS:
            raise ValueError("Unknown move kind")
        if not 0 < self.duration_ms <= MAX_MOVE_MS:
            raise ValueError(f"duration_ms must be 1-{MAX_MOVE_MS}")
        if abs(self.left) > 1000 or abs(self.right) > 1000:
            raise ValueError("Wheel speeds must be within ±1000 permille")
        if self.kind == MOVE_TURN and not 0 < abs(self.turn_deg) <= MAX_TURN_DDEG / 10:
            raise ValueError("Turn must be non-zero and at most 180 degrees")
        if self.kind == MOVE_FORWARD and (self.left <= 0 or self.right <= 0):
            raise ValueError("Forward moves need positive wheel speeds")
        if not 0 <= self.min_clear_mm <= 2000:
            raise ValueError("min_clear_mm must be 0-2000")

    def pack(self, req_id: int) -> bytes:
        self.validate()
        return MOVE.pack(req_id, self.kind, 0, self.left, self.right, self.duration_ms,
                         round(self.turn_deg * 10), self.min_clear_mm)

    @classmethod
    def unpack(cls, body: bytes) -> tuple[int, "MoveCommand"]:
        req_id, kind, _, left, right, duration, turn, clear = MOVE.unpack(body)
        return req_id, cls(kind, left, right, duration, turn / 10, clear)

    def describe(self) -> dict:
        return {"kind": MOVE_KINDS.get(self.kind, "?"), "left": self.left, "right": self.right,
                "duration_ms": self.duration_ms, "turn_deg": self.turn_deg,
                "min_clear_mm": self.min_clear_mm}


@dataclass(frozen=True)
class Telemetry:
    uptime_ms: int
    flags: int
    heartbeat_age_ms: int
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    range_mm: int
    active_req: int
    last_outcome: int
    firmware: str

    @classmethod
    def unpack(cls, body: bytes) -> "Telemetry":
        v = TELEMETRY.unpack(body)
        return cls(v[0], v[1], v[2], tuple(v[3:6]), tuple(v[6:9]), v[9], v[10], v[11], f"{v[12]}.{v[13]}")

    def pack(self) -> bytes:
        major, minor = (int(part) for part in self.firmware.split("."))
        return TELEMETRY.pack(self.uptime_ms, self.flags, self.heartbeat_age_ms, *self.accel_g,
                              *self.gyro_dps, self.range_mm, self.active_req, self.last_outcome,
                              major, minor)

    def as_dict(self) -> dict:
        f = self.flags
        return {
            "uptime_ms": self.uptime_ms,
            "armed": bool(f & TEL_ARMED), "estop": bool(f & TEL_ESTOP),
            "link_ok": bool(f & TEL_LINK_OK), "imu_ok": bool(f & TEL_IMU_OK),
            "range_ok": bool(f & TEL_RANGE_OK), "moving": bool(f & TEL_MOVING),
            "camera_ok": bool(f & TEL_CAMERA_OK),
            "heartbeat_age_ms": self.heartbeat_age_ms,
            "accel_g": list(self.accel_g), "gyro_dps": list(self.gyro_dps),
            "range_mm": self.range_mm if f & TEL_RANGE_OK else None,
            "active_req": self.active_req,
            "last_outcome": OUTCOMES.get(self.last_outcome, "unknown"),
            "firmware": self.firmware,
        }


@dataclass(frozen=True)
class FrameMeta:
    req_id: int
    jpeg_len: int
    chunk_count: int
    chunk_size: int
    width: int
    height: int
    frame_ms: int
    imu_ms: int
    imu_valid: bool
    range_valid: bool
    range_mm: int
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]

    @classmethod
    def unpack(cls, body: bytes) -> "FrameMeta":
        v = FRAME_META.unpack(body)
        return cls(*v[:8], bool(v[8]), bool(v[9]), v[10], tuple(v[11:14]), tuple(v[14:17]))

    def pack(self) -> bytes:
        return FRAME_META.pack(self.req_id, self.jpeg_len, self.chunk_count, self.chunk_size,
                               self.width, self.height, self.frame_ms, self.imu_ms,
                               int(self.imu_valid), int(self.range_valid), self.range_mm,
                               *self.accel_g, *self.gyro_dps)


@dataclass(frozen=True)
class MoveResult:
    req_id: int
    outcome: int
    kind: int
    elapsed_ms: int
    yaw_deg: float
    min_range_mm: int | None
    left: int
    right: int

    @classmethod
    def unpack(cls, body: bytes) -> "MoveResult":
        req_id, outcome, kind, elapsed, yaw, min_range, left, right = MOVE_RESULT.unpack(body)
        return cls(req_id, outcome, kind, elapsed, yaw, None if min_range == 0xFFFF else min_range,
                   left, right)

    def pack(self) -> bytes:
        return MOVE_RESULT.pack(self.req_id, self.outcome, self.kind, self.elapsed_ms, self.yaw_deg,
                                0xFFFF if self.min_range_mm is None else self.min_range_mm,
                                self.left, self.right)

    def as_dict(self) -> dict:
        return {"req_id": self.req_id, "outcome": OUTCOMES.get(self.outcome, "unknown"),
                "move_kind": MOVE_KINDS.get(self.kind, "?"), "elapsed_ms": self.elapsed_ms,
                "yaw_deg": round(self.yaw_deg, 2), "min_range_mm": self.min_range_mm,
                "applied_left": self.left, "applied_right": self.right}


@dataclass(frozen=True)
class Provision:
    robot_id: int
    master_mac: bytes
    channel: int
    pmk: bytes
    lmk: bytes
    motor_flags: int = 0
    yaw_axis: int = 2
    yaw_sign: int = 1
    max_speed: int = 600

    def pack(self) -> bytes:
        if not 1 <= self.robot_id <= MAX_ROBOTS:
            raise ValueError(f"robot_id must be 1-{MAX_ROBOTS}")
        if not 1 <= self.channel <= 13 or self.yaw_axis not in (0, 1, 2):
            raise ValueError("Invalid channel or yaw axis")
        if len(self.master_mac) != 6 or len(self.pmk) != KEY_LEN or len(self.lmk) != KEY_LEN:
            raise ValueError("Invalid MAC or key length")
        if not 0 <= self.max_speed <= 1000:
            raise ValueError("max_speed must be 0-1000")
        return b"P" + PROVISION.pack(PROTOCOL_VERSION, self.robot_id, self.master_mac, self.channel,
                                     self.pmk, self.lmk, self.motor_flags, self.yaw_axis,
                                     -1 if self.yaw_sign < 0 else 1, self.max_speed)


@dataclass(frozen=True)
class PeerConfig:
    robot_id: int
    mac: bytes
    lmk: bytes


def pack_config(channel: int, pmk: bytes, peers: list[PeerConfig]) -> bytes:
    if len(peers) > MAX_ROBOTS:
        raise ValueError(f"At most {MAX_ROBOTS} robots")
    return SER_CONFIG_S.pack(channel, pmk, len(peers)) + b"".join(
        SER_PEER.pack(p.robot_id, p.mac, p.lmk) for p in peers)


def unpack_config(payload: bytes) -> tuple[int, bytes, list[PeerConfig]]:
    channel, pmk, count = SER_CONFIG_S.unpack_from(payload)
    if len(payload) != SER_CONFIG_S.size + count * SER_PEER.size:
        raise ProtocolError("Config length mismatch")
    peers = [PeerConfig(*SER_PEER.unpack_from(payload, SER_CONFIG_S.size + i * SER_PEER.size))
             for i in range(count)]
    return channel, pmk, peers


@dataclass
class MasterStatus:
    uptime_ms: int
    flags: int
    channel: int
    peers: dict[int, dict] = field(default_factory=dict)

    @classmethod
    def unpack(cls, payload: bytes) -> "MasterStatus":
        uptime, flags, channel, count = SER_STATUS_S.unpack_from(payload)
        peers = {}
        for i in range(count):
            robot_id, age, ok, fail = SER_PEER_STATUS.unpack_from(
                payload, SER_STATUS_S.size + i * SER_PEER_STATUS.size)
            peers[robot_id] = {"last_rx_age_ms": None if age == 0xFFFFFFFF else age,
                               "tx_ok": ok, "tx_fail": fail}
        return cls(uptime, flags, channel, peers)

    def pack(self) -> bytes:
        return SER_STATUS_S.pack(self.uptime_ms, self.flags, self.channel, len(self.peers)) + b"".join(
            SER_PEER_STATUS.pack(robot_id, 0xFFFFFFFF if p["last_rx_age_ms"] is None else p["last_rx_age_ms"],
                                 p["tx_ok"], p["tx_fail"])
            for robot_id, p in self.peers.items())


def format_mac(mac: bytes) -> str:
    return ":".join(f"{b:02x}" for b in mac)


def parse_mac(text: str) -> bytes:
    parts = text.strip().lower().replace("-", ":").split(":")
    if len(parts) != 6:
        raise ValueError(f"Invalid MAC address: {text!r}")
    return bytes(int(part, 16) for part in parts)
