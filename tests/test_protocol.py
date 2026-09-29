import re
import shutil
import subprocess
from pathlib import Path

import pytest

from haggatrons import protocol as p

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "firmware" / "libraries" / "HaggatronsProtocol" / "src" / "HaggatronsProtocol.h"


def test_cobs_round_trip_and_no_zero_bytes():
    samples = [b"", b"\x00", b"\x00\x00", b"abc", bytes(range(256)) * 3, b"\x01" * 254, b"\x01" * 255]
    for sample in samples:
        encoded = p.cobs_encode(sample)
        assert 0 not in encoded
        assert p.cobs_decode(encoded) == sample


def test_frame_reader_splits_and_drops_corrupt_frames():
    good = p.encode_frame(p.SER_LOG, b"hello\x00world")
    corrupt = bytearray(p.encode_frame(p.SER_LOG, b"bad"))
    corrupt[2] ^= 0x55
    reader = p.FrameReader()
    frames = reader.feed(good[:5]) + reader.feed(good[5:] + bytes(corrupt) + good)
    assert frames == [(p.SER_LOG, b"hello\x00world")] * 2
    assert reader.dropped == 1


def test_crc_matches_ccitt_false_check_value():
    assert p.crc16(b"123456789") == 0x29B1


def test_move_command_validation():
    p.MoveCommand(p.MOVE_TURN, 400, 400, 1500, turn_deg=90).validate()
    with pytest.raises(ValueError):
        p.MoveCommand(p.MOVE_DRIVE, 400, 400, p.MAX_MOVE_MS + 1).validate()
    with pytest.raises(ValueError):
        p.MoveCommand(p.MOVE_FORWARD, 400, -400, 500).validate()
    with pytest.raises(ValueError):
        p.MoveCommand(p.MOVE_TURN, 400, 400, 500, turn_deg=181).validate()
    req, cmd = p.MoveCommand.unpack(p.MoveCommand(p.MOVE_TURN, 300, 300, 900, -45.5, 120).pack(7)[0:])
    assert req == 7 and cmd.turn_deg == -45.5 and cmd.min_clear_mm == 120


def _header_constants() -> dict[str, int]:
    text = HEADER.read_text()
    values = {}
    for name, value in re.findall(r"#define (HG_[A-Z0-9_]+) (0x[0-9A-Fa-f]+|\d+)", text):
        values[name] = int(value, 0)
    for name, value in re.findall(r"\b(HG_[A-Z0-9_]+) = (0x[0-9A-Fa-f]+|\d+)", text):
        values[name] = int(value, 0)
    return values


def test_python_constants_match_header():
    constants = _header_constants()
    renames = {"HG_PROTOCOL_VERSION": "PROTOCOL_VERSION", "HG_MAGIC": "MAGIC"}
    checked = 0
    for name, value in constants.items():
        attr = renames.get(name, name.removeprefix("HG_"))
        if attr.startswith("REJ_") or attr.startswith("OUT_") or hasattr(p, attr):
            assert getattr(p, attr) == value, name
            checked += 1
    assert checked > 50


STRUCTS = {
    "HgHeader": p.HEADER, "HgHeartbeat": p.HEARTBEAT, "HgCapture": p.CAPTURE, "HgMove": p.MOVE,
    "HgStop": p.STOP, "HgResend": p.RESEND, "HgTelemetry": p.TELEMETRY, "HgFrameMeta": p.FRAME_META,
    "HgFrameChunk": p.FRAME_CHUNK, "HgMoveResult": p.MOVE_RESULT, "HgRejectMsg": p.REJECT,
    "HgProvision": p.PROVISION, "HgSerHello": p.SER_HELLO_S, "HgSerPeer": p.SER_PEER,
    "HgSerConfig": p.SER_CONFIG_S, "HgSerRecvHeader": p.SER_RECV_HEADER,
    "HgSerKeepalive": p.SER_KEEPALIVE_S, "HgSerPeerStatus": p.SER_PEER_STATUS,
    "HgSerStatus": p.SER_STATUS_S, "HgSerConfigAck": p.SER_CONFIG_ACK_S,
}


@pytest.mark.skipif(shutil.which("cc") is None, reason="needs a C compiler")
def test_struct_sizes_and_cobs_match_c(tmp_path):
    sample = bytes([0, 1, 2, 0, 0]) + bytes(range(1, 255)) * 2
    source = tmp_path / "check.c"
    source.write_text(
        "#include <stdio.h>\n#include \"HaggatronsProtocol.h\"\nint main(void){\n"
        + "".join(f'printf("{name} %zu\\n", sizeof(struct {name}));\n' for name in STRUCTS)
        + f"static const uint8_t in[] = {{{','.join(map(str, sample))}}};\n"
        + "uint8_t enc[1024], dec[1024]; size_t n = hgCobsEncode(in, sizeof in, enc);\n"
        + 'printf("COBS "); for (size_t i = 0; i < n; ++i) printf("%02x", enc[i]); printf("\\n");\n'
        + 'size_t m = hgCobsDecode(enc, n, dec); printf("ROUND %d\\n", m == sizeof in && !memcmp(dec, in, m));\n'
        + 'printf("CRC %u\\n", hgCrc16((const uint8_t*)"123456789", 9));\nreturn 0;}\n'
    )
    binary = tmp_path / "check"
    subprocess.run(["cc", "-std=c11", "-include", "string.h", f"-I{HEADER.parent}", str(source), "-o", str(binary)],
                   check=True)
    output = dict(line.split(" ", 1) for line in subprocess.run([str(binary)], check=True, capture_output=True,
                                                                 text=True).stdout.splitlines())
    for name, layout in STRUCTS.items():
        assert int(output[name]) == layout.size, name
    assert bytes.fromhex(output["COBS"]) == p.cobs_encode(sample)
    assert output["ROUND"] == "1"
    assert int(output["CRC"]) == p.crc16(b"123456789")
