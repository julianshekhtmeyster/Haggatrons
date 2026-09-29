# ESP-NOW devices and link

ESP-NOW addresses each board by its Wi-Fi station MAC, which is the base MAC reported by `esptool read_mac`. The canonical list is `config/fleet.json`. `python -m haggatrons provision master` writes the master's MAC there.

| Board | Chip | MAC (station) | Read on |
| --- | --- | --- | --- |
| Master | ESP32-S3 (QFN56) rev v0.2, 8 MB PSRAM | `44:bd:8d:eb:de:b0` | 2026-09-29, `/dev/cu.usbmodem101` |
| Slave 1 (robot 1) | ESP32-S3 (QFN56) rev v0.2, 8 MB PSRAM | `44:bd:8d:eb:e1:dc` | 2026-09-29, `/dev/cu.usbmodem101` |
| Slave 2 (robot 2) | ESP32-S3 (QFN56) rev v0.2, 8 MB PSRAM | `44:bd:8d:eb:e3:94` | 2026-09-29, `/dev/cu.usbmodem101` (replacement board) |
| Retired (old Slave 2) | ESP32-S3, camera sends no image data | `44:bd:8d:eb:de:00` | Not in the fleet; its old key was rotated |

## How the link works

```
Chrome control page ⇄ HTTP (localhost) ⇄ Python backend ⇄ USB serial ⇄ master ESP32 ~~ESP-NOW (encrypted)~~ robots
                        │
                        └─ Flower ServerApp + one ClientApp per robot (via the local API)
```

- **Serial (Mac ⇄ master):** COBS-framed `[type][payload][CRC-16]`. The master stores nothing. On connect, the Mac sends the channel, the primary master key (PMK), and each robot's MAC and local master key (LMK).
- **ESP-NOW (master ⇄ robots):** encrypted unicast, v2 payloads up to 1470 bytes. Each robot accepts messages only from the master's MAC.
- **Heartbeats:** the Mac sends a keepalive every 100 ms. The master sends each robot a heartbeat every 100 ms carrying the arm and e-stop state. If the Mac goes quiet for 500 ms, the master e-stops by itself; if a robot hears nothing for 400 ms, it stops its motors.
- **Frames:** a JPEG is sent as numbered 1400-byte chunks. The Mac asks the robot to resend any missing chunks.
- **Keys:** `runs/fleet_keys.json` (Git-ignored, owner-only permissions) is generated on first use. `provision robot` writes the robot's key over USB.

The protocol is defined once, in `firmware/libraries/HaggatronsProtocol/src/HaggatronsProtocol.h`. Its Python mirror is `haggatrons/protocol.py`; `tests/test_protocol.py` compiles the header and checks that the two match.
