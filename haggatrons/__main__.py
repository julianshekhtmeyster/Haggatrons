"""Command line: python -m haggatrons <command>.

serve       run the control backend (the Electron app starts this for you)
mission     headless mission against the simulator or hardware, prints a summary
ports       list serial ports
provision   record the master's MAC, or write ESP-NOW settings to a robot over USB
flash       compile and upload firmware with arduino-cli
"""

from __future__ import annotations

import argparse
import json
import os
import secrets
import signal
import subprocess
import sys
import threading
import time

from haggatrons import protocol as p
from haggatrons.config import ROOT, load_fleet, load_keys, save_master_mac
from haggatrons.events import EventBus
from haggatrons.fleet import FleetController
from haggatrons.link import SerialLink, list_serial_ports
from haggatrons.server import ApiServer, Backend

FQBN = "esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc"


def build_backend(port: int = 0, token: str | None = None, sim_drop: float = 0.0) -> ApiServer:
    from haggatrons.simlink import SimLink

    fleet = load_fleet()
    keys = load_keys(fleet)
    events = EventBus()
    controller = FleetController(fleet, keys, events)
    backend = Backend(controller, events, token or secrets.token_urlsafe(24),
                      lambda: SimLink(fleet, chunk_drop=sim_drop))
    return ApiServer(backend, port=port)


def cmd_serve(args: argparse.Namespace) -> None:
    token = os.environ.get("HAGGATRONS_TOKEN") or secrets.token_urlsafe(24)
    server = build_backend(args.port, token)
    server.start()
    if args.sim:
        server.backend.fleet.connect(server.backend.make_sim_link())
    elif args.serial:
        server.backend.fleet.connect(SerialLink(args.serial))
    print("HAGGATRONS_READY " + json.dumps({"url": server.url}), flush=True)
    if not os.environ.get("HAGGATRONS_TOKEN"):
        print(f"Token (set HAGGATRONS_TOKEN to choose your own): {token}", flush=True)
    done = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: done.set())
    # Exit when the parent (Electron) closes our stdin, so robots are never left armed.
    if args.parent_stdin:
        threading.Thread(target=lambda: (sys.stdin.read(), done.set()), daemon=True).start()
    done.wait()
    server.close()


def cmd_mission(args: argparse.Namespace) -> None:
    from haggatrons.client import BackendClient

    server = build_backend(sim_drop=args.drop)
    server.start()
    fleet = server.backend.fleet
    api = BackendClient(server.url, server.backend.token)
    try:
        api.post("/api/link/connect", {"kind": "sim"} if args.sim else {"kind": "serial", "port": args.serial})
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not all(r.online() for r in fleet.robots.values()):
            time.sleep(0.1)
        offline = [r.name for r in fleet.robots.values() if not r.online()]
        if offline:
            raise SystemExit(f"No telemetry from: {', '.join(offline)}")
        api.post("/api/arm")
        api.post("/api/mission/start", {"steps": args.steps, "runner": args.runner,
                                        "vision": args.vision, "vision_budget": args.vision_budget})
        subscriber = server.backend.events.subscribe(replay=False)
        while True:
            event = subscriber.get()
            if event["kind"] in ("move_result", "move_refused", "robot_reject", "coordinator", "mission",
                                 "vision", "error") or (event["kind"] == "flower" and args.verbose):
                print(_describe(event), flush=True)
            if event["kind"] == "mission" and event["state"] in ("finished", "failed", "stopped"):
                break
        state = server.backend.mission.snapshot()
        print("MISSION_SUMMARY " + json.dumps({"state": event["state"], "summary": state["summary"],
                                               "log_dir": state["log_dir"], "error": event.get("error")}))
        if event["state"] != "finished":
            raise SystemExit(1)
    finally:
        server.close()


def _describe(event: dict) -> str:
    kind = event["kind"]
    if kind == "move_result":
        c = event["command"]
        return (f"  bot{event['robot_id']} {c['kind']:<7} → {event['outcome']:<18} "
                f"pose=({event['pose_after']['x']:.2f}, {event['pose_after']['y']:.2f}, "
                f"{event['pose_after']['heading_deg']:.0f}°)")
    if kind == "coordinator":
        goals = ", ".join(f"bot{g['robot_id']}:{g['kind']}" + (f"@{g['target']}" if g["target"] else "")
                          for g in event.get("goals") or [])
        return f"tick {event['tick']} [{event['phase']}] {goals} coverage={event['coverage']}"
    if kind == "flower":
        return f"  flower | {event['line']}"
    rest = {k: v for k, v in event.items() if k not in ("id", "ts", "kind")}
    return f"  {kind}: {json.dumps(rest, default=str)[:300]}"


def cmd_ports(_: argparse.Namespace) -> None:
    for port in list_serial_ports():
        tag = " (Espressif)" if port["espressif"] else ""
        print(f"{port['device']}\t{port['description']}{tag}")


def _read_master_hello(port: str) -> dict:
    hello: dict = {}
    got = threading.Event()

    def on_frame(frame_type: int, payload: bytes) -> None:
        if frame_type == p.SER_HELLO:
            proto, major, minor, mac, max_payload = p.SER_HELLO_S.unpack(payload)
            hello.update(protocol=proto, firmware=f"{major}.{minor}", mac=p.format_mac(mac),
                         max_payload=max_payload)
            got.set()

    link = SerialLink(port)
    link.start(on_frame, lambda reason: got.set())
    try:
        for _ in range(10):
            link.send(p.SER_HELLO_REQ)
            if got.wait(0.5):
                break
    finally:
        link.close()
    if not hello:
        raise SystemExit(f"No HELLO from a Haggatrons master on {port}. Is firmware/master flashed?")
    return hello


def _robot_status(serial_port) -> str:
    serial_port.reset_input_buffer()
    serial_port.write(b"S")
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        line = serial_port.readline().decode("ascii", "replace").strip()
        if line.startswith("HGBOT "):
            return line
    raise SystemExit("No HGBOT status line. Is firmware/robot flashed?")


def _field(line: str, name: str) -> str:
    for part in line.split():
        if part.startswith(name + "="):
            return part.split("=", 1)[1]
    return ""


def cmd_provision(args: argparse.Namespace) -> None:
    fleet = load_fleet()
    if args.role == "master":
        hello = _read_master_hello(args.port)
        save_master_mac(hello["mac"])
        print(f"Master {hello['mac']} (firmware {hello['firmware']}, ESP-NOW payload "
              f"{hello['max_payload']} B) saved to config/fleet.json")
        return

    import serial

    if fleet.master_mac is None:
        raise SystemExit("Provision the master first: python -m haggatrons provision master --port PORT")
    robot = fleet.robot(args.id)
    keys = load_keys(fleet)
    record = p.Provision(robot.id, fleet.master_mac, fleet.channel, keys.pmk, keys.lmk[robot.id],
                         robot.motor_flags, robot.yaw_axis, robot.yaw_sign, fleet.calibration.max_speed)
    with serial.Serial(args.port, 921600, timeout=1, write_timeout=2) as port:
        time.sleep(1.5)
        status = _robot_status(port)
        mac = _field(status, "mac")
        if mac != p.format_mac(robot.mac) and not args.force:
            raise SystemExit(f"Board MAC {mac} does not match {robot.name} ({p.format_mac(robot.mac)}) in "
                             f"config/fleet.json. Plug in the right board, or pass --force after updating it.")
        port.reset_input_buffer()
        port.write(record.pack())
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            line = port.readline().decode("ascii", "replace").strip()
            if line == "PROV OK":
                break
            if line.startswith("ERR"):
                raise SystemExit(line)
        else:
            raise SystemExit("Robot did not confirm provisioning")
    time.sleep(3)  # the robot restarts to apply its radio settings
    with serial.Serial(args.port, 921600, timeout=1) as port:
        time.sleep(1.5)
        status = _robot_status(port)
    print(status)
    if _field(status, "provisioned") != "1" or _field(status, "espnow") != "1":
        raise SystemExit("Robot restarted but ESP-NOW is not ready")
    print(f"{robot.name} provisioned: id {robot.id}, master {p.format_mac(fleet.master_mac)}, "
          f"channel {fleet.channel}. Keys stay in runs/fleet_keys.json.")


def cmd_flash(args: argparse.Namespace) -> None:
    sketch = ROOT / "firmware" / args.sketch
    build = sketch / "build"
    subprocess.run(["arduino-cli", "compile", "--fqbn", FQBN, "--libraries", str(ROOT / "firmware" / "libraries"),
                    "--build-path", str(build), str(sketch)], check=True)
    if args.port:
        subprocess.run(["arduino-cli", "upload", "--fqbn", FQBN + ",UploadSpeed=921600", "--port", args.port,
                        "--build-path", str(build), str(sketch)], check=True)


def main() -> None:
    parser = argparse.ArgumentParser(prog="python -m haggatrons", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    serve = sub.add_parser("serve", help="run the control backend")
    serve.add_argument("--port", type=int, default=0, help="API port (default: any free port)")
    link = serve.add_mutually_exclusive_group()
    link.add_argument("--sim", action="store_true", help="connect the simulated master at startup")
    link.add_argument("--serial", help="connect the master on this serial port at startup")
    serve.add_argument("--parent-stdin", action="store_true", help="exit when stdin closes")
    serve.set_defaults(func=cmd_serve)

    mission = sub.add_parser("mission", help="run one headless mission and print a summary")
    source = mission.add_mutually_exclusive_group(required=True)
    source.add_argument("--sim", action="store_true")
    source.add_argument("--serial")
    mission.add_argument("--steps", type=int, default=8)
    mission.add_argument("--runner", choices=("flower", "local"), default="flower")
    mission.add_argument("--vision", action="store_true", help="allow gpt-6-luna calls (paid)")
    mission.add_argument("--vision-budget", type=int, default=2)
    mission.add_argument("--drop", type=float, default=0.0, help="sim: drop this fraction of frame chunks")
    mission.add_argument("--verbose", action="store_true", help="also print Flower output")
    mission.set_defaults(func=cmd_mission)

    sub.add_parser("ports", help="list serial ports").set_defaults(func=cmd_ports)

    provision = sub.add_parser("provision", help="set up the master or a robot over USB")
    provision.add_argument("role", choices=("master", "robot"))
    provision.add_argument("--port", required=True)
    provision.add_argument("--id", type=int, help="robot id from config/fleet.json")
    provision.add_argument("--force", action="store_true", help="skip the MAC check")
    provision.set_defaults(func=cmd_provision)

    flash = sub.add_parser("flash", help="compile (and upload with --port) firmware")
    flash.add_argument("sketch", choices=("master", "robot", "usb_camera"))
    flash.add_argument("--port")
    flash.set_defaults(func=cmd_flash)

    args = parser.parse_args()
    if args.command == "provision" and args.role == "robot" and args.id is None:
        parser.error("provision robot needs --id")
    args.func(args)


if __name__ == "__main__":
    main()
