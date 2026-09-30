import { useEffect, useState } from "react";
import type { Api } from "../api";
import type { FleetEvent, FleetState, Robot } from "../types";
import { PersonBadge, VisionBlock } from "./Trace";

interface Props {
  robot: Robot;
  fleet: FleetState;
  missionActive: boolean;
  api: Api;
  refresh: () => void;
  vision?: FleetEvent;
}

const JOG_TURN_DEG = 30;
const JOG_FORWARD_MS = 400;

export default function RobotCard({ robot, fleet, missionActive, api, refresh, vision }: Props) {
  const [image, setImage] = useState<string | null>(null);
  const [busy, setBusy] = useState<string | null>(null);
  const t = robot.telemetry;
  const frameId = robot.last_capture?.frame_id;

  useEffect(() => {
    let cancelled = false;
    if (frameId) window.haggatrons.frame(frameId).then((url) => !cancelled && setImage(url));
    return () => {
      cancelled = true;
    };
  }, [frameId]);

  const act = async (label: string, route: string, body?: unknown) => {
    setBusy(label);
    await api.post(route, body);
    setBusy(null);
    refresh();
  };

  const canMove = fleet.armed && !fleet.estop && robot.online && !missionActive && !busy;
  const speed = fleet.calibration.cruise_speed ?? 450;
  const turn = (deg: number) =>
    act("turn", `/api/robots/${robot.id}/move`, { kind: "turn", left: fleet.calibration.turn_speed ?? 420, turn_deg: deg, duration_ms: 1600 });
  const forward = () =>
    act("forward", `/api/robots/${robot.id}/move`, { kind: "forward", left: speed, right: speed, duration_ms: JOG_FORWARD_MS });

  const linkAge = robot.radio?.last_rx_age_ms;
  // Flag a person only while the analysed frame is the one on screen.
  const seesPerson = Boolean((vision as any)?.decision?.person_visible && (vision as any)?.frame_id === frameId);
  return (
    <article className={`robot ${robot.online ? "" : "offline"} ${robot.moving ? "moving" : ""} ${seesPerson ? "person" : ""}`}>
      <header>
        <span className={`dot ${robot.online ? (t?.link_ok ? "ok" : "warn") : "off"}`} />
        <div>
          <strong>{robot.name}</strong>
          <small>#{robot.id} · {robot.mac}</small>
        </div>
        <span className={`badge ${t?.armed ? "armed" : t?.estop ? "estop" : ""}`}>
          {!robot.online ? "offline" : t?.estop ? "stopped" : t?.armed ? "armed" : "safe"}
        </span>
      </header>

      <div className="camera">
        {image ? <img src={image} alt={`${robot.name} camera`} /> : <div className="placeholder">No frame yet</div>}
        {robot.last_capture?.simulated && <span className="sim-tag">SIM</span>}
        {seesPerson && <PersonBadge location={(vision as any).decision.person_location} />}
      </div>

      <dl className="stats">
        <div><dt>Range</dt><dd>{t?.range_ok && t.range_mm != null ? `${t.range_mm} mm` : "—"}</dd></div>
        <div><dt>IMU</dt><dd className={t?.imu_ok ? "" : "bad"}>{t ? (t.imu_ok ? `${t.gyro_dps[2].toFixed(1)}°/s` : "fault") : "—"}</dd></div>
        <div><dt>Link</dt><dd>{robot.rssi != null ? `${robot.rssi} dBm` : "—"}{linkAge != null ? ` · ${linkAge} ms` : ""}</dd></div>
        <div><dt>Pose</dt><dd>{robot.pose.x.toFixed(2)}, {robot.pose.y.toFixed(2)} m · {robot.pose.heading_deg.toFixed(0)}°</dd></div>
        <div><dt>±</dt><dd>{(robot.pose.sigma_m * 100).toFixed(0)} cm · {robot.pose.sigma_deg.toFixed(0)}°</dd></div>
        <div><dt>Last</dt><dd>{robot.last_move ? robot.last_move.outcome.replaceAll("_", " ") : "—"}</dd></div>
      </dl>

      {vision && <VisionBlock event={vision as never} />}

      <div className="controls">
        <div className="row-buttons">
          <button disabled={!robot.online || missionActive || !!busy}
            onClick={() => act("capture", `/api/robots/${robot.id}/capture`, { framesize: "640x480" })}>
            {busy === "capture" ? "Capturing…" : "Capture"}
          </button>
          <button className="ai-button" title="Capture a frame and run it through gpt-6-luna (one paid call). Never moves the robot."
            disabled={!robot.online || missionActive || !!busy}
            onClick={() => act("analyze", `/api/robots/${robot.id}/analyze`, { framesize: "640x480" })}>
            {busy === "analyze" ? "Analyzing…" : "Analyze"}
          </button>
        </div>
        <div className="jog" title={missionActive ? "Manual moves are disabled during a mission" : fleet.armed ? "" : "Arm the fleet to jog"}>
          <button disabled={!canMove} onClick={() => turn(JOG_TURN_DEG)}>⟲</button>
          <button disabled={!canMove || !t?.range_ok} onClick={forward}>▲</button>
          <button disabled={!canMove} onClick={() => turn(-JOG_TURN_DEG)}>⟳</button>
          <button className="stop" disabled={!robot.online} onClick={() => act("stop", `/api/robots/${robot.id}/stop`)}>■</button>
        </div>
      </div>
    </article>
  );
}
