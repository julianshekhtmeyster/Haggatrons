import { useState } from "react";
import type { Api } from "../api";
import type { AppState } from "../types";

interface Props {
  state: AppState;
  api: Api;
  refresh: () => void;
}

export default function MissionPanel({ state, api, refresh }: Props) {
  const { mission, fleet } = state;
  const [steps, setSteps] = useState(12);
  const [runner, setRunner] = useState<"flower" | "local">("flower");
  const [vision, setVision] = useState(false);
  const [budget, setBudget] = useState(4);
  const [busy, setBusy] = useState(false);

  const run = async (route: string, body?: unknown) => {
    setBusy(true);
    await api.post(route, body);
    setBusy(false);
    refresh();
  };

  const active = mission.state === "running" || mission.state === "paused";
  const readyReason = fleet.estop
    ? "Clear the e-stop"
    : !fleet.master.configured
      ? "Connect the master or simulator"
      : !fleet.armed
        ? "Arm the fleet to start"
        : fleet.robots.some((r) => !r.online)
          ? `Waiting for ${fleet.robots.filter((r) => !r.online).map((r) => r.name).join(", ")}`
          : null;

  return (
    <div className="mission">
      <div className="panel-title">
        <h2>Mission</h2>
        <span className={`pill ${mission.state}`}>{mission.state}</span>
      </div>
      {active ? (
        <>
          <p className="muted">
            {mission.runner === "flower" ? "Flower federation" : "Local loop"} · {mission.steps} ticks
            {mission.vision ? ` · vision ${mission.vision_used}/${mission.vision_budget} calls` : " · vision off"}
          </p>
          <div className="row">
            {mission.state === "running" ? (
              <button disabled={busy} onClick={() => run("/api/mission/pause")}>Pause</button>
            ) : (
              <button disabled={busy} onClick={() => run("/api/mission/resume")}>Resume</button>
            )}
            <button className="danger" disabled={busy} onClick={() => run("/api/mission/stop")}>Stop mission</button>
          </div>
        </>
      ) : (
        <>
          <div className="form">
            <label>Ticks<input type="number" min={1} max={200} value={steps} onChange={(e) => setSteps(Number(e.target.value))} /></label>
            <label>Runner
              <select value={runner} onChange={(e) => setRunner(e.target.value as "flower" | "local")}>
                <option value="flower">Flower (one worker per robot)</option>
                <option value="local">Local loop (no Flower)</option>
              </select>
            </label>
            <label className="check">
              <input type="checkbox" checked={vision} onChange={(e) => setVision(e.target.checked)} />
              gpt-6-luna vision (paid)
            </label>
            {vision && <label>Call budget<input type="number" min={1} max={50} value={budget} onChange={(e) => setBudget(Number(e.target.value))} /></label>}
          </div>
          <button className="primary" disabled={busy || !!readyReason}
            onClick={() => run("/api/mission/start", { steps, runner, vision, vision_budget: budget })}>
            Start mission
          </button>
          {readyReason && <p className="muted small">{readyReason}.</p>}
          {mission.summary && (
            <p className="muted small">
              Last mission {mission.state}: {mission.summary.ticks} ticks, {mission.summary.area_m2} m² confirmed free.
            </p>
          )}
          {mission.state === "failed" && <p className="error small">See the timeline for the failure.</p>}
        </>
      )}
      <button className="ghost small" disabled={active || busy} onClick={() => run("/api/poses/reset")}>
        Reset poses to start positions
      </button>
    </div>
  );
}
