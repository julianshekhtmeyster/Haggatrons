import { useState } from "react";
import type { Api } from "../api";
import type { AppState, FleetEvent } from "../types";
import { Frame } from "./Trace";
import { ROBOT_COLORS } from "./MapView";

// Operator scan task: write an instruction, let the coordinator interpret it into
// a plan, edit it, and send it to every online robot. Each robot reports every
// look-and-turn step back here.

type AnyEvent = FleetEvent & Record<string, any>;
interface Plan {
  steps_per_rotation: number;
  rotations: number;
  target: string;
  stop_when_found: boolean;
  understood_as: string;
}

const OUTCOMES: Record<string, string> = {
  running: "scanning…",
  found: "✓ found",
  not_found: "not found",
  stopped: "stopped",
  stopped_person: "stopped: person in view",
  failed: "failed",
};

interface Props {
  state: AppState;
  events: FleetEvent[];
  api: Api;
  refresh: () => void;
}

export default function TaskPanel({ state, events, api, refresh }: Props) {
  const [instruction, setInstruction] = useState("Turn a full circle in 8 steps and look for a person. Stop when you find one.");
  const [plan, setPlan] = useState<Plan>({ steps_per_rotation: 8, rotations: 1, target: "person", stop_when_found: true, understood_as: "" });
  const [busy, setBusy] = useState<string | null>(null);
  const task = state.task;
  const running = task?.state === "running" || task?.state === "stopping";
  const online = state.fleet.robots.filter((r) => r.online);
  const steps = plan.steps_per_rotation * plan.rotations;

  const interpret = async () => {
    setBusy("plan");
    const result = await api.post<Plan>("/api/task/plan", { instruction });
    if (result.ok && result.data) setPlan(result.data);
    setBusy(null);
  };
  const run = async () => {
    setBusy("run");
    await api.post("/api/task/start", { plan, instruction });
    setBusy(null);
    refresh();
  };

  const all = events as AnyEvent[];
  const start = [...all].reverse().find((e) => e.kind === "task" && e.state === "running");
  const current = start ? all.filter((e) => e.id >= start.id) : [];
  const end = [...current].reverse().find((e) => e.kind === "task" && ["finished", "stopped"].includes(e.state));
  const blocker = state.fleet.estop ? "Clear the e-stop" : !state.fleet.armed ? "Arm the fleet (robots will turn in place)"
    : state.mission.state === "running" || state.mission.state === "paused" ? "Stop the mission first"
      : online.length === 0 ? "No robot is online" : null;

  return (
    <div className="task">
      <section className="task-compose">
        <h2>Coordinator instruction</h2>
        <textarea rows={3} value={instruction} disabled={running} onChange={(e) => setInstruction(e.target.value)}
          placeholder="e.g. Scan two full circles in 12 steps and look for a dog." />
        <div className="row">
          <button disabled={running || !!busy || !instruction.trim()} onClick={interpret}>
            {busy === "plan" ? "Interpreting…" : "Interpret with gpt-6-luna"}
          </button>
          <span className="muted small">or edit the plan directly</span>
        </div>

        <div className="plan">
          <label>Steps per 360°
            <input type="number" min={2} max={36} value={plan.steps_per_rotation} disabled={running}
              onChange={(e) => setPlan({ ...plan, steps_per_rotation: Number(e.target.value) })} />
          </label>
          <label>Rotations
            <input type="number" min={1} max={3} value={plan.rotations} disabled={running}
              onChange={(e) => setPlan({ ...plan, rotations: Number(e.target.value) })} />
          </label>
          <label className="grow">Look for
            <input type="text" maxLength={100} value={plan.target} disabled={running}
              onChange={(e) => setPlan({ ...plan, target: e.target.value })} />
          </label>
          <label className="check">
            <input type="checkbox" checked={plan.stop_when_found} disabled={running}
              onChange={(e) => setPlan({ ...plan, stop_when_found: e.target.checked })} />
            Stop when found
          </label>
        </div>
        {plan.understood_as && <p className="small understood">Understood as: {plan.understood_as}</p>}
        <p className="small muted">
          {(360 / Math.max(2, plan.steps_per_rotation)).toFixed(0)}° per step · {steps} looks per robot ·
          up to {steps * Math.max(1, online.length)} paid vision calls for {online.length} online robot{online.length === 1 ? "" : "s"}.
          A person in view stops that robot unless you are looking for people.
        </p>
        <div className="row">
          {running ? (
            <button className="danger" onClick={() => api.post("/api/task/stop").then(refresh)}>Stop task</button>
          ) : (
            <button className="primary" disabled={!!busy || !!blocker || !plan.target.trim()} onClick={run}>
              {busy === "run" ? "Sending…" : `Send to ${online.length} robot${online.length === 1 ? "" : "s"}`}
            </button>
          )}
        </div>
        {blocker && !running && <p className="small muted">{blocker}.</p>}
      </section>

      {start && (
        <section className="task-results">
          <div className="panel-title">
            <h2>Task {start.task_id}: look for “{start.plan.target}”</h2>
            <span className={`pill ${end ? end.state : "running"}`}>{end ? end.state : task?.state}</span>
          </div>
          {start.skipped_offline?.length > 0 && <p className="small warn">Skipped offline: robot {start.skipped_offline.join(", ")}</p>}
          <div className="task-robots">
            {state.fleet.robots.filter((r) => start.robots.includes(r.id)).map((robot) => {
              const index = state.fleet.robots.findIndex((r) => r.id === robot.id);
              const stepsFor = current.filter((e) => e.kind === "task_step" && e.robot_id === robot.id);
              const turns = current.filter((e) => e.kind === "move_result" && e.robot_id === robot.id && e.source === "task");
              const done = [...current].reverse().find((e) => e.kind === "task_robot" && e.robot_id === robot.id);
              const outcome = done?.outcome ?? "running";
              return (
                <article key={robot.id} className={`task-robot ${outcome}`} style={{ borderTopColor: ROBOT_COLORS[index % ROBOT_COLORS.length] }}>
                  <header>
                    <b>{robot.name}</b>
                    <span className={`outcome ${outcome}`}>{OUTCOMES[outcome] ?? outcome}</span>
                    <span className="muted small">{stepsFor.length}/{start.plan.steps_per_rotation * start.plan.rotations} looks</span>
                  </header>
                  {done?.error && <p className="small bad">{done.error}</p>}
                  {done?.reason && <p className="small warn">{done.reason}</p>}
                  <div className="looks">
                    {stepsFor.map((s) => {
                      const d = s.detection;
                      const turn = turns.find((t) => t.tick === s.step);
                      return (
                        <div key={s.id} className={`look ${d.target_visible ? "hit" : ""} ${d.person_visible ? "person" : ""}`}>
                          <div className="frame-wrap">
                            <Frame frameId={s.frame_id} />
                            <span className="look-step">#{s.step + 1} · {s.heading_turned_deg}°</span>
                            {d.target_visible && <span className="look-hit">FOUND · {d.target_location} · {d.confidence}</span>}
                          </div>
                          <p className="small">{d.target_visible ? d.target_description : d.scene_summary}</p>
                          {d.person_visible && <p className="small person-text">⚠ person ({d.person_location})</p>}
                          <p className="small muted">{s.seconds}s{turn ? ` · then turned ${turn.yaw_deg}° (${turn.outcome.replaceAll("_", " ")})` : ""}</p>
                        </div>
                      );
                    })}
                  </div>
                </article>
              );
            })}
          </div>
        </section>
      )}
    </div>
  );
}
