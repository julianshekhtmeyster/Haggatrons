import { useState } from "react";
import type { Api } from "../api";
import type { AppState, FleetEvent } from "../types";
import { Frame } from "./Trace";
import { ROBOT_COLORS } from "./MapView";

// Operator scan task: write an instruction, let the coordinator interpret it into
// a plan, edit it, and send it to every online robot. Every robot photographs
// every position around the circle; each position is one row in its log.

type AnyEvent = FleetEvent & Record<string, any>;
interface Plan {
  steps_per_rotation: number;
  rotations: number;
  target: string;
  understood_as: string;
}

const OUTCOMES: Record<string, string> = {
  running: "scanning…",
  completed: "completed",
  stopped: "stopped",
  failed: "failed",
};

interface Props {
  state: AppState;
  events: FleetEvent[];
  api: Api;
  refresh: () => void;
}

export default function TaskPanel({ state, events, api, refresh }: Props) {
  const [instruction, setInstruction] = useState("Do 8 rotations and look for a person.");
  const [plan, setPlan] = useState<Plan>({ steps_per_rotation: 8, rotations: 1, target: "person", understood_as: "" });
  const [busy, setBusy] = useState<string | null>(null);
  const task = state.task;
  const running = task?.state === "running" || task?.state === "stopping";
  const online = state.fleet.robots.filter((r) => r.online);
  const positions = plan.steps_per_rotation * plan.rotations;

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
        <textarea rows={2} value={instruction} disabled={running} onChange={(e) => setInstruction(e.target.value)}
          placeholder="e.g. Do 12 rotations and look for a dog." />
        <div className="row">
          <button disabled={running || !!busy || !instruction.trim()} onClick={interpret}>
            {busy === "plan" ? "Interpreting…" : "Interpret with gpt-6-luna"}
          </button>
          <span className="muted small">or set the plan directly</span>
        </div>

        <div className="plan">
          <label>Positions per full turn
            <input type="number" min={2} max={36} value={plan.steps_per_rotation} disabled={running}
              onChange={(e) => setPlan({ ...plan, steps_per_rotation: Number(e.target.value) })} />
          </label>
          <label>Full turns
            <input type="number" min={1} max={3} value={plan.rotations} disabled={running}
              onChange={(e) => setPlan({ ...plan, rotations: Number(e.target.value) })} />
          </label>
          <label className="grow">Look for
            <input type="text" maxLength={100} value={plan.target} disabled={running}
              onChange={(e) => setPlan({ ...plan, target: e.target.value })} />
          </label>
        </div>
        {plan.understood_as && <p className="small understood">Understood as: {plan.understood_as}</p>}
        <p className="small muted">
          Each robot photographs {positions} position{positions === 1 ? "" : "s"}, {(360 / Math.max(2, plan.steps_per_rotation)).toFixed(0)}° apart,
          and checks every photo for “{plan.target || "…"}”. Up to {positions * Math.max(1, online.length)} paid vision calls
          for {online.length} online robot{online.length === 1 ? "" : "s"}.
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
            <h2>Task {start.task_id} · looking for “{start.plan.target}”</h2>
            <span className={`pill ${end ? end.state : "running"}`}>{end ? end.state : task?.state}</span>
          </div>
          {start.skipped_offline?.length > 0 && <p className="small warn">Skipped (offline): robot {start.skipped_offline.join(", ")}</p>}
          {end?.log_dir && <p className="small muted">Logs: {end.log_dir}/robot-N.csv</p>}

          {state.fleet.robots.filter((r) => start.robots.includes(r.id)).map((robot) => {
            const index = state.fleet.robots.findIndex((r) => r.id === robot.id);
            const looks = current.filter((e) => e.kind === "task_step" && e.robot_id === robot.id);
            const turns = current.filter((e) => e.kind === "move_result" && e.robot_id === robot.id && e.source === "task");
            const done = [...current].reverse().find((e) => e.kind === "task_robot" && e.robot_id === robot.id);
            const outcome = done?.outcome ?? "running";
            const total = start.plan.steps_per_rotation * start.plan.rotations;
            const hits = looks.filter((l) => l.detection.target_visible);
            return (
              <article key={robot.id} className="robot-log" style={{ borderLeftColor: ROBOT_COLORS[index % ROBOT_COLORS.length] }}>
                <header>
                  <b>{robot.name}</b>
                  <span className={`outcome ${outcome}`}>{OUTCOMES[outcome] ?? outcome}</span>
                  <span className="muted small">{looks.length}/{total} positions</span>
                  <span className={`found-summary ${hits.length ? "yes" : ""}`}>
                    {hits.length
                      ? `“${start.plan.target}” seen at ${hits.length} position${hits.length === 1 ? "" : "s"}: ${hits.map((h) => `#${h.step + 1} (${h.planned_heading_deg}°)`).join(", ")}`
                      : looks.length ? `“${start.plan.target}” not seen yet` : ""}
                  </span>
                </header>
                {done?.error && <p className="small bad">{done.error}</p>}
                <table>
                  <thead>
                    <tr><th>#</th><th>Heading</th><th>Photo</th><th>{start.plan.target}?</th><th>Where</th><th>Confidence</th><th>What the camera saw</th><th>Then turned</th></tr>
                  </thead>
                  <tbody>
                    {looks.map((look) => {
                      const d = look.detection;
                      const turn = turns.find((t) => t.tick === look.step);
                      return (
                        <tr key={look.id} className={d.target_visible ? "hit" : ""}>
                          <td>{look.step + 1}</td>
                          <td>{look.planned_heading_deg}°</td>
                          <td className="thumb"><Frame frameId={look.frame_id} /></td>
                          <td className={d.target_visible ? "yes" : "no"}>{d.target_visible ? "YES" : "no"}</td>
                          <td>{d.target_visible ? d.target_location : "—"}</td>
                          <td>{d.confidence}</td>
                          <td>
                            {d.target_visible ? d.target_description : d.scene_summary}
                            {d.person_visible && <span className="person-text"> · ⚠ person ({d.person_location})</span>}
                          </td>
                          <td>{look.step + 1 === total ? "— last" : turn ? `${turn.yaw_deg}° (${turn.outcome.replaceAll("_", " ")})` : "turning…"}</td>
                        </tr>
                      );
                    })}
                    {outcome === "running" && looks.length < total && (
                      <tr className="pending"><td>{looks.length + 1}</td><td colSpan={7} className="muted">taking photo…</td></tr>
                    )}
                  </tbody>
                </table>
              </article>
            );
          })}
        </section>
      )}
    </div>
  );
}
