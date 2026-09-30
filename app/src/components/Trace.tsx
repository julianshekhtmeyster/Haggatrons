import { useEffect, useState } from "react";
import type { FleetEvent, Robot } from "../types";
import { ROBOT_COLORS } from "./MapView";

// Per-tick decision trace for the latest mission: what each robot saw, what the
// vision model concluded, what the coordinator assigned and why, what the worker
// proposed, the gate's verdict, and what the robot actually did.

type AnyEvent = FleetEvent & Record<string, any>;

export function Frame({ frameId, className }: { frameId?: string; className?: string }) {
  const [url, setUrl] = useState<string | null>(null);
  useEffect(() => {
    let live = true;
    if (frameId) window.haggatrons.frame(frameId).then((u) => live && setUrl(u));
    return () => {
      live = false;
    };
  }, [frameId]);
  if (!frameId) return <div className={`frame empty ${className ?? ""}`}>no frame</div>;
  return url ? <img className={`frame ${className ?? ""}`} src={url} alt={frameId} /> : <div className={`frame ${className ?? ""}`} />;
}

export function VisionBlock({ event }: { event: AnyEvent }) {
  const d = event.decision;
  return (
    <div className="vision-block">
      <div className="vision-head">
        <span className="tag ai">{event.model}</span>
        <span className="muted">{event.seconds}s · {event.usage?.total_tokens ?? "?"} tokens{event.source === "manual" ? " · manual" : ""}</span>
      </div>
      <p className="scene">{d.scene_summary}</p>
      <div className="dirs">
        {(["left", "center", "right"] as const).map((side) => (
          <span key={side} className={`dir ${d.directions[side]}`}>{side[0].toUpperCase()} {d.directions[side]}</span>
        ))}
      </div>
      {d.visible_landmarks?.length > 0 && <p className="small"><b>Sees:</b> {d.visible_landmarks.join(", ")}</p>}
      {d.visual_hazards?.length > 0 && <p className="small hazard"><b>Hazards:</b> {d.visual_hazards.join(", ")}</p>}
      <p className="small"><b>Suggests:</b> {d.proposed_action.replaceAll("_", " ")} — {d.action_reason}</p>
      {d.uncertainty && <p className="small muted"><b>Uncertain:</b> {d.uncertainty}</p>}
    </div>
  );
}

interface Props {
  events: FleetEvent[];
  robots: Robot[];
}

export default function Trace({ events, robots }: Props) {
  const all = events as AnyEvent[];
  const start = [...all].reverse().find((e) => e.kind === "mission" && e.state === "running" && !e.resumed);
  if (!start) {
    return <div className="trace empty-state">Start a mission to see each tick's observations, vision output, goals, gate decisions, and moves.</div>;
  }
  const mission = all.filter((e) => e.id >= start.id);
  const ticks = [...new Set(mission.filter((e) => typeof e.tick === "number").map((e) => e.tick as number))].sort((a, b) => b - a);
  const end = [...mission].reverse().find((e) => e.kind === "mission" && ["finished", "failed", "stopped"].includes(e.state));

  return (
    <div className="trace">
      <div className="trace-meta muted">
        Mission {start.mission_id} · {start.runner}{start.simulated ? " · SIMULATED" : " · hardware"} · vision {start.vision ? "on" : "off"}
        {end && ` · ${end.state}${end.error ? `: ${end.error}` : ""}`}
      </div>
      {ticks.length === 0 && <p className="muted">Waiting for the first tick…</p>}
      {ticks.map((tick) => {
        const at = mission.filter((e) => e.tick === tick);
        const assigned = at.find((e) => e.kind === "coordinator" && e.phase === "assigned");
        const gate = at.find((e) => e.kind === "gate");
        return (
          <section key={tick} className="tick">
            <header>
              <b>Tick {tick}</b>
              {assigned?.coverage && <span className="muted">shared map: {assigned.coverage.area_m2} m² free, {assigned.coverage.blocked_cells} obstacle cells</span>}
            </header>
            <div className="tick-robots">
              {robots.map((robot, index) => {
                const mine = (kind: string) => at.filter((e) => e.kind === kind && e.robot_id === robot.id);
                const obs = mine("observation")[0];
                const vision = mine("vision")[0];
                const visionFailed = mine("vision_failed")[0];
                const goal = assigned?.goals?.find((g: any) => g.robot_id === robot.id);
                const decision = gate?.decisions?.find((d: any) => d.proposal.robot_id === robot.id);
                const result = mine("move_result")[0];
                const refused = mine("move_refused")[0] || mine("move_failed")[0] || mine("robot_reject")[0];
                const held = mine("held")[0];
                const worker = mine("worker")[0];
                return (
                  <article key={robot.id} className="step" style={{ borderTopColor: ROBOT_COLORS[index % ROBOT_COLORS.length] }}>
                    <h3>{robot.name}</h3>
                    <Frame frameId={obs?.frame_id} />
                    <p className="small muted">
                      {obs ? `range ${obs.range_mm ?? "n/a"} mm · pose (${obs.pose.x}, ${obs.pose.y}) ${obs.pose.heading_deg}°` : worker ? worker.message : "no observation"}
                    </p>

                    <div className="stage"><span className="label">Vision</span>
                      {vision ? <VisionBlock event={vision} /> : visionFailed ? <p className="small bad">failed: {visionFailed.message}</p> : <p className="small muted">not used this tick</p>}
                    </div>
                    <div className="stage"><span className="label">Coordinator goal</span>
                      {goal ? <p className="small"><b>{goal.kind}{goal.target ? ` → (${goal.target.join(", ")})` : ""}</b> — {goal.reason}</p> : <p className="small muted">—</p>}
                    </div>
                    <div className="stage"><span className="label">Worker proposal</span>
                      {decision ? (
                        <p className="small"><b>{decision.proposal.action}{decision.proposal.action === "turn" ? ` ${decision.proposal.turn_deg}°` : decision.proposal.action === "forward" ? ` ${decision.proposal.distance_m} m` : ""}</b> — {decision.proposal.reason}</p>
                      ) : <p className="small muted">—</p>}
                    </div>
                    <div className="stage"><span className="label">Coordinator gate</span>
                      {decision ? <p className={`small ${decision.approved ? "ok" : "bad"}`}>{decision.approved ? "✓ approved" : `✗ refused: ${decision.refusal}`}</p> : <p className="small muted">—</p>}
                    </div>
                    <div className="stage"><span className="label">Robot</span>
                      {result ? (
                        <p className={`small ${result.outcome === "completed" ? "ok" : "warn"}`}>
                          {result.move_kind} {result.outcome.replaceAll("_", " ")} · {result.elapsed_ms} ms · yaw {result.yaw_deg}°{result.min_range_mm != null ? ` · closest ${result.min_range_mm} mm` : ""}
                        </p>
                      ) : refused ? <p className="small bad">{refused.reason || refused.error}</p>
                        : held ? <p className="small muted">held — {held.reason}</p> : <p className="small muted">—</p>}
                    </div>
                  </article>
                );
              })}
            </div>
          </section>
        );
      })}
    </div>
  );
}
