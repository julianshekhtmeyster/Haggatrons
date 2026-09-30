import { useEffect, useRef, useState } from "react";
import type { FleetEvent, Robot } from "../types";

interface Props {
  events: FleetEvent[];
  robots: Robot[];
}

type Line = { tone: string; title: string; detail?: string };

function describe(event: FleetEvent): Line | null {
  const e = event as Record<string, any>;
  const bot = e.robot_id != null ? `Robot ${e.robot_id}` : "";
  switch (event.kind) {
    case "safety":
      return { tone: e.state === "estop" ? "bad" : e.state === "armed" ? "warn" : "info", title: `Safety: ${String(e.state).replace("_", " ")}`, detail: e.reason };
    case "mission":
      return { tone: e.state === "failed" ? "bad" : "info", title: `Mission ${e.state}`, detail: e.error || (e.summary ? JSON.stringify(e.summary) : e.runner ? `${e.runner}${e.simulated ? " · simulated" : ""}` : undefined) };
    case "link":
      return { tone: e.state === "lost" ? "bad" : "info", title: `Link ${e.state}`, detail: e.link || e.reason };
    case "master":
      return e.state === "log" ? { tone: "muted", title: "Master", detail: e.text } : { tone: "info", title: `Master ${e.state}`, detail: e.mac };
    case "observation":
      return { tone: "muted", title: `${bot} observed`, detail: `range ${e.range_mm ?? "n/a"} mm · pose (${e.pose.x}, ${e.pose.y}, ${e.pose.heading_deg}°)${e.simulated ? " · SIM" : ""}` };
    case "vision":
      if (e.decision.person_visible) {
        return { tone: "person", title: `⚠ ${bot} sees a PERSON (${e.decision.person_location})`, detail: `${e.decision.scene_summary} → ${e.decision.proposed_action}` };
      }
      return { tone: "vision", title: `${bot} vision (${e.model})`, detail: `${e.decision.scene_summary} → proposes ${e.decision.proposed_action} (L ${e.decision.directions.left} · C ${e.decision.directions.center} · R ${e.decision.directions.right})` };
    case "vision_failed":
      return { tone: "warn", title: `${bot} vision failed`, detail: e.message };
    case "coordinator":
      return e.phase === "assigned"
        ? { tone: "coord", title: `Tick ${e.tick}: goals assigned`, detail: (e.goals || []).map((g: any) => `R${g.robot_id} ${g.kind}${g.target ? ` (${g.target.join(", ")})` : ""}`).join(" · ") }
        : { tone: "coord", title: `Coordinator ${e.phase}`, detail: e.coverage ? `${e.coverage.area_m2} m² free` : undefined };
    case "gate":
      return { tone: "coord", title: `Tick ${e.tick}: coordinator gate`, detail: (e.decisions || []).map((d: any) => `R${d.proposal.robot_id} ${d.proposal.action}${d.approved ? " ✓" : ` ✗ ${d.refusal}`}`).join(" · ") };
    case "move_sent":
      return { tone: "muted", title: `${bot} → ${e.command.kind}`, detail: `${e.source}${e.reason ? `: ${e.reason}` : ""}` };
    case "move_result":
      return { tone: e.outcome === "completed" ? "ok" : "warn", title: `${bot} ${e.move_kind} ${String(e.outcome).replaceAll("_", " ")}`, detail: `${e.elapsed_ms} ms · yaw ${e.yaw_deg}° · min range ${e.min_range_mm ?? "n/a"} mm` };
    case "move_refused":
      return { tone: "warn", title: `${bot} move refused`, detail: e.reason };
    case "move_failed":
      return { tone: "bad", title: `${bot} move failed`, detail: e.error };
    case "robot_reject":
      return { tone: "warn", title: `${bot} rejected request`, detail: e.reason };
    case "capture":
      return { tone: "muted", title: `${bot} frame`, detail: `${e.seconds}s over ESP-NOW${e.simulated ? " · SIM" : ""}` };
    case "error":
      return { tone: "bad", title: `Error (${e.source})`, detail: e.error };
    case "worker":
      return { tone: "warn", title: `${bot} worker`, detail: e.message };
    case "flower":
      return String(e.line).startsWith("HAGGATRONS_") || /error|traceback/i.test(String(e.line))
        ? { tone: /error|traceback/i.test(String(e.line)) ? "bad" : "muted", title: "Flower", detail: String(e.line) }
        : null;
    default:
      return null;
  }
}

export default function EventLog({ events, robots }: Props) {
  const [filter, setFilter] = useState<number | "all">("all");
  const listRef = useRef<HTMLOListElement>(null);
  const stick = useRef(true);

  const visible = events
    .filter((e) => filter === "all" || e.robot_id === undefined || e.robot_id === filter)
    .map((e) => ({ event: e, line: describe(e) }))
    .filter((item): item is { event: FleetEvent; line: Line } => item.line !== null)
    .slice(-250);

  useEffect(() => {
    const list = listRef.current;
    if (list && stick.current) list.scrollTop = list.scrollHeight;
  }, [visible.length]);

  return (
    <div className="events">
      <div className="panel-title">
        <h2>Timeline</h2>
        <select value={filter} onChange={(e) => setFilter(e.target.value === "all" ? "all" : Number(e.target.value))}>
          <option value="all">All robots</option>
          {robots.map((r) => <option key={r.id} value={r.id}>{r.name}</option>)}
        </select>
      </div>
      <ol ref={listRef} onScroll={(e) => {
        const el = e.currentTarget;
        stick.current = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
      }}>
        {visible.map(({ event, line }) => (
          <li key={event.id} className={line.tone}>
            <time>{new Date(event.ts * 1000).toLocaleTimeString([], { hour12: false })}</time>
            <div>
              <strong>{line.title}</strong>
              {line.detail && <p>{line.detail}</p>}
            </div>
          </li>
        ))}
      </ol>
    </div>
  );
}
