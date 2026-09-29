import { useEffect, useRef } from "react";
import type { CoordinatorState, FleetState } from "../types";

export const ROBOT_COLORS = ["#4cc9f0", "#f72585", "#b8f35d", "#ffb703"];

interface Props {
  fleet: FleetState;
  coordinator: CoordinatorState | null;
}

export default function MapView({ fleet, coordinator }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    if (!canvas || !wrap) return;
    const ratio = window.devicePixelRatio || 1;
    const width = wrap.clientWidth;
    const height = wrap.clientHeight;
    canvas.width = width * ratio;
    canvas.height = height * ratio;
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
    const ctx = canvas.getContext("2d")!;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.fillStyle = "#0b0f14";
    ctx.fillRect(0, 0, width, height);

    const map = coordinator?.map;
    const cell = map?.cell_m ?? 0.1;
    // Fit the view around everything known, with at least a 3 m × 3 m window.
    const xs: number[] = [0, 3];
    const ys: number[] = [0, 3];
    map?.cells.forEach(([cx, cy]) => {
      xs.push(cx * cell, (cx + 1) * cell);
      ys.push(cy * cell, (cy + 1) * cell);
    });
    fleet.robots.forEach((r) => {
      xs.push(r.pose.x);
      ys.push(r.pose.y);
    });
    const pad = 0.4;
    const minX = Math.min(...xs) - pad;
    const maxX = Math.max(...xs) + pad;
    const minY = Math.min(...ys) - pad;
    const maxY = Math.max(...ys) + pad;
    const scale = Math.min(width / (maxX - minX), height / (maxY - minY));
    const offsetX = (width - (maxX - minX) * scale) / 2;
    const offsetY = (height - (maxY - minY) * scale) / 2;
    const px = (x: number) => offsetX + (x - minX) * scale;
    const py = (y: number) => height - offsetY - (y - minY) * scale; // y up

    // 0.5 m grid
    ctx.strokeStyle = "rgba(255,255,255,0.05)";
    ctx.lineWidth = 1;
    for (let x = Math.ceil(minX * 2) / 2; x <= maxX; x += 0.5) {
      ctx.beginPath();
      ctx.moveTo(px(x), py(minY));
      ctx.lineTo(px(x), py(maxY));
      ctx.stroke();
    }
    for (let y = Math.ceil(minY * 2) / 2; y <= maxY; y += 0.5) {
      ctx.beginPath();
      ctx.moveTo(px(minX), py(y));
      ctx.lineTo(px(maxX), py(y));
      ctx.stroke();
    }

    if (map) {
      for (const [cx, cy, value] of map.cells) {
        const strength = Math.min(1, Math.abs(value) / 3);
        if (value <= map.free_t) ctx.fillStyle = `rgba(90, 200, 190, ${0.25 + 0.5 * strength})`;
        else if (value >= map.blocked_t) ctx.fillStyle = `rgba(255, 120, 70, ${0.4 + 0.6 * strength})`;
        else ctx.fillStyle = value < 0 ? "rgba(90, 200, 190, 0.1)" : "rgba(255, 120, 70, 0.12)";
        ctx.fillRect(px(cx * cell), py((cy + 1) * cell), cell * scale + 0.5, cell * scale + 0.5);
      }
      ctx.fillStyle = "rgba(255, 255, 255, 0.55)";
      for (const [cx, cy] of map.frontiers) {
        ctx.beginPath();
        ctx.arc(px((cx + 0.5) * cell), py((cy + 0.5) * cell), Math.max(1.5, scale * 0.012), 0, Math.PI * 2);
        ctx.fill();
      }
    }

    fleet.robots.forEach((robot, index) => {
      const color = ROBOT_COLORS[index % ROBOT_COLORS.length];
      const x = px(robot.pose.x);
      const y = py(robot.pose.y);
      const goal = coordinator?.goals.find((g) => g.robot_id === robot.id);
      if (goal?.target) {
        const gx = px(goal.target[0]);
        const gy = py(goal.target[1]);
        ctx.strokeStyle = color;
        ctx.setLineDash([6, 5]);
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(gx, gy);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.arc(gx, gy, 9, 0, Math.PI * 2);
        ctx.moveTo(gx - 13, gy);
        ctx.lineTo(gx + 13, gy);
        ctx.moveTo(gx, gy - 13);
        ctx.lineTo(gx, gy + 13);
        ctx.stroke();
      }
      // Position uncertainty
      ctx.fillStyle = `${color}22`;
      ctx.beginPath();
      ctx.arc(x, y, Math.max(6, robot.pose.sigma_m * scale), 0, Math.PI * 2);
      ctx.fill();
      // Robot body, pointing along its heading
      const heading = (robot.pose.heading_deg * Math.PI) / 180;
      const size = Math.max(9, 0.09 * scale);
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(-heading);
      ctx.fillStyle = robot.online ? color : "#555";
      ctx.beginPath();
      ctx.moveTo(size, 0);
      ctx.lineTo(-size * 0.7, size * 0.65);
      ctx.lineTo(-size * 0.7, -size * 0.65);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
      ctx.fillStyle = "#e6edf3";
      ctx.font = "600 12px ui-sans-serif, system-ui";
      ctx.fillText(`${robot.id}`, x + size + 3, y - size);
    });

    // Scale bar
    ctx.strokeStyle = "#8b949e";
    ctx.fillStyle = "#8b949e";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(16, height - 18);
    ctx.lineTo(16 + scale, height - 18);
    ctx.stroke();
    ctx.font = "11px ui-sans-serif, system-ui";
    ctx.fillText("1 m", 16, height - 24);
  });

  const map = coordinator?.map;
  return (
    <div className="map-panel">
      <div className="panel-title">
        <h2>Shared map</h2>
        <span className="muted">
          {map ? `${map.area_m2.toFixed(2)} m² free · ${map.blocked_cells} obstacle cells · tick ${coordinator!.tick}` : "No mission data yet"}
        </span>
      </div>
      <div className="map-wrap" ref={wrapRef}>
        <canvas ref={canvasRef} />
      </div>
      <div className="legend">
        <span><i className="sw free" /> free (range-confirmed)</span>
        <span><i className="sw blocked" /> obstacle</span>
        <span><i className="sw frontier" /> frontier</span>
        <span>◌ pose uncertainty · ⌖ assigned goal</span>
      </div>
      {coordinator && (
        <ul className="goals">
          {coordinator.goals.map((goal) => (
            <li key={goal.robot_id}>
              <b style={{ color: ROBOT_COLORS[fleet.robots.findIndex((r) => r.id === goal.robot_id) % ROBOT_COLORS.length] }}>
                Robot {goal.robot_id}
              </b>{" "}
              {goal.kind}{goal.target ? ` → (${goal.target[0].toFixed(2)}, ${goal.target[1].toFixed(2)})` : ""}
              <span className="muted"> — {goal.reason}</span>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
