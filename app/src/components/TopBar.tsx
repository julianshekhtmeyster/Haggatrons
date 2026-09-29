import { useEffect, useState } from "react";
import type { Api } from "../api";
import type { AppState, Port } from "../types";

interface Props {
  state: AppState;
  api: Api;
  refresh: () => void;
}

export default function TopBar({ state, api, refresh }: Props) {
  const { fleet, mission } = state;
  const [ports, setPorts] = useState<Port[]>([]);
  const [port, setPort] = useState("");
  const [busy, setBusy] = useState(false);

  const loadPorts = async () => {
    const result = await api.get<{ ports: Port[] }>("/api/ports");
    if (result.ok && result.data) {
      setPorts(result.data.ports);
      const preferred = result.data.ports.find((p) => p.espressif) ?? result.data.ports[0];
      setPort((current) => current || preferred?.device || "");
    }
  };
  useEffect(() => {
    loadPorts();
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const run = async (route: string, body?: unknown) => {
    setBusy(true);
    await api.post(route, body);
    setBusy(false);
    refresh();
  };

  const missionActive = mission.state === "running" || mission.state === "paused";
  const master = fleet.master;
  return (
    <header className="topbar">
      <div className="brand">
        <span className="logo">◆</span>
        <div>
          <strong>Haggatrons</strong>
          <small>fleet control</small>
        </div>
      </div>

      <div className="link">
        {master.connected ? (
          <>
            <span className={`dot ${master.configured ? "ok" : "warn"}`} />
            <div className="link-text">
              <strong>{fleet.simulated ? "Simulator" : "Master ESP32"}</strong>
              <small>
                {master.mac ?? "…"} · ch {master.channel ?? "?"} · {master.configured ? "peers configured" : "configuring"}
                {master.host_ok === false ? " · host timeout" : ""}
              </small>
            </div>
            <button disabled={busy || missionActive} onClick={() => run("/api/link/disconnect")}>Disconnect</button>
          </>
        ) : (
          <>
            <span className="dot off" />
            <select value={port} onChange={(e) => setPort(e.target.value)} onFocus={loadPorts}>
              {ports.length === 0 && <option value="">No serial ports</option>}
              {ports.map((p) => (
                <option key={p.device} value={p.device}>
                  {p.device}{p.espressif ? " (ESP32)" : ""}
                </option>
              ))}
            </select>
            <button disabled={busy || !port} onClick={() => run("/api/link/connect", { kind: "serial", port })}>
              Connect master
            </button>
            <button className="ghost" disabled={busy} onClick={() => run("/api/link/connect", { kind: "sim" })}>
              Use simulator
            </button>
            {master.error && <small className="error">{master.error}</small>}
          </>
        )}
      </div>

      <div className="safety">
        <div className={`safety-state ${fleet.estop ? "estop" : fleet.armed ? "armed" : "safe"}`}>
          {fleet.estop ? "E-STOPPED" : fleet.armed ? "ARMED" : "DISARMED"}
        </div>
        {fleet.estop ? (
          <button disabled={busy} onClick={() => run("/api/estop/clear")}>Clear e-stop</button>
        ) : fleet.armed ? (
          <button disabled={busy} onClick={() => run("/api/disarm")}>Disarm</button>
        ) : (
          <button className="arm" disabled={busy || !master.configured} onClick={() => run("/api/arm")}>Arm</button>
        )}
        <button className="estop" title="Emergency stop (Esc)" onClick={() => run("/api/estop", { reason: "button" })}>
          STOP
        </button>
      </div>
    </header>
  );
}
