import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { makeApi } from "./api";
import type { AppState, FleetEvent } from "./types";
import TopBar from "./components/TopBar";
import RobotCard from "./components/RobotCard";
import MapView from "./components/MapView";
import MissionPanel from "./components/MissionPanel";
import EventLog from "./components/EventLog";
import Trace from "./components/Trace";
import TaskPanel from "./components/TaskPanel";

const MAX_EVENTS = 600;

export default function App() {
  const [state, setState] = useState<AppState | null>(null);
  const [events, setEvents] = useState<FleetEvent[]>([]);
  const [toasts, setToasts] = useState<{ id: number; text: string }[]>([]);
  const [backend, setBackend] = useState<{ running: boolean; log: string[] }>({ running: false, log: [] });
  const toastId = useRef(0);
  const [view, setView] = useState<"map" | "trace" | "task">("map");

  const notify = useCallback((text: string) => {
    const id = ++toastId.current;
    setToasts((current) => [...current.slice(-3), { id, text }]);
    setTimeout(() => setToasts((current) => current.filter((toast) => toast.id !== id)), 6000);
  }, []);
  const api = useMemo(() => makeApi(notify), [notify]);

  const refresh = useCallback(async () => {
    const result = await api.get<AppState>("/api/state");
    if (result.ok && result.data) {
      setState(result.data);
      setEvents((current) => (current.length ? current : result.data!.events));
    }
  }, [api]);

  useEffect(() => {
    window.haggatrons.backendInfo().then((info) => setBackend({ running: info.running, log: info.log }));
    const offState = window.haggatrons.onBackendState((s) => setBackend((b) => ({ ...b, running: s.running })));
    const offLog = window.haggatrons.onBackendLog((line) => setBackend((b) => ({ ...b, log: [...b.log.slice(-200), line] })));
    const offEvent = window.haggatrons.onEvent((event) => {
      if (event.kind === "telemetry") return; // polled state carries telemetry
      setEvents((current) => (current.some((e) => e.id === event.id) ? current : [...current.slice(-MAX_EVENTS), event]));
      if (["safety", "mission", "link", "coordinator", "move_result", "capture", "vision", "task", "task_robot"].includes(event.kind)) refresh();
      if (event.kind === "mission" && event.state === "running" && !event.resumed) setView("trace");
    });
    const timer = setInterval(refresh, 500);
    refresh();
    return () => {
      offState();
      offLog();
      offEvent();
      clearInterval(timer);
    };
  }, [refresh]);

  // Escape is the keyboard e-stop.
  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      if (event.key === "Escape") api.post("/api/estop", { reason: "keyboard" }).then(refresh);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [api, refresh]);

  if (!state) {
    return (
      <div className="boot">
        <h1>Haggatrons Control</h1>
        <p>{backend.running ? "Connecting to backend…" : "Backend not reachable."}</p>
        <pre>{backend.log.slice(-12).join("\n")}</pre>
      </div>
    );
  }

  const missionActive = state.mission.state === "running" || state.mission.state === "paused"
    || state.task.state === "running" || state.task.state === "stopping";
  return (
    <div className={`app ${state.fleet.estop ? "estopped" : state.fleet.armed ? "armed" : ""}`}>
      <TopBar state={state} api={api} refresh={refresh} />
      <main className="layout">
        <section className="fleet">
          {state.fleet.robots.map((robot) => (
            <RobotCard key={robot.id} robot={robot} fleet={state.fleet} missionActive={missionActive}
              api={api} refresh={refresh}
              vision={[...events].reverse().find((e) => e.kind === "vision" && e.robot_id === robot.id)} />
          ))}
        </section>
        <section className="center">
          <div className="tabs">
            <button className={view === "map" ? "active" : ""} onClick={() => setView("map")}>Shared map</button>
            <button className={view === "trace" ? "active" : ""} onClick={() => setView("trace")}>AI trace</button>
            <button className={view === "task" ? "active" : ""} onClick={() => setView("task")}>Task</button>
          </div>
          {view === "map" && <MapView fleet={state.fleet} coordinator={state.coordinator} />}
          {view === "trace" && <Trace events={events} robots={state.fleet.robots} />}
          {view === "task" && <TaskPanel state={state} events={events} api={api} refresh={refresh} />}
        </section>
        <section className="side">
          <MissionPanel state={state} api={api} refresh={refresh} />
          <EventLog events={events} robots={state.fleet.robots} />
        </section>
      </main>
      <div className="toasts">
        {toasts.map((toast) => <div key={toast.id} className="toast">{toast.text}</div>)}
      </div>
    </div>
  );
}
