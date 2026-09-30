// Shapes of the backend's JSON (haggatrons/server.py, fleet.py, mission.py).

export interface Pose {
  x: number;
  y: number;
  heading_deg: number;
  sigma_m: number;
  sigma_deg: number;
}

export interface Telemetry {
  uptime_ms: number;
  armed: boolean;
  estop: boolean;
  link_ok: boolean;
  imu_ok: boolean;
  range_ok: boolean;
  moving: boolean;
  camera_ok: boolean;
  heartbeat_age_ms: number;
  accel_g: number[];
  gyro_dps: number[];
  range_mm: number | null;
  last_outcome: string;
  firmware: string;
}

export interface CaptureInfo {
  frame_id: string;
  width: number;
  height: number;
  captured_at: number;
  range_mm: number | null;
  imu_valid: boolean;
  simulated: boolean;
  pose: Pose;
}

export interface MoveInfo {
  outcome: string;
  move_kind?: string;
  elapsed_ms?: number;
  yaw_deg?: number;
  source?: string;
  error?: string;
  command?: { kind: string };
}

export interface Robot {
  id: number;
  name: string;
  mac: string;
  online: boolean;
  pose: Pose;
  telemetry: Telemetry | null;
  rssi: number | null;
  radio: { last_rx_age_ms: number | null; tx_ok: number; tx_fail: number } | null;
  last_capture: CaptureInfo | null;
  last_move: MoveInfo | null;
  moving: boolean;
}

export interface MasterInfo {
  connected: boolean;
  configured: boolean;
  link?: string;
  mac?: string;
  firmware?: string;
  channel?: number;
  host_ok?: boolean;
  error?: string;
}

export interface FleetState {
  master: MasterInfo;
  armed: boolean;
  estop: boolean;
  simulated: boolean;
  calibration: Record<string, number>;
  robots: Robot[];
}

export interface MissionState {
  state: "idle" | "running" | "paused" | "stopping" | "finished" | "stopped" | "failed";
  id: string | null;
  steps: number;
  runner: string;
  vision: boolean;
  vision_budget: number;
  vision_used: number;
  started_at: number | null;
  summary: Record<string, number> | null;
  log_dir: string | null;
}

export interface Goal {
  robot_id: number;
  kind: "frontier" | "scan" | "hold";
  target: [number, number] | null;
  reason: string;
  score: number;
}

export interface CoordinatorState {
  tick: number;
  phase: string;
  goals: Goal[];
  map: {
    cell_m: number;
    free_t: number;
    blocked_t: number;
    cells: [number, number, number][];
    frontiers: [number, number][];
    free_cells: number;
    blocked_cells: number;
    area_m2: number;
  };
}

export interface TaskState {
  state: "idle" | "running" | "stopping" | "finished" | "stopped";
  id: string | null;
  plan: { steps_per_rotation: number; rotations: number; target: string; stop_when_found: boolean } | null;
  instruction: string;
  robots: Record<string, { state: string; step: number; found: unknown }>;
}

export interface AppState {
  fleet: FleetState;
  mission: MissionState;
  task: TaskState;
  coordinator: CoordinatorState | null;
  events: FleetEvent[];
}

export interface FleetEvent {
  id: number;
  ts: number;
  kind: string;
  robot_id?: number;
  [key: string]: unknown;
}

export interface Port {
  device: string;
  description: string;
  espressif: boolean;
}

export interface ApiResult<T> {
  ok: boolean;
  status: number;
  data?: T;
  error?: string;
}

declare global {
  interface Window {
    haggatrons: {
      api: <T = unknown>(method: "GET" | "POST", route: string, body?: unknown) => Promise<ApiResult<T>>;
      frame: (frameId: string) => Promise<string | null>;
      backendInfo: () => Promise<{ running: boolean; url: string | null; log: string[]; python: string }>;
      onEvent: (callback: (event: FleetEvent) => void) => () => void;
      onBackendLog: (callback: (line: string) => void) => () => void;
      onBackendState: (callback: (state: { running: boolean; code?: number }) => void) => () => void;
    };
  }
}
