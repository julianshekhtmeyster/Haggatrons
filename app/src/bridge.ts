// Browser bridge to the Python backend (which owns the USB serial link to the master).
//
// `python -m haggatrons serve` opens Chrome at http://127.0.0.1:8765/#token=…; the
// token is kept in sessionStorage and sent as a bearer header. EventSource and
// <img> cannot set headers, so those two pass it as a query parameter.
import type { ApiResult, FleetEvent } from "./types";

function readToken(): string {
  const match = window.location.hash.match(/token=([^&]+)/);
  if (match) {
    sessionStorage.setItem("haggatrons-token", decodeURIComponent(match[1]));
    history.replaceState(null, "", window.location.pathname);  // keep the token out of the address bar
  }
  return sessionStorage.getItem("haggatrons-token") ?? "";
}

const token = readToken();
const withToken = (route: string) => `${route}${route.includes("?") ? "&" : "?"}token=${encodeURIComponent(token)}`;

type Listener<T> = (value: T) => void;
const eventListeners = new Set<Listener<FleetEvent>>();
const stateListeners = new Set<Listener<{ running: boolean }>>();
let source: EventSource | null = null;

function ensureStream() {
  if (source || !token) return;
  source = new EventSource(withToken("/api/events"));
  source.onopen = () => stateListeners.forEach((listener) => listener({ running: true }));
  source.onerror = () => stateListeners.forEach((listener) => listener({ running: false }));
  source.onmessage = (message) => {
    const event = JSON.parse(message.data) as FleetEvent;
    eventListeners.forEach((listener) => listener(event));
  };
}

export const bridge: Window["haggatrons"] = {
  async api<T>(method: "GET" | "POST", route: string, body?: unknown): Promise<ApiResult<T>> {
    if (!token) return { ok: false, status: 401, error: "Open the control page from the link printed by `python -m haggatrons serve`" };
    try {
      const response = await fetch(route, {
        method,
        headers: { Authorization: `Bearer ${token}`, "Content-Type": "application/json" },
        body: method === "POST" ? JSON.stringify(body ?? {}) : undefined,
      });
      const data = await response.json().catch(() => ({}));
      return response.ok
        ? { ok: true, status: response.status, data: data as T }
        : { ok: false, status: response.status, error: (data as { error?: string }).error || response.statusText };
    } catch (error) {
      return { ok: false, status: 0, error: `Backend unreachable: ${String(error)}` };
    }
  },
  async frame(frameId: string) {
    return /^[A-Za-z0-9_]+$/.test(frameId) ? withToken(`/api/frames/${frameId}.jpg`) : null;
  },
  async backendInfo() {
    return { running: Boolean(token), url: window.location.origin, log: token ? [] : ["No access token: open the link printed by `python -m haggatrons serve`."], python: "" };
  },
  onEvent(callback) {
    eventListeners.add(callback);
    ensureStream();
    return () => eventListeners.delete(callback);
  },
  onBackendLog() {
    return () => undefined;
  },
  onBackendState(callback) {
    stateListeners.add(callback);
    return () => stateListeners.delete(callback);
  },
};
