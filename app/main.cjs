// Electron main process: owns the Python backend and relays its API to the UI.
//
// The renderer never sees the API token. It calls window.haggatrons.api(),
// which this process forwards over IPC with the bearer token attached, and it
// receives the backend's server-sent events as IPC messages.
const { app, BrowserWindow, ipcMain, Menu, dialog } = require("electron");
const { spawn } = require("node:child_process");
const crypto = require("node:crypto");
const fs = require("node:fs");
const path = require("node:path");

const ROOT = path.resolve(__dirname, "..");
const TOKEN = crypto.randomBytes(24).toString("base64url");
const DEV_URL = process.env.VITE_DEV_SERVER_URL;

let backend = null;
let backendUrl = null;
let backendLog = [];
let mainWindow = null;
let eventAbort = null;
let quitting = false;

function pythonExecutable() {
  if (process.env.HAGGATRONS_PYTHON) return process.env.HAGGATRONS_PYTHON;
  const venv = path.join(ROOT, ".venv", "bin", "python");
  return fs.existsSync(venv) ? venv : "python3";
}

function send(channel, payload) {
  if (mainWindow && !mainWindow.isDestroyed()) mainWindow.webContents.send(channel, payload);
}

function startBackend() {
  return new Promise((resolve, reject) => {
    const python = pythonExecutable();
    backend = spawn(python, ["-m", "haggatrons", "serve", "--parent-stdin"], {
      cwd: ROOT,
      env: { ...process.env, HAGGATRONS_TOKEN: TOKEN, PYTHONUNBUFFERED: "1" },
      stdio: ["pipe", "pipe", "pipe"],
    });
    let buffered = "";
    const timer = setTimeout(() => reject(new Error("Backend did not start within 20 s")), 20000);
    const record = (line) => {
      backendLog.push(line);
      if (backendLog.length > 300) backendLog = backendLog.slice(-300);
      send("backend-log", line);
    };
    backend.stdout.on("data", (chunk) => {
      buffered += chunk.toString();
      let index;
      while ((index = buffered.indexOf("\n")) >= 0) {
        const line = buffered.slice(0, index).trim();
        buffered = buffered.slice(index + 1);
        if (line.startsWith("HAGGATRONS_READY ")) {
          backendUrl = JSON.parse(line.slice("HAGGATRONS_READY ".length)).url;
          clearTimeout(timer);
          resolve(backendUrl);
        } else if (line) {
          record(line);
        }
      }
    });
    backend.stderr.on("data", (chunk) => chunk.toString().split("\n").filter(Boolean).forEach(record));
    backend.on("error", (error) => {
      clearTimeout(timer);
      reject(new Error(`Cannot start ${python}: ${error.message}`));
    });
    backend.on("exit", (code) => {
      clearTimeout(timer);
      backendUrl = null;
      send("backend-state", { running: false, code });
      if (!quitting) reject(new Error(`Backend exited with code ${code}`));
    });
  });
}

async function api(method, route, body) {
  if (!backendUrl) return { ok: false, status: 0, error: "Backend is not running" };
  try {
    const response = await fetch(backendUrl + route, {
      method,
      headers: { Authorization: `Bearer ${TOKEN}`, "Content-Type": "application/json" },
      body: method === "POST" ? JSON.stringify(body ?? {}) : undefined,
    });
    const data = await response.json().catch(() => ({}));
    return response.ok ? { ok: true, status: response.status, data } : { ok: false, status: response.status, error: data.error || response.statusText };
  } catch (error) {
    return { ok: false, status: 0, error: String(error.message || error) };
  }
}

async function streamEvents() {
  while (!quitting && backendUrl) {
    eventAbort = new AbortController();
    try {
      const response = await fetch(`${backendUrl}/api/events`, {
        headers: { Authorization: `Bearer ${TOKEN}` },
        signal: eventAbort.signal,
      });
      const decoder = new TextDecoder();
      let buffered = "";
      for await (const chunk of response.body) {
        buffered += decoder.decode(chunk, { stream: true });
        let index;
        while ((index = buffered.indexOf("\n\n")) >= 0) {
          const block = buffered.slice(0, index);
          buffered = buffered.slice(index + 2);
          const data = block.split("\n").filter((l) => l.startsWith("data: ")).map((l) => l.slice(6)).join("");
          if (data) send("event", JSON.parse(data));
        }
      }
    } catch (error) {
      if (quitting) return;
    }
    await new Promise((resolve) => setTimeout(resolve, 1000));
  }
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1500,
    height: 940,
    minWidth: 1100,
    minHeight: 720,
    backgroundColor: "#0d1117",
    title: "Haggatrons Control",
    webPreferences: {
      preload: path.join(__dirname, "preload.cjs"),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true,
    },
  });
  mainWindow.webContents.setWindowOpenHandler(() => ({ action: "deny" }));
  mainWindow.webContents.on("will-navigate", (event) => event.preventDefault());
  if (DEV_URL) mainWindow.loadURL(DEV_URL);
  else mainWindow.loadFile(path.join(__dirname, "dist", "index.html"));
}

function buildMenu() {
  const template = [
    ...(process.platform === "darwin" ? [{ role: "appMenu" }] : []),
    {
      label: "Fleet",
      submenu: [
        { label: "EMERGENCY STOP", accelerator: "CmdOrCtrl+.", click: () => api("POST", "/api/estop", { reason: "menu" }) },
        { label: "Disarm", accelerator: "CmdOrCtrl+D", click: () => api("POST", "/api/disarm") },
      ],
    },
    { role: "editMenu" },
    { role: "viewMenu" },
  ];
  Menu.setApplicationMenu(Menu.buildFromTemplate(template));
}

ipcMain.handle("api", (_event, method, route, body) => {
  if (!["GET", "POST"].includes(method) || typeof route !== "string" || !route.startsWith("/api/")) {
    return { ok: false, status: 400, error: "Invalid request" };
  }
  return api(method, route, body);
});

ipcMain.handle("frame", async (_event, frameId) => {
  if (!backendUrl || !/^[A-Za-z0-9_]+$/.test(frameId)) return null;
  try {
    const response = await fetch(`${backendUrl}/api/frames/${frameId}.jpg`, { headers: { Authorization: `Bearer ${TOKEN}` } });
    if (!response.ok) return null;
    return `data:image/jpeg;base64,${Buffer.from(await response.arrayBuffer()).toString("base64")}`;
  } catch {
    return null;
  }
});

ipcMain.handle("backend-info", () => ({ running: Boolean(backendUrl), url: backendUrl, log: backendLog, python: pythonExecutable() }));

// Smoke test: HAGGATRONS_SMOKE=out.png drives a short simulated mission, saves a
// screenshot of the window, and quits. Never touches hardware.
async function smokeTest(outPath) {
  const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  const step = async (route, body) => {
    const result = await api("POST", route, body);
    if (!result.ok) throw new Error(`${route}: ${result.error}`);
    return result.data;
  };
  await step("/api/link/connect", { kind: "sim" });
  await wait(1500);
  await step("/api/robots/1/capture", { framesize: "640x480" });
  await step("/api/robots/2/capture", { framesize: "640x480" });
  await step("/api/arm");
  await step("/api/mission/start", { steps: Number(process.env.HAGGATRONS_SMOKE_STEPS || 6), runner: process.env.HAGGATRONS_SMOKE_RUNNER || "local" });
  for (let i = 0; i < 240; i += 1) {
    const state = await api("GET", "/api/mission/control");
    if (state.ok && !["running", "paused", "stopping"].includes(state.data.state)) break;
    await wait(500);
  }
  await wait(1200);
  const image = await mainWindow.webContents.capturePage();
  fs.writeFileSync(outPath, image.toPNG());
  const final = await api("GET", "/api/state");
  console.log("SMOKE_RESULT " + JSON.stringify({ mission: final.data?.mission?.state, summary: final.data?.mission?.summary }));
}

app.whenReady().then(async () => {
  buildMenu();
  createWindow();
  try {
    await startBackend();
    send("backend-state", { running: true });
    streamEvents();
    if (process.env.HAGGATRONS_SMOKE) {
      smokeTest(process.env.HAGGATRONS_SMOKE)
        .catch((error) => console.error("SMOKE_FAILED " + error.message))
        .finally(() => app.quit());
    }
  } catch (error) {
    dialog.showErrorBox("Haggatrons backend failed to start",
      `${error.message}\n\nInstall the Python package first:\n  cd ${ROOT}\n  python3 -m venv .venv\n  .venv/bin/pip install -e .\n\n${backendLog.slice(-15).join("\n")}`);
  }
});

app.on("before-quit", async (event) => {
  if (quitting || !backend) return;
  event.preventDefault();
  quitting = true;
  // Leave nothing armed: stop robots, then let the backend exit (it closes the link on stdin EOF).
  await api("POST", "/api/estop", { reason: "control center closed" });
  if (eventAbort) eventAbort.abort();
  backend.stdin.end();
  const forced = setTimeout(() => backend.kill("SIGTERM"), 3000);
  backend.once("exit", () => {
    clearTimeout(forced);
    app.quit();
  });
});

app.on("window-all-closed", () => app.quit());
