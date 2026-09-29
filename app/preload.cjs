// Narrow bridge between the sandboxed UI and the main process.
const { contextBridge, ipcRenderer } = require("electron");

function subscribe(channel, callback) {
  const listener = (_event, payload) => callback(payload);
  ipcRenderer.on(channel, listener);
  return () => ipcRenderer.removeListener(channel, listener);
}

contextBridge.exposeInMainWorld("haggatrons", {
  api: (method, route, body) => ipcRenderer.invoke("api", method, route, body),
  frame: (frameId) => ipcRenderer.invoke("frame", frameId),
  backendInfo: () => ipcRenderer.invoke("backend-info"),
  onEvent: (callback) => subscribe("event", callback),
  onBackendLog: (callback) => subscribe("backend-log", callback),
  onBackendState: (callback) => subscribe("backend-state", callback),
});
