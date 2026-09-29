// Starts the Vite dev server, then Electron pointed at it.
import { spawn } from "node:child_process";
import { createServer } from "vite";

const server = await createServer({ configFile: new URL("../vite.config.ts", import.meta.url).pathname });
await server.listen();
const url = server.resolvedUrls.local[0];
const electron = (await import("electron")).default;
const child = spawn(electron, ["."], {
  cwd: new URL("..", import.meta.url).pathname,
  env: { ...process.env, VITE_DEV_SERVER_URL: url },
  stdio: "inherit",
});
child.on("exit", async (code) => {
  await server.close();
  process.exit(code ?? 0);
});
