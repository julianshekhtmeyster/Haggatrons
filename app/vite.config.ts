import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  base: "./",
  plugins: [react()],
  build: { outDir: "dist", emptyOutDir: true },
  // `npm run dev` proxies the API to a running `python -m haggatrons serve`.
  server: { port: 5178, strictPort: true, proxy: { "/api": "http://127.0.0.1:8765" } },
});
