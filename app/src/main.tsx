import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import App from "./App";
import { bridge } from "./bridge";
import "./styles.css";

window.haggatrons = bridge;

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
