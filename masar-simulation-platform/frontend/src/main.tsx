import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./app/App";
import { SchoolBrandPortal } from "./app/SchoolBrandPortal";
import "./styles/tokens.css";
import "./styles/layout.css";
import "./styles/branding.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
    <SchoolBrandPortal />
  </StrictMode>
);
