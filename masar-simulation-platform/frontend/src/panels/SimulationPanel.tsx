import { useEffect, useRef } from "react";
import type { RunRecord, ScenarioDetail } from "../app/api";
import { PanelFrame } from "./PanelFrame";

interface WebotsViewElement extends HTMLElement {
  connect: (
    server: string,
    mode: string,
    broadcast: boolean,
    mobile: boolean,
    timeout: number,
    thumbnail: string
  ) => void;
  close?: () => void;
  showQuit?: boolean;
  showWorldSelection?: boolean;
}

function WebotsStream() {
  const host = useRef<HTMLDivElement>(null);
  useEffect(() => {
    let disposed = false;
    let view: WebotsViewElement | null = null;
    const start = async () => {
      if (!customElements.get("webots-view")) {
        let script = document.querySelector<HTMLScriptElement>("#webots-r2025a-module");
        if (!script) {
          script = document.createElement("script");
          script.id = "webots-r2025a-module";
          script.type = "module";
          script.src = "https://cyberbotics.com/wwi/R2025a/WebotsView.js";
          document.head.appendChild(script);
        }
      }
      await customElements.whenDefined("webots-view");
      if (disposed || !host.current) return;
      view = document.createElement("webots-view") as WebotsViewElement;
      view.className = "visual-frame webots-frame";
      view.showQuit = false;
      view.showWorldSelection = false;
      host.current.replaceChildren(view);
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      view.connect(
        `${protocol}//${window.location.host}/webots`,
        "w3d",
        false,
        false,
        -1,
        "https://cyberbotics.com/wwi/R2025a/images/loading/default_thumbnail.png"
      );
    };
    void start();
    return () => {
      disposed = true;
      view?.close?.();
    };
  }, []);
  return <div className="visual-frame-host" ref={host} />;
}

function noVncUrl(path: string): string {
  const base = path.replace(/\/$/, "");
  const socketPath = `${base.replace(/^\//, "")}/websockify`;
  return `${base}/vnc.html?autoconnect=true&resize=scale&path=${encodeURIComponent(socketPath)}`;
}

export function SimulationPanel({ scenario, run }: { scenario: ScenarioDetail; run: RunRecord | null }) {
  const simulation = scenario.display_plan.simulation;
  return (
    <PanelFrame
      panelId="simulation"
      eyebrow={scenario.simulator === "webots" ? "W3D STREAM" : "NATIVE GUI"}
      title={scenario.simulator === "webots" ? "Webots 仿真" : "Gazebo 仿真"}
      status={<span className={`live-mark ${run ? "is-active" : ""}`}>{run ? "画面通道" : "待启动"}</span>}
    >
      {run ? (
        simulation.transport === "webots" ? (
          <WebotsStream />
        ) : (
          <iframe className="visual-frame" title="Gazebo 仿真画面" src={noVncUrl(simulation.path)} />
        )
      ) : (
        <VisualStandby simulator={scenario.simulator} />
      )}
    </PanelFrame>
  );
}

function VisualStandby({ simulator }: { simulator: string }) {
  return (
    <div className="visual-standby">
      <div className="radar-sweep" aria-hidden="true">
        <i />
        <i />
        <i />
      </div>
      <p>启动场景后接入 {simulator === "webots" ? "Webots 三维流" : "Gazebo 原生桌面"}</p>
      <span>画面支持鼠标旋转、拖动与缩放</span>
    </div>
  );
}
