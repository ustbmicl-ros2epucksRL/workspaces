import type { RunRecord, ScenarioDetail } from "../app/api";
import { PanelFrame } from "./PanelFrame";

function noVncUrl(path: string): string {
  const base = path.replace(/\/$/, "");
  const socketPath = `${base.replace(/^\//, "")}/websockify`;
  return `${base}/vnc.html?autoconnect=true&resize=scale&path=${encodeURIComponent(socketPath)}`;
}

export function SimulationPanel({ scenario, run }: { scenario: ScenarioDetail; run: RunRecord | null }) {
  const simulation = scenario.display_plan.simulation;
  const live = run !== null && !["COMPLETED", "FAILED"].includes(run.state);
  return (
    <PanelFrame
      panelId="simulation"
      eyebrow="NATIVE GUI"
      title={scenario.simulator === "webots" ? "Webots 仿真" : "Gazebo 仿真"}
      status={<span className={`live-mark ${live ? "is-active" : ""}`}>{live ? "画面通道" : run ? "运行已结束" : "待启动"}</span>}
    >
      {live ? (
        <iframe
          className="visual-frame"
          title={`${scenario.simulator === "webots" ? "Webots" : "Gazebo"} 仿真画面`}
          src={noVncUrl(simulation.path)}
        />
      ) : (
        <VisualStandby simulator={scenario.simulator} ended={run !== null} />
      )}
    </PanelFrame>
  );
}

function VisualStandby({ simulator, ended }: { simulator: string; ended: boolean }) {
  return (
    <div className="visual-standby">
      <div className="radar-sweep" aria-hidden="true">
        <i />
        <i />
        <i />
      </div>
      <p>{ended ? "运行已结束，可新建运行" : `启动场景后接入 ${simulator === "webots" ? "Webots" : "Gazebo"} 原生桌面`}</p>
      <span>{ended ? "归档已保留，实时画面已断开" : "画面支持鼠标旋转、拖动与缩放"}</span>
    </div>
  );
}
