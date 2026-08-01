import type { RunRecord, ScenarioDetail } from "../app/api";
import { PanelFrame } from "./PanelFrame";

export function RvizPanel({ scenario, run }: { scenario: ScenarioDetail; run: RunRecord | null }) {
  const configured = scenario.display_plan.rviz !== null;
  const src = "/display/rviz/vnc.html?autoconnect=true&resize=scale&path=display%2Frviz%2Fwebsockify";
  return (
    <PanelFrame
      panelId="rviz"
      eyebrow="ROS VIEW"
      title="RViz 多机器人视图"
      status={<span className={`live-mark ${run && configured ? "is-active" : ""}`}>{configured ? "独立显示 :102" : "未配置"}</span>}
    >
      {run && configured ? (
        <iframe className="visual-frame" title="RViz 多机器人视图" src={src} />
      ) : (
        <div className="panel-empty panel-empty--map">
          <span className="map-crosshair" aria-hidden="true" />
          <p>等待 RViz 观测进程</p>
          <small>运行失败不会中断主 ROS 任务</small>
        </div>
      )}
    </PanelFrame>
  );
}
