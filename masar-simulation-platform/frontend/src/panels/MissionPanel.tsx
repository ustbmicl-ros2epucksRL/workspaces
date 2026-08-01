import type { RunEvent, RunRecord, WorkspaceId } from "../app/api";
import { PanelFrame } from "./PanelFrame";

const phaseLabels: Record<string, string> = {
  CREATED: "已创建",
  VALIDATING: "环境预检",
  STARTING: "启动节点",
  RUNNING: "任务运行",
  STOPPING: "停止中",
  COMPLETED: "已完成",
  FAILED: "运行失败"
};

export function MissionPanel({
  run,
  events,
  workspace
}: {
  run: RunRecord | null;
  events: RunEvent[];
  workspace: WorkspaceId;
}) {
  const recent = events.slice(-4).reverse();
  return (
    <PanelFrame
      panelId="mission"
      eyebrow={workspace === "rescue" ? "RESCUE FLOW" : "MISSION FLOW"}
      title={workspace === "rescue" ? "搜救任务进程" : "探索任务进程"}
    >
      <div className="mission-phase">
        <span>当前阶段</span>
        <strong>{run ? phaseLabels[run.state] ?? run.state : "等待场景启动"}</strong>
        <div className={`mission-rule mission-rule--${run?.state.toLowerCase() ?? "idle"}`} aria-hidden="true"><i /></div>
      </div>
      <ol className="event-compact">
        {recent.length ? (
          recent.map((event) => (
            <li key={event.sequence}>
              <span>{String(event.sequence).padStart(3, "0")}</span>
              <strong>{event.kind}</strong>
            </li>
          ))
        ) : (
          <li className="is-empty"><strong>事件将在运行后按时间进入</strong></li>
        )}
      </ol>
    </PanelFrame>
  );
}
