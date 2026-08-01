import { PanelFrame } from "./PanelFrame";

export function ProcessLogPanel({ running }: { running: boolean }) {
  return (
    <PanelFrame panelId="logs" eyebrow="PROCESS OUTPUT" title="节点日志">
      <div className="process-placeholder">
        <span>$</span>
        <p>{running ? "选择进程后读取 Agent 持有的 stdout / stderr" : "启动场景后可查看节点日志"}</p>
      </div>
    </PanelFrame>
  );
}
