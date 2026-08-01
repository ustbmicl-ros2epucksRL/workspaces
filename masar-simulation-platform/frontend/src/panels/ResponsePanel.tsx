import type { DataSource } from "../app/api";
import { PanelFrame } from "./PanelFrame";
import { sourceFor, StateBadge } from "./panelUtils";

export function ResponsePanel({ sources }: { sources: DataSource[] }) {
  const response = sourceFor(sources, "response_status");
  const state = response?.availability ?? "WAITING";
  return (
    <PanelFrame
      panelId="response"
      eyebrow="RESPONSE GROUP"
      title="响应组状态"
      status={<StateBadge state={state} />}
    >
      <dl className="response-grid">
        <div><dt>当前行为</dt><dd>未提供</dd></div>
        <div><dt>任务目标</dt><dd>未提供</dd></div>
        <div><dt>编队误差</dt><dd>未提供</dd></div>
        <div><dt>安全间距</dt><dd>未提供</dd></div>
      </dl>
    </PanelFrame>
  );
}
