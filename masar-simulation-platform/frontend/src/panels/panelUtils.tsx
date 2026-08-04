import type { DataAvailability, DataSource } from "../app/api";

export const availabilityLabels: Record<DataAvailability, string> = {
  UNAVAILABLE: "未提供",
  DISCOVERED: "已发现",
  WAITING: "等待数据",
  ONLINE: "在线",
  STALE: "数据已陈旧",
  TYPE_MISMATCH: "类型不匹配",
  DISCONNECTED: "连接中断",
  UNKNOWN: "待检测"
};

export function sourceFor(sources: DataSource[], capability: string): DataSource | undefined {
  return sources.find((source) => source.capability === capability);
}

export function StateBadge({ state }: { state: DataAvailability }) {
  return (
    <span className={`state-badge state-badge--${state.toLowerCase()}`} data-testid={`topic-state-${state}`}>
      <i aria-hidden="true" />
      {availabilityLabels[state]}
    </span>
  );
}
