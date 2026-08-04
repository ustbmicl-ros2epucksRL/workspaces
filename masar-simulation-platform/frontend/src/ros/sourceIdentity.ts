import type { DataSource } from "../app/api";

export function sourceKey(source: DataSource | undefined): string {
  return source
    ? [source.graph, source.topic ?? source.topic_pattern, source.message_type, source.capability].join(":")
    : "";
}

export function runScopedSourceKey(runId: string | null, source: DataSource | undefined): string {
  return `${runId ?? "idle"}:${sourceKey(source)}`;
}
