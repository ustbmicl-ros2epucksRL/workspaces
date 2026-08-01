import type { DataAvailability, DataSource } from "../app/api";
import { PanelFrame } from "./PanelFrame";
import { StateBadge } from "./panelUtils";

export function HealthPanel({ sources, running }: { sources: DataSource[]; running: boolean }) {
  const visible = sources.length
    ? sources.slice(0, 6)
    : [
        {
          topic_pattern: "ROS graph",
          capability: "graph",
          required: true,
          availability: (running ? "DISCONNECTED" : "WAITING") as DataAvailability
        }
      ];
  return (
    <PanelFrame panelId="health" eyebrow="GRAPH HEALTH" title="机器人与话题">
      <div className="health-list">
        {visible.map((source) => (
          <div className="health-row" key={`${source.topic_pattern}-${source.capability}`}>
            <div>
              <strong>{source.topic ?? source.topic_pattern}</strong>
              <span>{healthDetail(source)}</span>
            </div>
            <StateBadge state={source.availability} />
          </div>
        ))}
      </div>
    </PanelFrame>
  );
}

function healthDetail(source: DataSource): string {
  const parts = [source.required ? "必要数据" : "可选数据"];
  if (source.observed_hz !== undefined) parts.push(`${source.observed_hz.toFixed(1)} Hz`);
  if (source.last_message_age_ms !== undefined) parts.push(`${Math.round(source.last_message_age_ms)} ms`);
  return parts.join(" · ");
}
