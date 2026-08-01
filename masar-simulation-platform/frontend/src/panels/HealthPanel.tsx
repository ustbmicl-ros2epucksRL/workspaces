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
              <strong>{source.topic_pattern}</strong>
              <span>{source.required ? "必要数据" : "可选数据"}</span>
            </div>
            <StateBadge state={source.availability} />
          </div>
        ))}
      </div>
    </PanelFrame>
  );
}
