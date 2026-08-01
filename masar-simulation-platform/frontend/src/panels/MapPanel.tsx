import type { DataSource } from "../app/api";
import { PanelFrame } from "./PanelFrame";
import { sourceFor, StateBadge } from "./panelUtils";

export function MapPanel({ sources, trajectoryOnly = false }: { sources: DataSource[]; trajectoryOnly?: boolean }) {
  const map = sourceFor(sources, "map");
  const state = map?.availability ?? "WAITING";
  return (
    <PanelFrame
      panelId={trajectoryOnly ? "trajectory" : "map"}
      eyebrow={trajectoryOnly ? "POSE TRACE" : "OCCUPANCY GRID"}
      title={trajectoryOnly ? "轨迹与位姿" : "任务地图"}
      status={<StateBadge state={state} />}
    >
      <div className="native-map-placeholder">
        <div className="map-grid" aria-hidden="true" />
        <div className="map-message">
          <strong>{trajectoryOnly ? "等待机器人位姿" : "等待 /sim_map"}</strong>
          <span>{trajectoryOnly ? "收到位姿后叠加各机器人轨迹" : "收到地图消息后显示真实 OccupancyGrid"}</span>
        </div>
      </div>
    </PanelFrame>
  );
}
