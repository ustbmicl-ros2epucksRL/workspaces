import { useEffect, useMemo, useRef, useState } from "react";
import type { DataSource } from "../app/api";
import { MapRenderer } from "../ros/MapRenderer";
import { rosDataRuntime } from "../ros/RosDataRuntime";
import { PanelFrame } from "./PanelFrame";
import { sourceFor, StateBadge } from "./panelUtils";

export function MapPanel({ sources, trajectoryOnly = false }: { sources: DataSource[]; trajectoryOnly?: boolean }) {
  const map = sourceFor(sources, "map");
  const odometry = useMemo(
    () => sources.filter((source) => source.capability === "odometry" && source.topic).slice(0, 20),
    [sources]
  );
  const state = trajectoryOnly ? combinedState(odometry) : map?.availability ?? "WAITING";
  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<MapRenderer | null>(null);
  const [renderError, setRenderError] = useState<string | null>(null);
  const subscriptionKey = trajectoryOnly
    ? odometry.map(sourceKey).join("|")
    : map ? sourceKey(map) : "";

  useEffect(() => {
    if (!containerRef.current) return;
    const renderer = new MapRenderer(containerRef.current);
    rendererRef.current = renderer;
    return () => {
      renderer.destroy();
      rendererRef.current = null;
    };
  }, [trajectoryOnly]);

  useEffect(() => {
    if (trajectoryOnly) {
      return releaseAll(
        odometry.map((source) =>
          rosDataRuntime.subscribe(source, (message) => {
            void rendererRef.current
              ?.renderOdometry(source.topic ?? source.topic_pattern, message)
              .then((rendered) => setRenderError(rendered ? null : "位姿消息结构无效"))
              .catch(() => setRenderError("轨迹渲染器不可用"));
          })
        )
      );
    }
    if (!map?.topic) return;
    return rosDataRuntime.subscribe(map, (message) => {
      void rendererRef.current
        ?.renderOccupancyGrid(message)
        .then((rendered) => setRenderError(rendered ? null : "OccupancyGrid 消息结构无效"))
        .catch(() => setRenderError("地图渲染器不可用"));
    });
  // Exact graph/topic/type identity controls the underlying subscription.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [trajectoryOnly, subscriptionKey]);

  return (
    <PanelFrame
      panelId={trajectoryOnly ? "trajectory" : "map"}
      eyebrow={trajectoryOnly ? "POSE TRACE" : "OCCUPANCY GRID"}
      title={trajectoryOnly ? "轨迹与位姿" : "任务地图"}
      status={<StateBadge state={state} />}
    >
      <div className="native-map-placeholder">
        <div className="map-grid" aria-hidden="true" />
        <div className="ros-map-canvas" ref={containerRef} data-testid={trajectoryOnly ? "trajectory-canvas" : "map-canvas"} />
        {state !== "ONLINE" || renderError ? (
          <div className="map-message">
            <strong>{renderError ?? (trajectoryOnly ? "等待机器人位姿" : "等待 /sim_map")}</strong>
            <span>{trajectoryOnly ? "收到位姿后叠加各机器人轨迹；画布支持拖动与缩放" : "收到真实 OccupancyGrid 后显示；画布支持拖动与缩放"}</span>
          </div>
        ) : null}
      </div>
    </PanelFrame>
  );
}

function sourceKey(source: DataSource): string {
  return [source.graph, source.topic, source.message_type].join(":");
}

function releaseAll(releases: Array<() => void>): () => void {
  return () => releases.forEach((release) => release());
}

function combinedState(sources: DataSource[]): DataSource["availability"] {
  if (sources.some((source) => source.availability === "ONLINE")) return "ONLINE";
  if (sources.some((source) => source.availability === "TYPE_MISMATCH")) return "TYPE_MISMATCH";
  if (sources.some((source) => source.availability === "DISCONNECTED")) return "DISCONNECTED";
  if (sources.some((source) => source.availability === "STALE")) return "STALE";
  return "WAITING";
}
