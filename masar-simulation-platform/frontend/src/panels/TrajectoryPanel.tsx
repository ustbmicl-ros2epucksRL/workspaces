import { useEffect, useMemo, useState } from "react";
import type { DataSource } from "../app/api";
import {
  decodeGazeboModelPoses,
  decodeGlobalInfoRobotPoses,
  type WorldPose2D,
  type WorldRobotPoses
} from "../ros/decoders";
import { robotOptions } from "../ros/robotSources";
import { rosDataRuntime } from "../ros/RosDataRuntime";
import { runScopedSourceKey } from "../ros/sourceIdentity";
import {
  appendTrajectoryPoint,
  projectTrajectoryPoint,
  trajectoryBounds,
  type RobotTrajectories,
  type TrajectoryPoint
} from "../ros/trajectoryPlot";
import { PanelFrame } from "./PanelFrame";
import { sourceFor } from "./panelUtils";

const VIEW_WIDTH = 640;
const VIEW_HEIGHT = 360;
const VIEW_PADDING = 26;
const MAX_POINTS_PER_ROBOT = 360;
const MIN_POINT_DISTANCE_METERS = 0.006;
const TRACE_COLORS = [
  "#62d6c7",
  "#efb45c",
  "#78a9ff",
  "#ed7c8c",
  "#b69cff",
  "#9bd46a",
  "#f28c5b",
  "#57c7e3"
];

interface TrajectoryPanelProps {
  runId: string | null;
  sources: DataSource[];
  expectedRobotCount: number;
  running: boolean;
}

export function TrajectoryPanel({
  runId,
  sources,
  expectedRobotCount,
  running
}: TrajectoryPanelProps) {
  const robots = useMemo(
    () => robotOptions(sources, expectedRobotCount),
    [sources, expectedRobotCount]
  );
  const webotsWorld = sourceFor(sources, "global_info");
  const gazeboWorld = sourceFor(sources, "gazebo_model_states");
  const worldSource = webotsWorld?.topic ? webotsWorld : gazeboWorld;
  const worldSourceKey = runScopedSourceKey(runId, worldSource);
  const [trajectories, setTrajectories] = useState<RobotTrajectories>({});
  const [poses, setPoses] = useState<WorldRobotPoses>({});

  useEffect(() => {
    setTrajectories({});
    setPoses({});
  }, [runId]);

  useEffect(() => {
    if (!worldSource?.topic) return;
    return rosDataRuntime.subscribe(worldSource, (message) => {
      const snapshot = worldSource.capability === "global_info"
        ? decodeGlobalInfoRobotPoses(message)
        : decodeGazeboModelPoses(message);
      if (!snapshot) return;
      setPoses(snapshot);
      setTrajectories((current) => appendWorldSnapshot(current, snapshot));
    });
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [worldSourceKey]);

  const bounds = useMemo(() => trajectoryBounds(trajectories), [trajectories]);
  const plotLines = useMemo(() => gridLines(bounds), [bounds]);
  const activeCount = Object.keys(poses).length;

  return (
    <PanelFrame
      panelId="trajectory"
      eyebrow="LIVE XY TRACE"
      title="机器人运行轨迹"
      status={<span className="trajectory-count">{activeCount}/{robots.length} 世界位姿</span>}
    >
      <div className="trajectory-plot-shell">
        <svg
          className="trajectory-plot"
          data-testid="trajectory-plot"
          viewBox={`0 0 ${VIEW_WIDTH} ${VIEW_HEIGHT}`}
          role="img"
          aria-label="多机器人实时 XY 运行轨迹"
        >
          <rect className="trajectory-plot__background" width={VIEW_WIDTH} height={VIEW_HEIGHT} />
          <g className="trajectory-plot__grid" aria-hidden="true">
            {plotLines.map((line) => (
              <line key={line.key} x1={line.x1} y1={line.y1} x2={line.x2} y2={line.y2} />
            ))}
          </g>
          <g className="trajectory-plot__axis-labels" aria-hidden="true">
            <text x={18} y={18}>Y {bounds.maxY.toFixed(1)} m</text>
            <text x={18} y={VIEW_HEIGHT - 10}>Y {bounds.minY.toFixed(1)} m</text>
            <text x={VIEW_WIDTH - 94} y={VIEW_HEIGHT - 10}>X {bounds.maxX.toFixed(1)} m</text>
          </g>
          {robots.map((robot, index) => {
            const points = trajectories[robot.key] ?? [];
            const projected = points.map((point) =>
              projectTrajectoryPoint(point, bounds, VIEW_WIDTH, VIEW_HEIGHT, VIEW_PADDING)
            );
            const latest = projected.at(-1);
            const color = TRACE_COLORS[index % TRACE_COLORS.length];
            return (
              <g key={robot.key} style={{ color }}>
                {projected.length > 1 ? (
                  <polyline
                    className="trajectory-plot__trace"
                    data-testid={`trajectory-path-${robot.key}`}
                    points={projected.map((point) => `${point.x.toFixed(2)},${point.y.toFixed(2)}`).join(" ")}
                  />
                ) : null}
                {latest ? (
                  <g className="trajectory-plot__marker" transform={`translate(${latest.x} ${latest.y})`}>
                    <circle r="7" />
                    <text y="3">{robot.key}</text>
                  </g>
                ) : null}
              </g>
            );
          })}
        </svg>

        {activeCount === 0 ? (
          <div className="trajectory-empty">
            <strong>{running ? "等待机器人世界位姿" : "等待运行"}</strong>
            <span>轨迹将在机器人移动时按统一 XY 比例同步绘制</span>
          </div>
        ) : null}

        <div className="trajectory-legend" aria-label="机器人实时位置">
          {robots.map((robot, index) => {
            const pose = poses[robot.key];
            return (
              <div className="trajectory-legend__row" data-testid="trajectory-robot-row" key={robot.key}>
                <i style={{ background: TRACE_COLORS[index % TRACE_COLORS.length] }} aria-hidden="true" />
                <strong>{robot.label}</strong>
                <span>{pose ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}` : poseStatus(worldSource, running)}</span>
              </div>
            );
          })}
        </div>
      </div>
    </PanelFrame>
  );
}

function appendIfMoved(
  trajectories: RobotTrajectories,
  robotKey: string,
  pose: WorldPose2D
): RobotTrajectories {
  const point = { x: pose.x, y: pose.y };
  const latest = trajectories[robotKey]?.at(-1);
  if (latest && Math.hypot(point.x - latest.x, point.y - latest.y) < MIN_POINT_DISTANCE_METERS) {
    return trajectories;
  }
  return appendTrajectoryPoint(trajectories, robotKey, point, MAX_POINTS_PER_ROBOT);
}

function appendWorldSnapshot(
  trajectories: RobotTrajectories,
  poses: WorldRobotPoses
): RobotTrajectories {
  let updated = trajectories;
  for (const [robotKey, pose] of Object.entries(poses)) {
    updated = appendIfMoved(updated, robotKey, pose);
  }
  return updated;
}

function gridLines(bounds: ReturnType<typeof trajectoryBounds>) {
  return Array.from({ length: 5 }, (_, index) => index / 4).flatMap((ratio) => {
    const x = bounds.minX + (bounds.maxX - bounds.minX) * ratio;
    const y = bounds.minY + (bounds.maxY - bounds.minY) * ratio;
    const verticalStart = projectTrajectoryPoint({ x, y: bounds.minY }, bounds, VIEW_WIDTH, VIEW_HEIGHT, VIEW_PADDING);
    const verticalEnd = projectTrajectoryPoint({ x, y: bounds.maxY }, bounds, VIEW_WIDTH, VIEW_HEIGHT, VIEW_PADDING);
    const horizontalStart = projectTrajectoryPoint({ x: bounds.minX, y }, bounds, VIEW_WIDTH, VIEW_HEIGHT, VIEW_PADDING);
    const horizontalEnd = projectTrajectoryPoint({ x: bounds.maxX, y }, bounds, VIEW_WIDTH, VIEW_HEIGHT, VIEW_PADDING);
    return [
      { key: `x-${ratio}`, x1: verticalStart.x, y1: verticalStart.y, x2: verticalEnd.x, y2: verticalEnd.y },
      { key: `y-${ratio}`, x1: horizontalStart.x, y1: horizontalStart.y, x2: horizontalEnd.x, y2: horizontalEnd.y }
    ];
  });
}

function poseStatus(source: DataSource | undefined, running: boolean): string {
  if (!running) return "等待运行";
  if (!source?.topic) return "未发现世界位姿源";
  if (source.availability === "TYPE_MISMATCH") return "类型不匹配";
  if (source.availability === "DISCONNECTED") return "连接中断";
  if (source.availability === "STALE") return "世界位姿已陈旧";
  return "等待世界位姿";
}
