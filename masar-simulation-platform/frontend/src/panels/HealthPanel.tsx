import { useEffect, useMemo } from "react";
import type { DataAvailability, DataSource } from "../app/api";
import { robotKeyForSource, robotOptions, shouldSplitRobotGroups, sourceForCapability } from "../ros/robotSources";
import { rosDataRuntime } from "../ros/RosDataRuntime";
import { runScopedSourceKey, sourceKey } from "../ros/sourceIdentity";
import { PanelFrame } from "./PanelFrame";
import { StateBadge } from "./panelUtils";

interface HealthPanelProps {
  runId: string | null;
  sources: DataSource[];
  running: boolean;
  expectedRobotCount: number;
}

export function HealthPanel({ runId, sources, running, expectedRobotCount }: HealthPanelProps) {
  const robots = useMemo(
    () => robotOptions(sources, expectedRobotCount),
    [sources, expectedRobotCount]
  );
  const splitRobotGroups = useMemo(
    () => shouldSplitRobotGroups(sources, expectedRobotCount),
    [sources, expectedRobotCount]
  );
  const robotSources = useMemo(
    () => robots.map((robot) => ({
      ...robot,
      sources: sources.filter((source) => robotKeyForSource(source, splitRobotGroups) === robot.key)
    })),
    [robots, sources, splitRobotGroups]
  );
  const systemSources = sources.filter(
    (source) => robotKeyForSource(source, splitRobotGroups) === null
      && !isUnresolvedRobotPattern(source)
      && (Boolean(source.topic) || source.required || !["WAITING", "UNKNOWN"].includes(source.availability))
  );
  const odometry = sources.filter((source) => source.capability === "odometry" && source.topic);
  const odometryKey = odometry.map((source) => runScopedSourceKey(runId, source)).join("|");

  useEffect(() => releaseAll(odometry.map((source) => rosDataRuntime.subscribe(source, () => undefined))),
    // Run and topic identity control subscriptions; health updates do not recreate them.
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [odometryKey]
  );

  return (
    <PanelFrame panelId="health" eyebrow="GRAPH HEALTH" title="机器人与话题">
      <div className="health-list">
        {robotSources.map((robot) => (
          <div className="health-row" data-testid="health-robot-row" key={robot.key}>
            <div>
              <strong>{robot.label}</strong>
              <span>{robotHealthDetail(robot.sources, running)}</span>
            </div>
            <StateBadge state={combinedRobotState(robot.sources, running)} />
          </div>
        ))}
        {systemSources.map((source) => (
          <div className="health-row" key={sourceKey(source)}>
            <div>
              <strong>{source.topic ?? source.topic_pattern}</strong>
              <span>{healthDetail(source, running)}</span>
            </div>
            <StateBadge state={presentedState(source, running)} />
          </div>
        ))}
        {robots.length === 0 && systemSources.length === 0 ? (
          <div className="health-row">
            <div>
              <strong>ROS graph</strong>
              <span>{running ? "尚未发现允许的话题" : "等待运行"}</span>
            </div>
            <StateBadge state={(running ? "DISCONNECTED" : "WAITING") as DataAvailability} />
          </div>
        ) : null}
      </div>
    </PanelFrame>
  );
}

function robotHealthDetail(sources: DataSource[], running: boolean): string {
  if (!running) return "等待运行";
  const discovered = sources.filter((source) => source.topic).length;
  if (discovered === 0) return "未发现机器人话题";
  const odometry = sourceForCapability(sources.filter((source) => source.topic), "odometry");
  const capabilities = [...new Set(sources.filter((source) => source.topic).map((source) => capabilityLabel(source.capability)))];
  const sourceSummary = `${discovered} 个数据源已发现`;
  if (!odometry) return `${sourceSummary} · 未发现位姿话题 · ${capabilities.join(" / ")}`;
  if (odometry.availability === "ONLINE") {
    const rate = odometry.observed_hz === undefined ? "位姿在线" : `位姿 ${odometry.observed_hz.toFixed(1)} Hz`;
    return `${rate} · ${sourceSummary} · ${capabilities.join(" / ")}`;
  }
  if (odometry.availability === "STALE") return `位姿已陈旧 · ${sourceSummary}`;
  if (odometry.availability === "TYPE_MISMATCH") return `位姿类型不匹配 · ${sourceSummary}`;
  if (odometry.availability === "DISCONNECTED") return `ROS 连接中断 · ${sourceSummary}`;
  return `位姿话题已发现 · 等待首帧 · ${sourceSummary}`;
}

function capabilityLabel(capability: string): string {
  const labels: Record<string, string> = {
    camera: "相机",
    laser: "激光",
    infrared: "红外",
    imu: "IMU",
    odometry: "位姿",
    command_velocity: "速度",
    discussion_opinion: "讨论观点",
    dispersion_distance: "分散距离",
    discussion_broadcast: "讨论广播"
  };
  return labels[capability] ?? capability;
}

function combinedRobotState(sources: DataSource[], running: boolean): DataAvailability {
  if (!running) return "WAITING";
  if (sources.some((source) => source.availability === "TYPE_MISMATCH")) return "TYPE_MISMATCH";
  const odometry = sourceForCapability(sources.filter((source) => source.topic), "odometry");
  if (odometry?.availability === "ONLINE") return "ONLINE";
  if (odometry?.availability === "STALE") return "STALE";
  if (odometry?.availability === "DISCONNECTED") return "DISCONNECTED";
  if (sources.some((source) => source.topic)) return "DISCOVERED";
  return "DISCONNECTED";
}

function healthDetail(source: DataSource, running: boolean): string {
  if (!running) return source.required ? "必要数据 · 等待运行" : "可选数据 · 等待运行";
  const parts = [source.required ? "必要数据" : "可选数据"];
  if (source.observed_hz !== undefined) parts.push(`${source.observed_hz.toFixed(1)} Hz`);
  else if (source.topic && ["WAITING", "UNKNOWN"].includes(source.availability)) parts.push("话题已发现 · 未采样");
  if (source.last_message_age_ms !== undefined) parts.push(`${Math.round(source.last_message_age_ms)} ms`);
  if (!source.topic) parts.push("未发现话题");
  return parts.join(" · ");
}

function presentedState(source: DataSource, running: boolean): DataAvailability {
  if (!running) return "WAITING";
  if (["ONLINE", "STALE", "TYPE_MISMATCH", "DISCONNECTED", "UNAVAILABLE"].includes(source.availability)) {
    return source.availability;
  }
  if (!source.topic && ["WAITING", "UNKNOWN"].includes(source.availability)) return source.availability;
  if (source.topic && ["WAITING", "UNKNOWN"].includes(source.availability)) return "DISCOVERED";
  if (!source.topic && source.required) return "DISCONNECTED";
  return source.availability;
}

function releaseAll(releases: Array<() => void>): () => void {
  return () => releases.forEach((release) => release());
}

function isUnresolvedRobotPattern(source: DataSource): boolean {
  return !source.topic && /<robot>|<agent>|tb_i/.test(source.topic_pattern);
}
