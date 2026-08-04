import { useEffect, useMemo, useState } from "react";
import type { DataSource, WorkspaceId } from "../app/api";
import {
  decodeAgentStatus,
  decodeEmergencyObject,
  decodeEmergencyObjects,
  type AgentStatusSnapshot,
  type EmergencyObjectKind,
  type EmergencyObjectSnapshot
} from "../ros/decoders";
import { rosDataRuntime } from "../ros/RosDataRuntime";
import { robotOptions, sourceForRobot } from "../ros/robotSources";
import { runScopedSourceKey } from "../ros/sourceIdentity";
import { PanelFrame } from "./PanelFrame";
import { sourceFor } from "./panelUtils";

const kindLabels: Record<EmergencyObjectKind, string> = {
  victim: "受害者",
  fire: "火源",
  box: "待推送障碍"
};

interface MissionPanelProps {
  runId: string | null;
  sources: DataSource[];
  running: boolean;
  workspace: WorkspaceId;
  expectedRobotCount: number;
}

export function MissionPanel({
  runId,
  sources,
  running,
  workspace,
  expectedRobotCount
}: MissionPanelProps) {
  const agentSource = sourceFor(sources, "agent_status");
  const objectsSource = sourceFor(sources, "emergency_objects");
  const eventSource = sourceFor(sources, "emergency_events");
  const [agents, setAgents] = useState<AgentStatusSnapshot | null>(null);
  const [objects, setObjects] = useState<EmergencyObjectSnapshot[] | null>(null);
  const [latestObject, setLatestObject] = useState<EmergencyObjectSnapshot | null>(null);
  const agentKey = runScopedSourceKey(runId, agentSource);
  const objectsKey = runScopedSourceKey(runId, objectsSource);
  const eventKey = runScopedSourceKey(runId, eventSource);

  useEffect(() => {
    setAgents(null);
    setObjects(null);
    setLatestObject(null);
  }, [runId]);

  useEffect(() => subscribeIfPresent(agentSource, (message) => setAgents(decodeAgentStatus(message))),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [agentKey]
  );
  useEffect(() => subscribeIfPresent(objectsSource, (message) => {
    const decoded = decodeEmergencyObjects(message);
    if (decoded) setObjects(decoded);
  }),
  // eslint-disable-next-line react-hooks/exhaustive-deps
  [objectsKey]);
  useEffect(() => subscribeIfPresent(eventSource, (message) => {
    const decoded = decodeEmergencyObject(message);
    if (decoded) setLatestObject(decoded);
  }),
  // eslint-disable-next-line react-hooks/exhaustive-deps
  [eventKey]);

  const counts = useMemo(() => objects === null ? null : countObjects(objects), [objects]);

  return (
    <PanelFrame
      panelId="mission"
      eyebrow={workspace === "rescue" ? "RESCUE SITUATION" : "EXPLORATION SITUATION"}
      title={workspace === "rescue" ? "搜救任务态势" : "探索任务态势"}
    >
      <div className="mission-situation">
        <dl className="mission-stat-grid">
          <div><dt>在线探索机器人</dt><dd data-testid="active-agent-count">{agentCount(agents, sources, expectedRobotCount)}</dd></div>
          <div><dt>受害者</dt><dd data-testid="emergency-count-victim">{countOrDash(counts?.victim)}</dd></div>
          <div><dt>火源</dt><dd data-testid="emergency-count-fire">{countOrDash(counts?.fire)}</dd></div>
          <div><dt>待推送障碍</dt><dd data-testid="emergency-count-box">{countOrDash(counts?.box)}</dd></div>
        </dl>

        <div className="emergency-feed">
          <div className="emergency-feed__heading">
            <strong>已发现搜救对象</strong>
            <span>{latestObject
              ? `最近：${kindLabels[latestObject.kind]} ${latestObject.id}`
              : objects === null ? "等待快照" : `${objects.length} 个`}</span>
          </div>
          <ol>
            {objects !== null && objects.length > 0 ? objects.slice(-6).reverse().map((object) => (
              <li key={`${object.kind}-${object.id}`}>
                <i className={`emergency-kind emergency-kind--${object.kind}`} aria-hidden="true" />
                <div>
                  <strong>{kindLabels[object.kind]} {object.id}</strong>
                  <span>({object.position.x.toFixed(2)}, {object.position.y.toFixed(2)})</span>
                </div>
                <small>{object.detectorRobot || "探测机器人未知"}</small>
              </li>
            )) : (
              <li className="emergency-feed__empty">
                <span>{!running ? "等待运行" : objects === null ? "等待搜救对象快照" : "尚未发现搜救对象"}</span>
              </li>
            )}
          </ol>
        </div>
      </div>
    </PanelFrame>
  );
}

function subscribeIfPresent(source: DataSource | undefined, callback: (message: unknown) => void): (() => void) | undefined {
  return source?.topic ? rosDataRuntime.subscribe(source, callback) : undefined;
}

function countObjects(objects: EmergencyObjectSnapshot[]): Record<EmergencyObjectKind, number> {
  const result: Record<EmergencyObjectKind, number> = { victim: 0, fire: 0, box: 0 };
  for (const object of objects) result[object.kind] += 1;
  return result;
}

function countOrDash(value: number | undefined): string {
  return value === undefined ? "—" : String(value);
}

function agentCount(
  value: AgentStatusSnapshot | null,
  sources: DataSource[],
  expectedRobotCount: number
): string {
  const allRobots = robotOptions(sources, expectedRobotCount);
  const robots = allRobots.some((robot) => robot.key.startsWith("agent:"))
    ? allRobots.filter((robot) => robot.key.startsWith("agent:"))
    : allRobots;
  const odometry = robots.map((robot) => sourceForRobot(sources, "odometry", robot.key));
  const active = odometry.filter((source) => source?.availability === "ONLINE").length;
  if (active > 0) return `${active}/${robots.length}`;
  if (value) return `${value.active}/${value.total}`;
  if (!odometry.some((source) => source?.topic)) return "—";
  const observedOffline = odometry.some((source) => ["STALE", "DISCONNECTED"].includes(source?.availability ?? ""));
  return observedOffline ? `0/${robots.length}` : `—/${robots.length}`;
}
