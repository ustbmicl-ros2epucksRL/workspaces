import type { DataSource } from "../app/api";

export interface RobotOption {
  key: string;
  label: string;
}

const ROS1_AGENT_TOPIC = /^\/robosar_agent_bringup_node\/(agent_(\d+))\//;
const NAMESPACED_ROBOT_TOPIC = /^\/(robot_(\d+)|tb_(\d+))\//;
const ROS1_AGENT_PATTERN = /^\/robosar_agent_bringup_node\/<agent>\//;
const RESPONSE_ROBOT_PATTERN = /^\/tb_i\//;

export function robotKeyForSource(source: DataSource, splitGroups = false): string | null {
  if (!source.topic) return null;
  const ros1 = ROS1_AGENT_TOPIC.exec(source.topic);
  if (ros1) return splitGroups ? `agent:${ros1[2]}` : ros1[2];
  const namespaced = NAMESPACED_ROBOT_TOPIC.exec(source.topic);
  if (!namespaced) return null;
  const robotIndex = namespaced[2] ?? namespaced[3];
  if (splitGroups && namespaced[3] !== undefined) return `tb:${robotIndex}`;
  return robotIndex;
}

export function robotOptions(sources: DataSource[], expectedCount = 0): RobotOption[] {
  const splitGroups = shouldSplitRobotGroups(sources, expectedCount);
  const keys = new Set<string>();
  for (let index = 0; index < expectedCount; index += 1) {
    keys.add(splitGroups ? `agent:${index}` : String(index));
  }
  for (const source of sources) {
    const key = robotKeyForSource(source, splitGroups);
    if (key !== null) keys.add(key);
  }
  return [...keys]
    .sort(compareRobotKeys)
    .map((key) => ({ key, label: robotLabelForKey(key) }));
}

export function robotSupportsCapability(robotType: string | null, capability: string): boolean {
  if (robotType === "e_puck") {
    return ["camera", "laser", "infrared", "odometry", "command_velocity"].includes(capability);
  }
  if (robotType === "burger") {
    return ["laser", "imu", "odometry", "command_velocity"].includes(capability);
  }
  return true;
}

export function sourceForRobot(
  sources: DataSource[],
  capability: string,
  robotKey: string | null
): DataSource | undefined {
  if (robotKey === null) return undefined;
  const splitGroups = shouldSplitRobotGroups(sources) || robotKey.includes(":");
  return sourceForCapability(sources.filter(
    (source) => source.capability === capability && robotKeyForSource(source, splitGroups) === robotKey
  ), capability);
}

export function sourceForCapability(
  sources: DataSource[],
  capability: string
): DataSource | undefined {
  let preferred: DataSource | undefined;
  for (const source of sources) {
    if (source.capability !== capability) continue;
    if (!preferred || availabilityRank(source) < availabilityRank(preferred)) preferred = source;
  }
  return preferred;
}

export function robotLabelForSource(source: DataSource): string {
  const key = robotKeyForSource(source);
  return key === null ? source.topic ?? source.topic_pattern : `机器人 ${key}`;
}

function availabilityRank(source: DataSource): number {
  const ranks: Record<DataSource["availability"], number> = {
    ONLINE: 0,
    STALE: 1,
    DISCOVERED: 2,
    WAITING: 2,
    UNKNOWN: 2,
    DISCONNECTED: 3,
    TYPE_MISMATCH: 4,
    UNAVAILABLE: 5
  };
  return ranks[source.availability];
}

export function shouldSplitRobotGroups(sources: DataSource[], expectedCount = 0): boolean {
  const exploration = new Set<string>();
  const response = new Set<string>();
  let hasExplorationPattern = false;
  let hasResponsePattern = false;
  for (const source of sources) {
    hasExplorationPattern ||= ROS1_AGENT_PATTERN.test(source.topic_pattern);
    hasResponsePattern ||= RESPONSE_ROBOT_PATTERN.test(source.topic_pattern);
    if (source.topic) {
      const ros1 = ROS1_AGENT_TOPIC.exec(source.topic);
      if (ros1) exploration.add(ros1[2]);
      const namespaced = NAMESPACED_ROBOT_TOPIC.exec(source.topic);
      if (namespaced?.[3] !== undefined) response.add(namespaced[3]);
    }
  }
  return [...exploration].some((key) => response.has(key))
    || (expectedCount > 0 && hasExplorationPattern && hasResponsePattern);
}

function compareRobotKeys(left: string, right: string): number {
  const parsedLeft = parseRobotKey(left);
  const parsedRight = parseRobotKey(right);
  if (parsedLeft.groupOrder !== parsedRight.groupOrder) return parsedLeft.groupOrder - parsedRight.groupOrder;
  return parsedLeft.index - parsedRight.index;
}

function robotLabelForKey(key: string): string {
  const parsed = parseRobotKey(key);
  if (key.startsWith("agent:")) return `探索机器人 ${parsed.index}`;
  if (key.startsWith("tb:")) return `响应机器人 ${parsed.index}`;
  return `机器人 ${parsed.index}`;
}

function parseRobotKey(key: string): { groupOrder: number; index: number } {
  const [prefix, value] = key.includes(":") ? key.split(":", 2) : ["robot", key];
  const index = Number(value);
  const groupOrder = prefix === "agent" ? 0 : prefix === "tb" ? 1 : 0;
  return { groupOrder, index: Number.isFinite(index) ? index : Number.MAX_SAFE_INTEGER };
}
