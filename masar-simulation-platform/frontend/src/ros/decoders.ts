export interface LaserSnapshot {
  beamCount: number;
  validCount: number;
  closestMeters: number | null;
  farthestMeters: number | null;
}

export interface Pose2D {
  x: number;
  y: number;
  orientation: { x: number; y: number; z: number; w: number };
}

export interface WorldPose2D {
  x: number;
  y: number;
}

export type WorldRobotPoses = Record<string, WorldPose2D>;

export interface TwistSnapshot {
  linearSpeed: number;
  angularZ: number;
}

export interface OpinionSnapshot {
  robotId: number;
  opinion: number;
}

export interface RangeSnapshot {
  meters: number;
  minMeters: number | null;
  maxMeters: number | null;
}

export interface ImuSnapshot {
  acceleration: number;
  angularSpeed: number;
}

export interface TaskAssignment {
  robotId: string;
  start: { x: number; y: number };
  goal: { x: number; y: number };
  goalType: number;
  goalId: number;
}

export interface FrontierSnapshot {
  count: number;
  maximumInformationGain: number | null;
}

export interface AgentStatusSnapshot {
  total: number;
  active: number;
}

export type EmergencyObjectKind = "victim" | "fire" | "box";

export interface EmergencyObjectSnapshot {
  kind: EmergencyObjectKind;
  id: number;
  detectorRobot: string;
  position: { x: number; y: number };
  goal: { x: number; y: number } | null;
}

export type ResponseTaskKind = EmergencyObjectKind | "home";
export type ResponseTaskStatus = "new" | "dispatched" | "en_route" | "acting" | "done" | "failed";

export interface EmergencyTaskSnapshot {
  taskId: number;
  kind: ResponseTaskKind;
  status: ResponseTaskStatus;
  objectId: number;
  detectorRobot: string;
  staging: { x: number; y: number } | null;
  goal: { x: number; y: number } | null;
  behaviorType: string;
  assignedRobots: string[];
}

export type ResponseGroupState = "idle" | "navigating" | "acting" | "done" | "failed";

export interface ResponseStatusSnapshot {
  state: ResponseGroupState;
  activeTaskId: number | null;
  activeTaskKind: ResponseTaskKind | null;
  behaviorType: string;
  detail: string;
  formationCenter: { x: number; y: number } | null;
  target: { x: number; y: number } | null;
  holdRemaining: number;
  assignedRobots: string[];
  planningState: string;
  longitudinalScale: number;
  lateralScale: number;
  rmsSlotError: number;
  maxSlotError: number;
  minimumClearance: number;
  motionEnabled: boolean;
}

export function decodeTaskAllocation(message: unknown): TaskAssignment[] | null {
  if (!isRecord(message)) return null;
  const fields = [
    message.id,
    message.startx,
    message.starty,
    message.goalx,
    message.goaly,
    message.goal_type,
    message.goal_id
  ];
  if (!fields.every(Array.isArray)) return null;
  const [ids, startX, startY, goalX, goalY, goalTypes, goalIds] = fields as unknown[][];
  const length = ids.length;
  if ([startX, startY, goalX, goalY, goalTypes, goalIds].some((field) => field.length !== length)) {
    return null;
  }
  const assignments: TaskAssignment[] = [];
  for (let index = 0; index < length; index += 1) {
    const robotId = ids[index];
    const values = [startX[index], startY[index], goalX[index], goalY[index], goalTypes[index], goalIds[index]];
    if (typeof robotId !== "string" || values.some((value) => finiteNumber(value) === null)) return null;
    assignments.push({
      robotId,
      start: { x: values[0] as number, y: values[1] as number },
      goal: { x: values[2] as number, y: values[3] as number },
      goalType: values[4] as number,
      goalId: values[5] as number
    });
  }
  return assignments;
}

export function decodeFrontiers(message: unknown): FrontierSnapshot | null {
  if (!isRecord(message) || !Array.isArray(message.points) || !Array.isArray(message.infoGain)) return null;
  if (message.infoGain.some((value) => finiteNumber(value) === null)) return null;
  let maximumInformationGain: number | null = null;
  for (const value of message.infoGain) {
    maximumInformationGain = maximumInformationGain === null
      ? value as number
      : Math.max(maximumInformationGain, value as number);
  }
  return { count: message.points.length, maximumInformationGain };
}

export function decodeAgentStatus(message: unknown): AgentStatusSnapshot | null {
  if (!isRecord(message) || !Array.isArray(message.robot_id) || !Array.isArray(message.status)) return null;
  if (message.robot_id.length !== message.status.length) return null;
  const alive = new Set(["ROBOT_STATUS_ACTIVE", "ROBOT_STATUS_INACTIVE"]);
  return {
    total: message.robot_id.length,
    active: message.status.filter((status) => typeof status === "string" && alive.has(status)).length
  };
}

export function decodeEmergencyObjects(message: unknown): EmergencyObjectSnapshot[] | null {
  if (!isRecord(message) || !Array.isArray(message.objects)) return null;
  return message.objects.flatMap((value) => {
    const decoded = decodeEmergencyObject(value);
    return decoded ? [decoded] : [];
  });
}

export function decodeEmergencyObject(message: unknown): EmergencyObjectSnapshot | null {
  if (!isRecord(message) || message.discovered !== true) return null;
  const kinds: Record<number, EmergencyObjectKind> = { 1: "victim", 2: "fire", 3: "box" };
  const objectType = finiteNumber(message.object_type);
  const id = finiteNumber(message.object_id);
  const position = decodeStampedPosition(message.precise_pose);
  const detectorRobot = message.detector_robot;
  if (objectType === null || !kinds[objectType] || id === null || !position || typeof detectorRobot !== "string") {
    return null;
  }
  const goal = message.has_goal_pose === true ? decodeStampedPosition(message.goal_pose) : null;
  if (message.has_goal_pose === true && !goal) return null;
  return { kind: kinds[objectType], id, detectorRobot, position, goal };
}

export function decodeEmergencyTasks(message: unknown): EmergencyTaskSnapshot[] | null {
  if (!isRecord(message) || !Array.isArray(message.tasks)) return null;
  return message.tasks.flatMap((value) => {
    const decoded = decodeEmergencyTask(value);
    return decoded ? [decoded] : [];
  });
}

export function decodeEmergencyTask(message: unknown): EmergencyTaskSnapshot | null {
  if (!isRecord(message)) return null;
  const taskId = integerNumber(message.task_id);
  const taskType = integerNumber(message.task_type);
  const status = integerNumber(message.status);
  const objectId = integerNumber(message.object_id);
  const detectorRobot = message.detector_robot;
  const behaviorType = message.behavior_type;
  const assignedRobots = message.assigned_robots;
  if (
    taskId === null || taskType === null || status === null || objectId === null
    || typeof detectorRobot !== "string" || typeof behaviorType !== "string"
    || !Array.isArray(assignedRobots) || assignedRobots.some((robot) => typeof robot !== "string")
  ) return null;
  const taskStatus: Record<number, ResponseTaskStatus> = {
    1: "new", 2: "dispatched", 3: "en_route", 4: "acting", 5: "done", 6: "failed"
  };
  const kind = taskKind(taskType);
  const decodedStatus = taskStatus[status];
  if (!kind || !decodedStatus) return null;
  const staging = decodeStampedPosition(message.staging_pose);
  const hasGoal = message.has_goal_pose === true;
  const goal = hasGoal ? decodeStampedPosition(message.goal_pose) : null;
  if (hasGoal && !goal) return null;
  return {
    taskId,
    kind,
    status: decodedStatus,
    objectId,
    detectorRobot,
    staging,
    goal,
    behaviorType,
    assignedRobots: assignedRobots as string[]
  };
}

export function decodeResponseStatus(message: unknown): ResponseStatusSnapshot | null {
  if (!isRecord(message)) return null;
  const state = integerNumber(message.state);
  const activeTaskIdRaw = integerNumber(message.active_task_id);
  const activeTaskType = integerNumber(message.active_task_type);
  const behaviorType = message.behavior_type;
  const detail = message.detail;
  const holdRemaining = finiteNumber(message.hold_remaining);
  const assignedRobots = message.assigned_robots;
  const planningState = message.planning_state;
  const longitudinalScale = finiteNumber(message.longitudinal_scale);
  const lateralScale = finiteNumber(message.lateral_scale);
  const rmsSlotError = finiteNumber(message.rms_slot_error);
  const maxSlotError = finiteNumber(message.max_slot_error);
  const minimumClearance = finiteNumber(message.minimum_clearance);
  const motionEnabled = typeof message.motion_enabled === "boolean" ? message.motion_enabled : null;
  if (
    state === null || activeTaskIdRaw === null || activeTaskType === null
    || typeof behaviorType !== "string" || typeof detail !== "string"
    || holdRemaining === null || !Array.isArray(assignedRobots)
    || assignedRobots.some((robot) => typeof robot !== "string")
    || typeof planningState !== "string" || longitudinalScale === null || lateralScale === null
    || rmsSlotError === null || maxSlotError === null || minimumClearance === null || motionEnabled === null
  ) return null;
  const stateLabels: Record<number, ResponseGroupState> = { 1: "idle", 2: "navigating", 3: "acting", 4: "done", 5: "failed" };
  const decodedState = stateLabels[state];
  if (!decodedState) return null;
  const activeTaskId = activeTaskIdRaw === -2147483648 ? null : activeTaskIdRaw;
  return {
    state: decodedState,
    activeTaskId,
    activeTaskKind: activeTaskId === null ? null : taskKind(activeTaskType),
    behaviorType,
    detail,
    formationCenter: decodeStampedPosition(message.formation_center),
    target: decodeStampedPosition(message.target_pose),
    holdRemaining,
    assignedRobots: assignedRobots as string[],
    planningState,
    longitudinalScale,
    lateralScale,
    rmsSlotError,
    maxSlotError,
    minimumClearance,
    motionEnabled
  };
}

export function decodeLaserScan(message: unknown): LaserSnapshot | null {
  if (!isRecord(message) || !Array.isArray(message.ranges)) return null;
  const minimum = finiteNumber(message.range_min);
  const maximum = finiteNumber(message.range_max);
  let validCount = 0;
  let closestMeters = Number.POSITIVE_INFINITY;
  let farthestMeters = Number.NEGATIVE_INFINITY;
  for (const value of message.ranges) {
    if (typeof value !== "number" || !Number.isFinite(value) || value < 0) continue;
    if (minimum !== null && value < minimum) continue;
    if (maximum !== null && value > maximum) continue;
    validCount += 1;
    closestMeters = Math.min(closestMeters, value);
    farthestMeters = Math.max(farthestMeters, value);
  }
  return {
    beamCount: message.ranges.length,
    validCount,
    closestMeters: validCount > 0 ? closestMeters : null,
    farthestMeters: validCount > 0 ? farthestMeters : null
  };
}

export function decodeOdometry(message: unknown): Pose2D | null {
  if (!isRecord(message) || !isRecord(message.pose) || !isRecord(message.pose.pose)) return null;
  const pose = message.pose.pose;
  if (!isRecord(pose.position) || !isRecord(pose.orientation)) return null;
  const x = finiteNumber(pose.position.x);
  const y = finiteNumber(pose.position.y);
  const qx = finiteNumber(pose.orientation.x);
  const qy = finiteNumber(pose.orientation.y);
  const qz = finiteNumber(pose.orientation.z);
  const qw = finiteNumber(pose.orientation.w);
  if ([x, y, qx, qy, qz, qw].some((value) => value === null)) return null;
  return {
    x: x!,
    y: y!,
    orientation: { x: qx!, y: qy!, z: qz!, w: qw! }
  };
}

export function decodeGlobalInfoRobotPoses(message: unknown): WorldRobotPoses | null {
  if (!isRecord(message) || typeof message.data !== "string") return null;
  let payload: unknown;
  try {
    payload = JSON.parse(message.data);
  } catch {
    return null;
  }
  if (!isRecord(payload) || !isRecord(payload.robots)) return null;

  const poses: WorldRobotPoses = {};
  for (const [name, value] of Object.entries(payload.robots)) {
    const robotKey = simulatorRobotKey(name, "robot");
    if (robotKey === null || !isRecord(value) || !isRecord(value.position)) continue;
    const x = finiteNumber(value.position.x);
    const y = finiteNumber(value.position.y);
    if (x !== null && y !== null) poses[robotKey] = { x, y };
  }
  return poses;
}

export function decodeGazeboModelPoses(message: unknown): WorldRobotPoses | null {
  if (!isRecord(message) || !Array.isArray(message.name) || !Array.isArray(message.pose)) return null;
  if (message.name.length !== message.pose.length) return null;

  const poses: WorldRobotPoses = {};
  for (let index = 0; index < message.name.length; index += 1) {
    const name = message.name[index];
    const pose = message.pose[index];
    if (typeof name !== "string") return null;
    const robotKey = simulatorRobotKey(name, "tb");
    if (robotKey === null || !isRecord(pose) || !isRecord(pose.position)) continue;
    const x = finiteNumber(pose.position.x);
    const y = finiteNumber(pose.position.y);
    if (x !== null && y !== null) poses[robotKey] = { x, y };
  }
  return poses;
}

export function decodeTwist(message: unknown): TwistSnapshot | null {
  if (!isRecord(message)) return null;
  const value = isRecord(message.twist) && isRecord(message.twist.linear)
    ? message.twist
    : message;
  if (!isRecord(value.linear) || !isRecord(value.angular)) return null;
  const x = finiteNumber(value.linear.x);
  const y = finiteNumber(value.linear.y);
  const z = finiteNumber(value.linear.z);
  const angularZ = finiteNumber(value.angular.z);
  if ([x, y, z, angularZ].some((item) => item === null)) return null;
  return { linearSpeed: Math.hypot(x!, y!, z!), angularZ: angularZ! };
}

export function decodeOpinion(message: unknown): OpinionSnapshot | null {
  if (!isRecord(message)) return null;
  const robotId = finiteNumber(message.id);
  const opinion = finiteNumber(message.opinion);
  if (robotId === null || opinion === null) return null;
  return { robotId, opinion };
}

export function decodeScalar(message: unknown): number | null {
  return isRecord(message) ? finiteNumber(message.data) : null;
}

export function decodeString(message: unknown): string | null {
  return isRecord(message) && typeof message.data === "string" ? message.data : null;
}

export function decodeBoolean(message: unknown): boolean | null {
  return isRecord(message) && typeof message.data === "boolean" ? message.data : null;
}

export function decodeRange(message: unknown): RangeSnapshot | null {
  if (!isRecord(message)) return null;
  const meters = finiteNumber(message.range);
  if (meters === null || meters < 0) return null;
  return {
    meters,
    minMeters: finiteNumber(message.min_range),
    maxMeters: finiteNumber(message.max_range)
  };
}

export function decodeImu(message: unknown): ImuSnapshot | null {
  if (!isRecord(message) || !isRecord(message.linear_acceleration) || !isRecord(message.angular_velocity)) {
    return null;
  }
  const accelerationX = finiteNumber(message.linear_acceleration.x);
  const accelerationY = finiteNumber(message.linear_acceleration.y);
  const accelerationZ = finiteNumber(message.linear_acceleration.z);
  const angularX = finiteNumber(message.angular_velocity.x);
  const angularY = finiteNumber(message.angular_velocity.y);
  const angularZ = finiteNumber(message.angular_velocity.z);
  if ([accelerationX, accelerationY, accelerationZ, angularX, angularY, angularZ].some((value) => value === null)) {
    return null;
  }
  return {
    acceleration: Math.hypot(accelerationX!, accelerationY!, accelerationZ!),
    angularSpeed: Math.hypot(angularX!, angularY!, angularZ!)
  };
}

function finiteNumber(value: unknown): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function simulatorRobotKey(name: string, prefix: "robot" | "tb"): string | null {
  const match = new RegExp(`(?:^|/)${prefix}_(\\d+)$`).exec(name);
  return match?.[1] ?? null;
}

function integerNumber(value: unknown): number | null {
  const number = finiteNumber(value);
  return number !== null && Number.isInteger(number) ? number : null;
}

function taskKind(value: number): ResponseTaskKind | null {
  return ({ 0: "home", 1: "victim", 2: "fire", 3: "box" } as Record<number, ResponseTaskKind>)[value] ?? null;
}

function decodeStampedPosition(value: unknown): { x: number; y: number } | null {
  if (!isRecord(value) || !isRecord(value.pose) || !isRecord(value.pose.position)) return null;
  const x = finiteNumber(value.pose.position.x);
  const y = finiteNumber(value.pose.position.y);
  return x === null || y === null ? null : { x, y };
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}
