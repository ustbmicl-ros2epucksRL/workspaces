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

export interface TwistSnapshot {
  linearSpeed: number;
  angularZ: number;
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

function finiteNumber(value: unknown): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}
