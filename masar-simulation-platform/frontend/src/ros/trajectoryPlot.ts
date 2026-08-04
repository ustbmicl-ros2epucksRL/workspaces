export interface TrajectoryPoint {
  x: number;
  y: number;
}

export type RobotTrajectories = Record<string, TrajectoryPoint[]>;

export interface TrajectoryBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

const MIN_SPAN_METERS = 2;
const BOUNDS_MARGIN_RATIO = 0.1;

export function appendTrajectoryPoint(
  trajectories: RobotTrajectories,
  robotKey: string,
  point: TrajectoryPoint,
  maxPoints: number
): RobotTrajectories {
  const current = trajectories[robotKey] ?? [];
  const next = [...current, point].slice(-Math.max(1, maxPoints));
  return { ...trajectories, [robotKey]: next };
}

export function trajectoryBounds(trajectories: RobotTrajectories): TrajectoryBounds {
  const points = Object.values(trajectories).flat();
  if (points.length === 0) return { minX: -1, maxX: 1, minY: -1, maxY: 1 };

  const minX = Math.min(...points.map((point) => point.x));
  const maxX = Math.max(...points.map((point) => point.x));
  const minY = Math.min(...points.map((point) => point.y));
  const maxY = Math.max(...points.map((point) => point.y));
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  const span = Math.max(maxX - minX, maxY - minY, MIN_SPAN_METERS);
  const halfSpan = span / 2 + span * BOUNDS_MARGIN_RATIO;
  return {
    minX: centerX - halfSpan,
    maxX: centerX + halfSpan,
    minY: centerY - halfSpan,
    maxY: centerY + halfSpan
  };
}

export function projectTrajectoryPoint(
  point: TrajectoryPoint,
  bounds: TrajectoryBounds,
  width: number,
  height: number,
  padding: number
): TrajectoryPoint {
  const drawableWidth = Math.max(1, width - padding * 2);
  const drawableHeight = Math.max(1, height - padding * 2);
  const spanX = Math.max(Number.EPSILON, bounds.maxX - bounds.minX);
  const spanY = Math.max(Number.EPSILON, bounds.maxY - bounds.minY);
  const scale = Math.min(drawableWidth / spanX, drawableHeight / spanY);
  const plotWidth = spanX * scale;
  const plotHeight = spanY * scale;
  const left = (width - plotWidth) / 2;
  const top = (height - plotHeight) / 2;
  return {
    x: left + (point.x - bounds.minX) * scale,
    y: top + (bounds.maxY - point.y) * scale
  };
}
