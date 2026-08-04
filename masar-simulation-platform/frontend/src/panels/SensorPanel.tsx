import { useEffect, useMemo, useRef, useState } from "react";
import type { DataSource } from "../app/api";
import {
  decodeImu,
  decodeLaserScan,
  decodeOdometry,
  decodeOpinion,
  decodeRange,
  decodeScalar,
  decodeTwist,
  type ImuSnapshot,
  type LaserSnapshot,
  type OpinionSnapshot,
  type Pose2D,
  type RangeSnapshot,
  type TwistSnapshot
} from "../ros/decoders";
import {
  robotKeyForSource,
  robotOptions,
  robotSupportsCapability,
  shouldSplitRobotGroups,
  sourceForRobot
} from "../ros/robotSources";
import { rosDataRuntime } from "../ros/RosDataRuntime";
import { runScopedSourceKey } from "../ros/sourceIdentity";
import { VideoClient } from "../ros/VideoClient";
import { PanelFrame } from "./PanelFrame";
import { sourceFor } from "./panelUtils";

interface SensorPanelProps {
  runId: string | null;
  sources: DataSource[];
  running: boolean;
  expectedRobotCount: number;
  robotType: string | null;
}

const CAMERA_FIRST_FRAME_RETRY_MS = 1500;

export function SensorPanel({
  runId,
  sources,
  running,
  expectedRobotCount,
  robotType
}: SensorPanelProps) {
  const robots = useMemo(
    () => robotOptions(sources, expectedRobotCount),
    [sources, expectedRobotCount]
  );
  const splitRobotGroups = useMemo(
    () => shouldSplitRobotGroups(sources, expectedRobotCount),
    [sources, expectedRobotCount]
  );
  const [selectedRobot, setSelectedRobot] = useState<string | null>(null);
  const selectedRobotKey = robots.some((robot) => robot.key === selectedRobot)
    ? selectedRobot
    : robots[0]?.key ?? null;
  const discoveredCamera = sourceForRobot(sources, "camera", selectedRobotKey);
  const cameraSupported = Boolean(discoveredCamera?.topic) || robotSupportsCapability(robotType, "camera");
  const camera = cameraSupported ? discoveredCamera : undefined;
  const laser = sourceForRobot(sources, "laser", selectedRobotKey);
  const discoveredImu = sourceForRobot(sources, "imu", selectedRobotKey);
  const selectedRobotType = selectedRobotKey?.startsWith("agent:") ? null : robotType;
  const imuSupported = Boolean(discoveredImu?.topic)
    || selectedRobotType === "burger";
  const imu = imuSupported ? discoveredImu : undefined;
  const odometry = sourceForRobot(sources, "odometry", selectedRobotKey);
  const commandVelocity = sourceForRobot(sources, "command_velocity", selectedRobotKey);
  const opinion = sourceForRobot(sources, "discussion_opinion", selectedRobotKey);
  const dispersionDistance = sourceForRobot(sources, "dispersion_distance", selectedRobotKey);
  const discussionBroadcast = sourceFor(sources, "discussion_broadcast");
  const infrared = sources.filter(
    (source) => source.capability === "infrared" && robotKeyForSource(source, splitRobotGroups) === selectedRobotKey
  );
  const infraredSupported = infrared.some((source) => Boolean(source.topic))
    || selectedRobotType === "e_puck";
  const imageRef = useRef<HTMLImageElement>(null);
  const videoRef = useRef(new VideoClient());
  const [cameraLoaded, setCameraLoaded] = useState(false);
  const [cameraFailed, setCameraFailed] = useState(false);
  const [cameraRetry, setCameraRetry] = useState(0);
  const [laserSnapshot, setLaserSnapshot] = useState<LaserSnapshot | null>(null);
  const [imuSnapshot, setImuSnapshot] = useState<ImuSnapshot | null>(null);
  const [pose, setPose] = useState<Pose2D | null>(null);
  const [velocity, setVelocity] = useState<TwistSnapshot | null>(null);
  const [opinionSnapshot, setOpinionSnapshot] = useState<OpinionSnapshot | null>(null);
  const [distanceSnapshot, setDistanceSnapshot] = useState<number | null>(null);
  const [broadcastSnapshot, setBroadcastSnapshot] = useState<OpinionSnapshot | null>(null);
  const [infraredSnapshots, setInfraredSnapshots] = useState<Record<string, RangeSnapshot>>({});
  const cameraBaseKey = runScopedSourceKey(runId, camera);
  const cameraKey = `${cameraBaseKey}:${cameraRetry}`;
  const cameraPresented = cameraLoaded;
  const laserKey = runScopedSourceKey(runId, laser);
  const imuKey = runScopedSourceKey(runId, imu);
  const odometryKey = runScopedSourceKey(runId, odometry);
  const commandVelocityKey = runScopedSourceKey(runId, commandVelocity);
  const opinionKey = runScopedSourceKey(runId, opinion);
  const dispersionDistanceKey = runScopedSourceKey(runId, dispersionDistance);
  const discussionBroadcastKey = runScopedSourceKey(runId, discussionBroadcast);
  const infraredKey = `${runId ?? "idle"}:${infrared.map((source) => runScopedSourceKey(runId, source)).join("|")}`;

  useEffect(() => {
    setSelectedRobot(null);
  }, [runId]);

  useEffect(() => {
    setCameraRetry(0);
    setCameraLoaded(false);
    setCameraFailed(false);
  }, [cameraBaseKey]);

  useEffect(() => {
    const image = imageRef.current;
    if (!image || !camera?.graph || !camera.topic) return;
    setCameraFailed(false);
    videoRef.current.select(
      image,
      { graph: camera.graph, topic: camera.topic },
      cameraRetry > 0 ? cameraRetry : undefined
    );
    return () => videoRef.current.clear(image);
  // A source or Run switch must synchronously clear the previous HTTP request.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cameraKey]);

  useEffect(() => {
    if (cameraLoaded || !camera?.topic || !running) return;
    const timer = window.setTimeout(() => {
      if ((imageRef.current?.naturalWidth ?? 0) > 0) {
        setCameraLoaded(true);
        setCameraFailed(false);
        return;
      }
      setCameraRetry((current) => current + 1);
    }, CAMERA_FIRST_FRAME_RETRY_MS);
    return () => window.clearTimeout(timer);
  }, [cameraLoaded, camera?.topic, cameraKey, running]);

  useEffect(() => {
    setLaserSnapshot(null);
    if (!laser?.topic) return;
    return rosDataRuntime.subscribe(laser, (message) => setLaserSnapshot(decodeLaserScan(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [laserKey]);

  useEffect(() => {
    setImuSnapshot(null);
    if (!imu?.topic) return;
    return rosDataRuntime.subscribe(imu, (message) => setImuSnapshot(decodeImu(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [imuKey]);

  useEffect(() => {
    setPose(null);
    if (!odometry?.topic) return;
    return rosDataRuntime.subscribe(odometry, (message) => setPose(decodeOdometry(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [odometryKey]);

  useEffect(() => {
    setVelocity(null);
    if (!commandVelocity?.topic) return;
    return rosDataRuntime.subscribe(commandVelocity, (message) => setVelocity(decodeTwist(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [commandVelocityKey]);

  useEffect(() => {
    setOpinionSnapshot(null);
    if (!opinion?.topic) return;
    return rosDataRuntime.subscribe(opinion, (message) => setOpinionSnapshot(decodeOpinion(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [opinionKey]);

  useEffect(() => {
    setDistanceSnapshot(null);
    if (!dispersionDistance?.topic) return;
    return rosDataRuntime.subscribe(dispersionDistance, (message) => setDistanceSnapshot(decodeScalar(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dispersionDistanceKey]);

  useEffect(() => {
    setBroadcastSnapshot(null);
    if (!discussionBroadcast?.topic) return;
    return rosDataRuntime.subscribe(discussionBroadcast, (message) => setBroadcastSnapshot(decodeOpinion(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [discussionBroadcastKey]);

  useEffect(() => {
    setInfraredSnapshots({});
    return releaseAll(infrared.map((source) => rosDataRuntime.subscribe(source, (message) => {
      const snapshot = decodeRange(message);
      if (snapshot) {
        setInfraredSnapshots((current) => ({ ...current, [runScopedSourceKey(runId, source)]: snapshot }));
      }
    })));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [infraredKey]);

  return (
    <PanelFrame panelId="sensors" eyebrow="SENSOR DECK" title="传感器观测">
      <div className="sensor-deck">
        <div className="sensor-camera">
          {camera?.topic ? (
            <>
              <img
                ref={imageRef}
                className="camera-stream"
                alt={`${camera.topic} 实时图像`}
                onLoad={() => {
                  setCameraLoaded(true);
                  setCameraFailed(false);
                }}
                onError={() => {
                  setCameraLoaded(false);
                  setCameraFailed(true);
                }}
              />
              {!cameraPresented ? (
                <div className="sensor-camera-empty sensor-camera-empty--overlay">
                  <span>机载相机</span>
                  <strong data-testid="sensor-camera-state">
                    {cameraFailed ? "视频连接重试中" : "等待相机首帧"}
                  </strong>
                </div>
              ) : null}
            </>
          ) : (
            <div className="sensor-camera-empty">
              <span>机载相机</span>
              <strong data-testid="sensor-camera-state">
                {cameraEmptyState(cameraSupported, running, robotType)}
              </strong>
            </div>
          )}
        </div>
        <div className="sensor-readouts">
          <label className="sensor-robot-select">
            <span>观测机器人</span>
            <select
              aria-label="选择观测机器人"
              value={selectedRobotKey ?? ""}
              disabled={robots.length === 0}
              onChange={(event) => setSelectedRobot(event.target.value)}
            >
              {robots.length === 0 ? <option value="">未发现机器人</option> : null}
              {robots.map((robot) => (
                <option key={robot.key} value={robot.key}>{robot.label}</option>
              ))}
            </select>
          </label>
          <div>
            <span>相机数据源</span>
            <strong data-testid="sensor-source-topic">
              {cameraSourceLabel(camera, cameraSupported, running)}
            </strong>
          </div>
          <div>
            <span>LaserScan</span>
            <strong>{laserReading(laser, laserSnapshot, running)}</strong>
          </div>
          {infraredSupported ? (
            <div>
              <span>红外距离</span>
              <strong data-testid="infrared-reading">
                {infraredReading(infrared, infraredSnapshots, running, runId)}
              </strong>
            </div>
          ) : null}
          {imuSupported ? (
            <div>
              <span>IMU</span>
              <strong data-testid="imu-reading">{imuReading(imu, imuSnapshot, running)}</strong>
            </div>
          ) : null}
          <div>
            <span>Odometry</span>
            <strong>{poseReading(odometry, pose, running)}</strong>
          </div>
          <div>
            <span>cmd_vel</span>
            <strong>{velocityReading(commandVelocity, velocity, running)}</strong>
          </div>
          {opinion?.topic || dispersionDistance?.topic || discussionBroadcast?.topic ? (
            <div className="discussion-state" data-testid="discussion-state">
              <span>讨论状态</span>
              <strong>{discussionPhase(opinion, dispersionDistance, opinionSnapshot, distanceSnapshot)}</strong>
              <small>{discussionReading(opinion, opinionSnapshot, distanceSnapshot, broadcastSnapshot)}</small>
            </div>
          ) : null}
        </div>
      </div>
    </PanelFrame>
  );
}

function cameraEmptyState(supported: boolean, running: boolean, robotType: string | null): string {
  if (!running) return "等待运行";
  if (!supported) return robotType === "burger" ? "Burger 未配备相机" : "未配备相机";
  return "未发现相机话题";
}

function cameraSourceLabel(source: DataSource | undefined, supported: boolean, running: boolean): string {
  if (!running) return "等待运行";
  if (!supported) return "未配备";
  if (source?.topic) return source.topic;
  return "未发现话题";
}

function laserReading(
  laser: DataSource | undefined,
  snapshot: LaserSnapshot | null,
  running: boolean
): string {
  if (snapshot?.closestMeters !== null && snapshot?.closestMeters !== undefined) {
    return `最近 ${snapshot.closestMeters.toFixed(2)} m · ${snapshot.validCount}/${snapshot.beamCount}`;
  }
  return missingReading(laser, running);
}

function imuReading(source: DataSource | undefined, snapshot: ImuSnapshot | null, running: boolean): string {
  if (snapshot) {
    return `加速度 ${snapshot.acceleration.toFixed(2)} m/s² · 角速度 ${snapshot.angularSpeed.toFixed(2)} rad/s`;
  }
  return missingReading(source, running);
}

function poseReading(
  source: DataSource | undefined,
  pose: Pose2D | null,
  running: boolean
): string {
  if (pose) return `x ${pose.x.toFixed(2)} m · y ${pose.y.toFixed(2)} m`;
  return missingReading(source, running);
}

function velocityReading(
  source: DataSource | undefined,
  velocity: TwistSnapshot | null,
  running: boolean
): string {
  if (velocity) return `线 ${velocity.linearSpeed.toFixed(2)} m/s · 角 ${velocity.angularZ.toFixed(2)} rad/s`;
  return missingReading(source, running);
}

function missingReading(source: DataSource | undefined, running: boolean): string {
  if (!running) return "等待运行";
  if (!source?.topic) return "未发现话题";
  if (source.availability === "TYPE_MISMATCH") return "类型不匹配";
  if (source.availability === "DISCONNECTED") return "连接中断";
  if (source.availability === "STALE") return "数据已陈旧";
  if (source.availability === "UNAVAILABLE") return "未提供";
  return "等待首帧";
}

function infraredReading(
  sources: DataSource[],
  snapshots: Record<string, RangeSnapshot>,
  running: boolean,
  runId: string | null
): string {
  const values = Object.values(snapshots);
  if (values.length > 0) {
    const nearest = Math.min(...values.map((value) => value.meters));
    return `${sources.length} 路 · 最近 ${nearest.toFixed(3)} m`;
  }
  if (!running) return "等待运行";
  if (sources.length === 0) return "未发现话题";
  const invalid = sources.find((source) => ["TYPE_MISMATCH", "DISCONNECTED", "STALE"].includes(source.availability));
  if (invalid) return `${sources.length} 路 · ${missingReading(invalid, running)}`;
  const discovered = sources.filter((source) => runScopedSourceKey(runId, source)).length;
  return `${discovered} 路 · 等待首帧`;
}

function releaseAll(releases: Array<() => void>): () => void {
  return () => releases.forEach((release) => release());
}

function discussionPhase(
  opinion: DataSource | undefined,
  distance: DataSource | undefined,
  opinionSnapshot: OpinionSnapshot | null,
  distanceSnapshot: number | null
): string {
  if (distanceSnapshot !== null || distance?.availability === "ONLINE") return "分散执行中";
  if (opinionSnapshot || opinion?.availability === "ONLINE") return "观点交换中";
  return "等待讨论数据";
}

function discussionReading(
  source: DataSource | undefined,
  opinion: OpinionSnapshot | null,
  distance: number | null,
  broadcast: OpinionSnapshot | null
): string {
  const values = [source?.topic];
  if (opinion) values.push(`观点 ${opinion.opinion}`);
  if (distance !== null) values.push(`目标间距 ${distance.toFixed(2)} m`);
  if (broadcast) values.push(`总线 robot_${broadcast.robotId}: ${broadcast.opinion}`);
  return values.filter(Boolean).join(" · ");
}
