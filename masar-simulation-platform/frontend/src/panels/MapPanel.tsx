import { useEffect, useRef, useState } from "react";
import type { DataAvailability, DataSource } from "../app/api";
import {
  decodeFrontiers,
  decodeScalar,
  decodeTaskAllocation,
  type FrontierSnapshot,
  type TaskAssignment
} from "../ros/decoders";
import { rosDataRuntime } from "../ros/RosDataRuntime";
import { runScopedSourceKey } from "../ros/sourceIdentity";
import { VideoClient } from "../ros/VideoClient";
import { PanelFrame } from "./PanelFrame";
import { sourceFor, StateBadge } from "./panelUtils";

interface MapPanelProps {
  runId: string | null;
  sources: DataSource[];
  running: boolean;
}

export function MapPanel({ runId, sources, running }: MapPanelProps) {
  const allocationImage = sourceFor(sources, "task_allocation_image");
  const allocation = sourceFor(sources, "task_allocation");
  const frontiers = sourceFor(sources, "frontiers");
  const progress = sourceFor(sources, "exploration_progress");
  const imageRef = useRef<HTMLImageElement>(null);
  const videoRef = useRef(new VideoClient());
  const [imageLoaded, setImageLoaded] = useState(false);
  const [imageRunId, setImageRunId] = useState<string | null>(null);
  const [imageFailed, setImageFailed] = useState(false);
  const [imageRetry, setImageRetry] = useState(0);
  const [assignments, setAssignments] = useState<TaskAssignment[] | null>(null);
  const [frontierSnapshot, setFrontierSnapshot] = useState<FrontierSnapshot | null>(null);
  const [progressValue, setProgressValue] = useState<number | null>(null);
  const imageBaseKey = runScopedSourceKey(runId, allocationImage);
  const imageKey = `${imageBaseKey}:${imageRetry}`;
  const allocationKey = runScopedSourceKey(runId, allocation);
  const frontierKey = runScopedSourceKey(runId, frontiers);
  const progressKey = runScopedSourceKey(runId, progress);
  const hasPresentedImage = running && imageLoaded && imageRunId === runId;

  useEffect(() => {
    setImageLoaded(false);
    setImageRunId(null);
    setImageFailed(false);
    setImageRetry(0);
    setAssignments(null);
    setFrontierSnapshot(null);
    setProgressValue(null);
  }, [runId]);

  useEffect(() => {
    const image = imageRef.current;
    if (!image || !allocationImage?.graph || !allocationImage.topic) return;
    setImageFailed(false);
    const source = { graph: allocationImage.graph, topic: allocationImage.topic };
    const retryKey = imageRetry > 0 ? imageRetry : undefined;
    if (!hasPresentedImage) {
      videoRef.current.snapshot(image, source, retryKey);
      return;
    }

    const candidate = new Image();
    candidate.onload = () => {
      if (candidate.naturalWidth <= 0) {
        setImageFailed(true);
        return;
      }
      videoRef.current.snapshot(image, source, retryKey);
      setImageFailed(false);
    };
    candidate.onerror = () => setImageFailed(true);
    videoRef.current.snapshot(candidate, source, retryKey);
    return () => {
      candidate.onload = null;
      candidate.onerror = null;
      videoRef.current.clear(candidate);
    };
  // The cache suffix retries or refreshes one low-frequency allocation snapshot.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [imageKey]);

  useEffect(() => {
    const image = imageRef.current;
    return () => {
      if (image) videoRef.current.clear(image);
    };
  }, [runId]);

  useEffect(() => {
    if (!allocationImage?.topic || !running) return;
    const timer = window.setTimeout(
      () => setImageRetry((current) => current + 1),
      hasPresentedImage ? 3000 : 1500
    );
    return () => window.clearTimeout(timer);
  }, [allocationImage?.topic, hasPresentedImage, imageKey, running]);

  useEffect(() => {
    setAssignments(null);
    if (!allocation?.topic) return;
    return rosDataRuntime.subscribe(allocation, (message) => setAssignments(decodeTaskAllocation(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [allocationKey]);

  useEffect(() => {
    setFrontierSnapshot(null);
    if (!frontiers?.topic) return;
    return rosDataRuntime.subscribe(frontiers, (message) => setFrontierSnapshot(decodeFrontiers(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [frontierKey]);

  useEffect(() => {
    setProgressValue(null);
    if (!progress?.topic) return;
    return rosDataRuntime.subscribe(progress, (message) => setProgressValue(decodeScalar(message)));
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [progressKey]);

  const presentedState: DataAvailability = hasPresentedImage
    ? allocationImage?.topic
      ? "ONLINE"
      : "STALE"
    : imageFailed
      ? "DISCONNECTED"
      : allocationImage?.availability ?? "WAITING";

  return (
    <PanelFrame
      panelId="map"
      eyebrow="TASK ALLOCATION"
      title="任务分配与建图"
      status={<StateBadge state={presentedState} />}
    >
      <div className="allocation-view">
        {allocationImage?.topic || hasPresentedImage ? (
          <img
            ref={imageRef}
            className="task-allocation-image"
            data-testid="task-allocation-image"
            alt="实时任务分配与建图结果"
            onLoad={(event) => {
              setImageLoaded(event.currentTarget.naturalWidth > 0);
              setImageRunId(runId);
              setImageFailed(false);
            }}
            onError={() => {
              setImageLoaded(false);
              setImageFailed(true);
            }}
          />
        ) : null}
        {!hasPresentedImage ? (
          <div className="allocation-empty">
            <strong>{allocationEmptyState(allocationImage, running, imageFailed)}</strong>
            <span>{allocationImage?.topic ?? "/task_allocation_image"}</span>
          </div>
        ) : null}
        <dl className="allocation-stats">
          <div><dt>地图覆盖</dt><dd data-testid="map-coverage">{percentOrDash(progressValue)}</dd></div>
          <div><dt>候选前沿</dt><dd data-testid="frontier-count">{valueOrDash(frontierSnapshot?.count)}</dd></div>
          <div><dt>本轮分配</dt><dd data-testid="map-assignment-count">{valueOrDash(assignments?.length)}</dd></div>
        </dl>
      </div>
    </PanelFrame>
  );
}

function allocationEmptyState(source: DataSource | undefined, running: boolean, failed: boolean): string {
  if (!running) return "等待运行";
  if (!source?.topic) return "未发现任务分配图";
  if (failed) return "任务分配图连接重试中";
  return "等待任务分配图首帧";
}

function valueOrDash(value: number | undefined | null): string {
  return value === undefined || value === null ? "—" : String(value);
}

function percentOrDash(value: number | null): string {
  return value === null ? "—" : `${(Math.max(0, Math.min(1, value)) * 100).toFixed(1)}%`;
}
