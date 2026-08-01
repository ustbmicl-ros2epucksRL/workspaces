import { useEffect, useRef, useState } from "react";
import type { DataSource } from "../app/api";
import { decodeLaserScan, type LaserSnapshot } from "../ros/decoders";
import { rosDataRuntime } from "../ros/RosDataRuntime";
import { VideoClient } from "../ros/VideoClient";
import { PanelFrame } from "./PanelFrame";
import { availabilityLabels, sourceFor, StateBadge } from "./panelUtils";

export function SensorPanel({ sources, running }: { sources: DataSource[]; running: boolean }) {
  const camera = sourceFor(sources, "camera");
  const laser = sourceFor(sources, "laser");
  const imageRef = useRef<HTMLImageElement>(null);
  const videoRef = useRef(new VideoClient());
  const [cameraFailed, setCameraFailed] = useState(false);
  const [laserSnapshot, setLaserSnapshot] = useState<LaserSnapshot | null>(null);
  const cameraState = cameraFailed
    ? "DISCONNECTED"
    : camera?.availability ?? (running ? "UNAVAILABLE" : "WAITING");
  const cameraKey = camera ? [camera.graph, camera.topic].join(":") : "";
  const laserKey = laser ? [laser.graph, laser.topic, laser.message_type].join(":") : "";

  useEffect(() => {
    const image = imageRef.current;
    if (!image || !camera?.graph || !camera.topic) return;
    setCameraFailed(false);
    videoRef.current.select(image, { graph: camera.graph, topic: camera.topic });
    return () => videoRef.current.clear(image);
  // A source switch must synchronously clear the previous HTTP request.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cameraKey]);

  useEffect(() => {
    setLaserSnapshot(null);
    if (!laser?.topic) return;
    return rosDataRuntime.subscribe(laser, (message) => {
      setLaserSnapshot(decodeLaserScan(message));
    });
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [laserKey]);

  return (
    <PanelFrame
      panelId="sensors"
      eyebrow="SENSOR DECK"
      title="传感器观测"
      status={<StateBadge state={cameraState} />}
    >
      <div className="sensor-deck">
        <div className="sensor-camera">
          {camera?.topic ? (
            <img
              ref={imageRef}
              className="camera-stream"
              alt={`${camera.topic} 实时图像`}
              onLoad={() => {
                setCameraFailed(false);
                rosDataRuntime.markObserved(camera);
              }}
              onError={() => {
                setCameraFailed(true);
                rosDataRuntime.markDisconnected(camera);
              }}
            />
          ) : null}
          <div className="camera-reticle" aria-hidden="true" />
          <span>主相机</span>
          <strong data-testid="sensor-camera-state">{availabilityLabels[cameraState]}</strong>
        </div>
        <div className="sensor-readouts">
          <div>
            <span>LaserScan</span>
            <strong>{laserReading(laser, laserSnapshot, running)}</strong>
          </div>
          <div>
            <span>数据原则</span>
            <strong>仅显示已观测值</strong>
          </div>
        </div>
      </div>
    </PanelFrame>
  );
}

function laserReading(
  laser: DataSource | undefined,
  snapshot: LaserSnapshot | null,
  running: boolean
): string {
  if (snapshot?.closestMeters !== null && snapshot?.closestMeters !== undefined) {
    return `最近 ${snapshot.closestMeters.toFixed(2)} m · ${snapshot.validCount}/${snapshot.beamCount}`;
  }
  return laser ? availabilityLabels[laser.availability] : running ? "未提供" : "等待运行";
}
