import type { DataSource } from "../app/api";
import { PanelFrame } from "./PanelFrame";
import { availabilityLabels, sourceFor, StateBadge } from "./panelUtils";

export function SensorPanel({ sources, running }: { sources: DataSource[]; running: boolean }) {
  const camera = sourceFor(sources, "camera");
  const laser = sourceFor(sources, "laser");
  const cameraState = camera?.availability ?? (running ? "UNAVAILABLE" : "WAITING");
  return (
    <PanelFrame
      panelId="sensors"
      eyebrow="SENSOR DECK"
      title="传感器观测"
      status={<StateBadge state={cameraState} />}
    >
      <div className="sensor-deck">
        <div className="sensor-camera">
          <div className="camera-reticle" aria-hidden="true" />
          <span>主相机</span>
          <strong data-testid="sensor-camera-state">{availabilityLabels[cameraState]}</strong>
        </div>
        <div className="sensor-readouts">
          <div>
            <span>LaserScan</span>
            <strong>{laser ? availabilityLabels[laser.availability] : running ? "未提供" : "等待运行"}</strong>
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
