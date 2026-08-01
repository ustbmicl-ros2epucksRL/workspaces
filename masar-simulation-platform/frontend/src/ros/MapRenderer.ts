import { EventEmitter2 } from "eventemitter2";
import * as Roslib from "roslib";
import easelUrl from "easeljs/lib/easeljs.min.js?url";
import ros2dUrl from "ros2d/build/ros2d.min.js?url";
import { decodeOdometry } from "./decoders";

interface OccupancyGridMessage {
  info: {
    width: number;
    height: number;
    resolution: number;
    origin: {
      position: { x: number; y: number; z?: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: ArrayLike<number>;
}

interface LegacyScene {
  canvas: HTMLCanvasElement;
  addChild(child: unknown): void;
  removeChild(child: unknown): void;
  removeAllChildren(): void;
}

interface LegacyViewer {
  scene: LegacyScene;
  scaleToDimensions(width: number, height: number): void;
  shift(x: number, y: number): void;
}

interface LegacyGrid {
  width: number;
  height: number;
  pose: { position: { x: number; y: number } };
}

interface LegacyTrace {
  addPose(pose: unknown): void;
}

interface LegacyMarker {
  x: number;
  y: number;
  rotation: number;
}

interface LegacyPanView {
  startPan(x: number, y: number): void;
  pan(x: number, y: number): void;
}

interface LegacyZoomView {
  startZoom(x: number, y: number): void;
  zoom(scale: number): void;
}

interface Ros2DNamespace {
  Viewer: new (options: { divID: string; width: number; height: number; background: string }) => LegacyViewer;
  OccupancyGrid: new (options: { message: OccupancyGridMessage }) => LegacyGrid;
  TraceShape: new (options: {
    strokeSize: number;
    strokeColor: string;
    maxPoses: number;
    minDist: number;
  }) => LegacyTrace;
  ArrowShape: new (options: {
    size: number;
    strokeSize: number;
    strokeColor: string;
    fillColor: string;
    pulse: boolean;
  }) => LegacyMarker;
  PanView: new (options: { rootObject: LegacyScene }) => LegacyPanView;
  ZoomView: new (options: { rootObject: LegacyScene; minScale: number }) => LegacyZoomView;
}

interface LegacyWindow extends Window {
  ROSLIB?: typeof Roslib;
  ROS2D?: Ros2DNamespace;
  EventEmitter2?: typeof EventEmitter2;
  createjs?: {
    Ticker: { removeEventListener(event: string, listener: unknown): void };
  };
}

let rendererSequence = 0;
let ros2dPromise: Promise<Ros2DNamespace> | null = null;

/** Stable adapter around the legacy global ROS2D API. */
export class MapRenderer {
  private viewer: LegacyViewer | null = null;
  private currentGrid: LegacyGrid | null = null;
  private readonly traces = new Map<string, LegacyTrace>();
  private readonly markers = new Map<string, LegacyMarker>();
  private interactionCleanup: (() => void) | null = null;
  private destroyed = false;

  constructor(private readonly container: HTMLElement) {}

  async renderOccupancyGrid(message: unknown): Promise<boolean> {
    if (!isOccupancyGrid(message)) return false;
    const ros2d = await loadRos2D();
    if (this.destroyed) return false;
    const viewer = this.ensureViewer(ros2d);
    if (this.currentGrid) viewer.scene.removeChild(this.currentGrid);
    const grid = new ros2d.OccupancyGrid({ message });
    viewer.scene.addChild(grid);
    viewer.scaleToDimensions(grid.width, grid.height);
    viewer.shift(grid.pose.position.x, grid.pose.position.y);
    this.currentGrid = grid;
    return true;
  }

  async renderOdometry(robotId: string, message: unknown): Promise<boolean> {
    const decoded = decodeOdometry(message);
    if (!decoded) return false;
    const ros2d = await loadRos2D();
    if (this.destroyed) return false;
    const viewer = this.ensureViewer(ros2d, true);
    let trace = this.traces.get(robotId);
    if (!trace) {
      const color = colorFor(robotId);
      trace = new ros2d.TraceShape({
        strokeSize: 2,
        strokeColor: color,
        maxPoses: 500,
        minDist: 0.05
      });
      this.traces.set(robotId, trace);
      viewer.scene.addChild(trace);
      const marker = new ros2d.ArrowShape({
        size: 1.4,
        strokeSize: 0.1,
        strokeColor: color,
        fillColor: color,
        pulse: false
      });
      this.markers.set(robotId, marker);
      viewer.scene.addChild(marker);
    }
    trace.addPose(new Roslib.Pose({
      position: { x: decoded.x, y: decoded.y, z: 0 },
      orientation: decoded.orientation
    }));
    const marker = this.markers.get(robotId)!;
    marker.x = decoded.x;
    marker.y = -decoded.y;
    marker.rotation = -yawDegrees(decoded.orientation);
    return true;
  }

  destroy(): void {
    this.destroyed = true;
    this.interactionCleanup?.();
    this.interactionCleanup = null;
    if (this.viewer) {
      const legacyWindow = window as LegacyWindow;
      legacyWindow.createjs?.Ticker.removeEventListener("tick", this.viewer.scene);
      this.viewer.scene.removeAllChildren();
    }
    this.container.replaceChildren();
    this.viewer = null;
    this.currentGrid = null;
    this.traces.clear();
    this.markers.clear();
  }

  private ensureViewer(ros2d: Ros2DNamespace, trajectoryOnly = false): LegacyViewer {
    if (this.viewer) return this.viewer;
    if (!this.container.id) this.container.id = `masar-map-${++rendererSequence}`;
    const bounds = this.container.getBoundingClientRect();
    this.viewer = new ros2d.Viewer({
      divID: this.container.id,
      width: Math.max(320, Math.round(bounds.width || 640)),
      height: Math.max(220, Math.round(bounds.height || 360)),
      background: "#071016"
    });
    if (trajectoryOnly) {
      this.viewer.scaleToDimensions(20, 20);
      this.viewer.shift(-10, -10);
    }
    this.interactionCleanup = attachInteractions(this.viewer, ros2d);
    return this.viewer;
  }
}

function isOccupancyGrid(value: unknown): value is OccupancyGridMessage {
  if (!isRecord(value) || !isRecord(value.info)) return false;
  const info = value.info;
  if (!isRecord(info.origin) || !isRecord(info.origin.position) || !isRecord(info.origin.orientation)) {
    return false;
  }
  const width = positiveInteger(info.width);
  const height = positiveInteger(info.height);
  const resolution = finiteNumber(info.resolution);
  if (width === null || height === null || resolution === null || resolution <= 0) return false;
  if (width * height > 16_777_216) return false;
  const data = value.data;
  return Boolean(
    data &&
    typeof data === "object" &&
    "length" in data &&
    typeof data.length === "number" &&
    data.length >= width * height
  );
}

async function loadRos2D(): Promise<Ros2DNamespace> {
  if (ros2dPromise) return ros2dPromise;
  ros2dPromise = (async () => {
    const legacyWindow = window as LegacyWindow;
    legacyWindow.ROSLIB = Roslib;
    legacyWindow.EventEmitter2 = EventEmitter2;
    await loadScript(easelUrl, "easeljs");
    await loadScript(ros2dUrl, "ros2d");
    if (!legacyWindow.ROS2D) throw new Error("ROS2D 加载失败");
    return legacyWindow.ROS2D;
  })();
  return ros2dPromise;
}

function loadScript(src: string, name: string): Promise<void> {
  const existing = document.querySelector<HTMLScriptElement>(`script[data-masar-library="${name}"]`);
  if (existing?.dataset.loaded === "true") return Promise.resolve();
  return new Promise((resolve, reject) => {
    const script = existing ?? document.createElement("script");
    script.dataset.masarLibrary = name;
    script.onload = () => {
      script.dataset.loaded = "true";
      resolve();
    };
    script.onerror = () => reject(new Error(`${name} 加载失败`));
    if (!existing) {
      script.src = src;
      document.head.appendChild(script);
    }
  });
}

function attachInteractions(viewer: LegacyViewer, ros2d: Ros2DNamespace): () => void {
  const canvas = viewer.scene.canvas;
  const pan = new ros2d.PanView({ rootObject: viewer.scene });
  const zoom = new ros2d.ZoomView({ rootObject: viewer.scene, minScale: 0.01 });
  let panning = false;
  const pointerDown = (event: PointerEvent) => {
    panning = true;
    canvas.setPointerCapture(event.pointerId);
    pan.startPan(event.offsetX, event.offsetY);
  };
  const pointerMove = (event: PointerEvent) => {
    if (panning) pan.pan(event.offsetX, event.offsetY);
  };
  const pointerUp = (event: PointerEvent) => {
    panning = false;
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  };
  const wheel = (event: WheelEvent) => {
    event.preventDefault();
    zoom.startZoom(event.offsetX, event.offsetY);
    zoom.zoom(Math.exp(-event.deltaY * 0.001));
  };
  canvas.addEventListener("pointerdown", pointerDown);
  canvas.addEventListener("pointermove", pointerMove);
  canvas.addEventListener("pointerup", pointerUp);
  canvas.addEventListener("pointercancel", pointerUp);
  canvas.addEventListener("wheel", wheel, { passive: false });
  return () => {
    canvas.removeEventListener("pointerdown", pointerDown);
    canvas.removeEventListener("pointermove", pointerMove);
    canvas.removeEventListener("pointerup", pointerUp);
    canvas.removeEventListener("pointercancel", pointerUp);
    canvas.removeEventListener("wheel", wheel);
  };
}

function colorFor(value: string): string {
  const palette = ["#91f2d0", "#f0b85a", "#75b8ff", "#ff7d8d", "#c79cff", "#d7e36d"];
  let hash = 0;
  for (const character of value) hash = (hash * 31 + character.charCodeAt(0)) >>> 0;
  return palette[hash % palette.length];
}

function yawDegrees(orientation: { x: number; y: number; z: number; w: number }): number {
  const sinYaw = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const cosYaw = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return Math.atan2(sinYaw, cosYaw) * 180 / Math.PI;
}

function positiveInteger(value: unknown): number | null {
  return typeof value === "number" && Number.isInteger(value) && value > 0 ? value : null;
}

function finiteNumber(value: unknown): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}
