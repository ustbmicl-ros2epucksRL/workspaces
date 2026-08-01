import { ROS2TFClient, TFClient as Ros1TFClient, type Ros, type Transform } from "roslib";
import type { RosGraph, RosLike } from "./RosConnectionManager";

interface UnderlyingTfClient {
  subscribe(frame: string, callback: (transform: unknown) => void): void;
  unsubscribe(frame: string, callback?: (transform: unknown) => void): void;
  dispose(): void;
}

interface TfClientOptions {
  fixedFrame: string;
  graph?: RosGraph;
  createClient?: (
    ros: RosLike,
    options: { fixedFrame: string; rate: number }
  ) => UnderlyingTfClient;
}

export class TfClient {
  private readonly client: UnderlyingTfClient;

  constructor(ros: RosLike, options: TfClientOptions) {
    this.client = options.createClient
      ? options.createClient(ros, { fixedFrame: options.fixedFrame, rate: 10 })
      : createTfClient(ros, {
          fixedFrame: options.fixedFrame,
          rate: 10,
          graph: options.graph ?? "ros1"
        });
  }

  subscribe(frames: string[], callback: (frame: string, transform: unknown) => void): () => void {
    const uniqueFrames = [...new Set(frames)].filter(isSafeFrame);
    const callbacks = new Map<string, (transform: unknown) => void>();
    for (const frame of uniqueFrames) {
      const frameCallback = (transform: unknown) => callback(frame, transform);
      callbacks.set(frame, frameCallback);
      this.client.subscribe(frame, frameCallback);
    }
    let active = true;
    return () => {
      if (!active) return;
      active = false;
      for (const [frame, frameCallback] of callbacks) {
        this.client.unsubscribe(frame, frameCallback);
      }
    };
  }

  dispose(): void {
    this.client.dispose();
  }
}

function isSafeFrame(frame: string): boolean {
  return /^[A-Za-z][A-Za-z0-9_\-/]*$/.test(frame) && !frame.includes("//") && !frame.includes("..");
}

function createTfClient(
  ros: RosLike,
  options: { fixedFrame: string; rate: number; graph?: RosGraph }
): UnderlyingTfClient {
  const Client = options.graph === "ros2" ? ROS2TFClient : Ros1TFClient;
  const client = new Client({ ros: ros as Ros, fixedFrame: options.fixedFrame, rate: options.rate });
  return {
    subscribe(frame, callback) {
      client.subscribe(frame, callback as (transform: Transform) => void);
    },
    unsubscribe(frame, callback) {
      client.unsubscribe(frame, callback as ((transform: Transform) => void) | undefined);
    },
    dispose() {
      client.dispose();
    }
  };
}
