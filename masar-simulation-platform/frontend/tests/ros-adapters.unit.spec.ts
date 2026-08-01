import { describe, expect, it, vi } from "vitest";
import { RosConnectionManager, type RosGraph, type RosLike } from "../src/ros/RosConnectionManager";
import { TopicHealth } from "../src/ros/TopicHealth";
import {
  TopicRegistry,
  TopicTypeMismatch,
  type TopicContract,
  type TopicLike
} from "../src/ros/TopicRegistry";
import { TfClient } from "../src/ros/TfClient";
import { VideoClient } from "../src/ros/VideoClient";
import { RosDataRuntime } from "../src/ros/RosDataRuntime";
import { decodeLaserScan, decodeOdometry, decodeTwist } from "../src/ros/decoders";

class FakeRos implements RosLike {
  connectCalls: string[] = [];
  closeCalls = 0;
  private handlers = new Map<string, Array<(...args: unknown[]) => void>>();

  connect(url: string) {
    this.connectCalls.push(url);
  }

  close() {
    this.closeCalls += 1;
  }

  on(event: string, callback: (...args: unknown[]) => void) {
    this.handlers.set(event, [...(this.handlers.get(event) ?? []), callback]);
  }

  emit(event: string, ...args: unknown[]) {
    for (const callback of this.handlers.get(event) ?? []) callback(...args);
  }
}

class DiscoverableFakeRos extends FakeRos {
  constructor(private readonly result: { topics: string[]; types: string[] }) {
    super();
  }

  getTopics(callback: (result: { topics: string[]; types: string[] }) => void) {
    callback(this.result);
  }
}

class FakeTopic implements TopicLike {
  subscribeCalls = 0;
  unsubscribeCalls = 0;
  callback: ((message: unknown) => void) | null = null;

  constructor(readonly options: Record<string, unknown>) {}

  subscribe(callback: (message: unknown) => void) {
    this.subscribeCalls += 1;
    this.callback = callback;
  }

  unsubscribe() {
    this.unsubscribeCalls += 1;
    this.callback = null;
  }

  emit(message: unknown) {
    this.callback?.(message);
  }
}

describe("RosConnectionManager", () => {
  it("shares exactly one connection per ROS graph", () => {
    const created: Record<RosGraph, FakeRos[]> = { ros1: [], ros2: [] };
    const manager = new RosConnectionManager({
      origin: "http://platform.test:18090",
      createRos: (graph) => {
        const ros = new FakeRos();
        created[graph].push(ros);
        return ros;
      }
    });

    expect(manager.get("ros1")).toBe(manager.get("ros1"));
    expect(manager.get("ros2")).toBe(manager.get("ros2"));
    expect(created.ros1).toHaveLength(1);
    expect(created.ros2).toHaveLength(1);
    expect(created.ros1[0].connectCalls).toEqual(["ws://platform.test:18090/ros1/"]);
    expect(created.ros2[0].connectCalls).toEqual(["ws://platform.test:18090/ros2/"]);
    expect(manager.get("ros1")).not.toBe(manager.get("ros2"));
  });

  it("tracks connection, error and close without exposing a publisher", () => {
    const ros = new FakeRos();
    const manager = new RosConnectionManager({ createRos: () => ros });
    const states: string[] = [];
    manager.subscribeState("ros1", (state) => states.push(state));
    manager.get("ros1");

    ros.emit("connection");
    ros.emit("error", new Error("test"));
    ros.emit("close");

    expect(states).toEqual(["CONNECTING", "ONLINE", "ERROR", "DISCONNECTED"]);
    expect("publish" in manager).toBe(false);
  });

  it("reconnects the shared graph after a transient close", () => {
    vi.useFakeTimers();
    const ros = new FakeRos();
    const manager = new RosConnectionManager({ createRos: () => ros });
    manager.get("ros1");

    ros.emit("close");
    vi.advanceTimersByTime(1000);

    expect(ros.connectCalls).toEqual([
      "ws://127.0.0.1/ros1/",
      "ws://127.0.0.1/ros1/"
    ]);
    manager.close();
    vi.useRealTimers();
  });
});

describe("TopicRegistry and TopicHealth", () => {
  it("uses queue length one, one underlying subscription and a 10 Hz last-value cap", () => {
    vi.useFakeTimers();
    const ros = new FakeRos();
    const manager = new RosConnectionManager({ createRos: () => ros });
    let topic: FakeTopic | null = null;
    const registry = new TopicRegistry(
      manager,
      [
        {
          graph: "ros2",
          topic: "/robot_0/scan",
          messageType: "sensor_msgs/msg/LaserScan",
          staleAfterMs: 500
        }
      ],
      {
        createTopic: (_ros, options) => {
          topic = new FakeTopic(options);
          return topic;
        }
      }
    );
    const first: number[] = [];
    const second: number[] = [];
    const releaseFirst = registry.subscribe(
      { graph: "ros2", topic: "/robot_0/scan", messageType: "sensor_msgs/msg/LaserScan" },
      (message) => first.push((message as { value: number }).value)
    );
    const releaseSecond = registry.subscribe(
      { graph: "ros2", topic: "/robot_0/scan", messageType: "sensor_msgs/msg/LaserScan" },
      (message) => second.push((message as { value: number }).value)
    );

    expect(topic).not.toBeNull();
    expect(topic!.options).toMatchObject({ queue_length: 1, throttle_rate: 100 });
    expect(topic!.subscribeCalls).toBe(1);
    topic!.emit({ value: 1 });
    topic!.emit({ value: 2 });
    topic!.emit({ value: 3 });
    expect(first).toEqual([1]);
    vi.advanceTimersByTime(100);
    expect(first).toEqual([1, 3]);
    expect(second).toEqual([1, 3]);

    releaseFirst();
    expect(topic!.unsubscribeCalls).toBe(0);
    releaseSecond();
    expect(topic!.unsubscribeCalls).toBe(1);
    vi.useRealTimers();
  });

  it("rejects undeclared message types before creating a topic", () => {
    const manager = new RosConnectionManager({ createRos: () => new FakeRos() });
    const createTopic = vi.fn();
    const registry = new TopicRegistry(
      manager,
      [
        {
          graph: "ros1",
          topic: "/sim_map",
          messageType: "nav_msgs/OccupancyGrid",
          staleAfterMs: 2000
        }
      ],
      { createTopic }
    );

    expect(() =>
      registry.subscribe(
        { graph: "ros1", topic: "/sim_map", messageType: "std_msgs/String" },
        () => undefined
      )
    ).toThrow(TopicTypeMismatch);
    expect(createTopic).not.toHaveBeenCalled();
  });

  it("moves from waiting to online to stale and preserves type/disconnect failures", () => {
    let now = 1000;
    const health = new TopicHealth(500, () => now);
    expect(health.snapshot().availability).toBe("WAITING");
    health.observeMessage();
    expect(health.snapshot()).toMatchObject({ availability: "ONLINE", lastAgeMs: 0 });
    now = 1601;
    expect(health.snapshot()).toMatchObject({ availability: "STALE", lastAgeMs: 601 });
    health.markTypeMismatch();
    expect(health.snapshot().availability).toBe("TYPE_MISMATCH");
    health.markDisconnected();
    expect(health.snapshot().availability).toBe("DISCONNECTED");
  });
});

describe("RosDataRuntime", () => {
  it("resolves profile patterns to exact type-checked topics", async () => {
    const ros = new DiscoverableFakeRos({
      topics: ["/robot_0/scan", "/robot_1/scan", "/robot_2/scan"],
      types: [
        "sensor_msgs/msg/LaserScan",
        "sensor_msgs/msg/LaserScan",
        "std_msgs/msg/String"
      ]
    });
    const manager = new RosConnectionManager({ createRos: () => ros });
    const runtime = new RosDataRuntime(manager);
    runtime.configure("run-test", [
      {
        graph: "ros2",
        topic_pattern: "/<robot>/scan",
        message_types: ["sensor_msgs/msg/LaserScan"],
        capability: "laser",
        required: true,
        availability: "UNKNOWN"
      }
    ]);
    ros.emit("connection");
    await vi.waitFor(() => expect(runtime.getSnapshot()).toHaveLength(2));

    expect(runtime.getSnapshot()).toEqual([
      expect.objectContaining({
        topic: "/robot_0/scan",
        message_type: "sensor_msgs/msg/LaserScan",
        availability: "WAITING"
      }),
      expect.objectContaining({
        topic: "/robot_1/scan",
        message_type: "sensor_msgs/msg/LaserScan",
        availability: "WAITING"
      })
    ]);
  });

  it("reports a discovered name with an undeclared type without subscribing", async () => {
    const ros = new DiscoverableFakeRos({
      topics: ["/sim_map"],
      types: ["std_msgs/String"]
    });
    const manager = new RosConnectionManager({ createRos: () => ros });
    const runtime = new RosDataRuntime(manager);
    runtime.configure("run-test", [
      {
        graph: "ros1",
        topic_pattern: "/sim_map",
        message_types: ["nav_msgs/OccupancyGrid"],
        capability: "map",
        required: true,
        availability: "UNKNOWN"
      }
    ]);
    ros.emit("connection");
    await vi.waitFor(() =>
      expect(runtime.getSnapshot()[0]).toMatchObject({
        topic: "/sim_map",
        message_type: "std_msgs/String",
        availability: "TYPE_MISMATCH"
      })
    );
  });

  it("preserves active subscriptions when reconnect discovery resolves the same contracts", async () => {
    const ros = new DiscoverableFakeRos({
      topics: ["/robot_0/scan"],
      types: ["sensor_msgs/msg/LaserScan"]
    });
    const manager = new RosConnectionManager({ createRos: () => ros });
    const registries: FakeRegistry[] = [];
    const runtime = new RosDataRuntime(manager, {
      createRegistry: (_connections, contracts) => {
        const registry = new FakeRegistry(contracts);
        registries.push(registry);
        return registry;
      }
    });
    runtime.configure("run-test", [laserProfileSource()]);
    ros.emit("connection");
    await vi.waitFor(() => expect(runtime.getSnapshot()[0]?.topic).toBe("/robot_0/scan"));
    const release = runtime.subscribe(runtime.getSnapshot()[0], () => undefined);

    ros.emit("close");
    ros.emit("connection");
    await vi.waitFor(() => expect(runtime.getSnapshot()[0]?.availability).toBe("WAITING"));

    expect(registries).toHaveLength(1);
    expect(registries[0].closeCalls).toBe(0);
    expect(registries[0].activeSubscriptions).toBe(1);
    release();
  });

  it("stops health polling after the final topic subscription is released", async () => {
    vi.useFakeTimers();
    const ros = new DiscoverableFakeRos({
      topics: ["/robot_0/scan"],
      types: ["sensor_msgs/msg/LaserScan"]
    });
    const manager = new RosConnectionManager({ createRos: () => ros });
    const runtime = new RosDataRuntime(manager, {
      createRegistry: (_connections, contracts) => new FakeRegistry(contracts)
    });
    runtime.configure("run-test", [laserProfileSource()]);
    ros.emit("connection");
    await vi.waitFor(() => expect(runtime.getSnapshot()[0]?.topic).toBe("/robot_0/scan"));
    const release = runtime.subscribe(runtime.getSnapshot()[0], () => undefined);
    expect(vi.getTimerCount()).toBe(1);

    release();

    expect(vi.getTimerCount()).toBe(0);
    vi.useRealTimers();
  });

  it("does not emit store updates when a polled health snapshot is unchanged", async () => {
    vi.useFakeTimers();
    const ros = new DiscoverableFakeRos({
      topics: ["/robot_0/scan"],
      types: ["sensor_msgs/msg/LaserScan"]
    });
    const manager = new RosConnectionManager({ createRos: () => ros });
    const runtime = new RosDataRuntime(manager, {
      createRegistry: (_connections, contracts) => new FakeRegistry(contracts)
    });
    runtime.configure("run-test", [laserProfileSource()]);
    ros.emit("connection");
    await vi.waitFor(() => expect(runtime.getSnapshot()[0]?.topic).toBe("/robot_0/scan"));
    const release = runtime.subscribe(runtime.getSnapshot()[0], () => undefined);
    let emissions = 0;
    const releaseStore = runtime.subscribeStore(() => {
      emissions += 1;
    });

    vi.advanceTimersByTime(1500);

    expect(emissions).toBe(0);
    releaseStore();
    release();
    vi.useRealTimers();
  });

  it("keeps HTTP video health independent from topic polling", async () => {
    vi.useFakeTimers();
    const ros = new DiscoverableFakeRos({
      topics: ["/robot_0/camera", "/robot_0/scan"],
      types: ["sensor_msgs/msg/Image", "sensor_msgs/msg/LaserScan"]
    });
    const manager = new RosConnectionManager({ createRos: () => ros });
    const runtime = new RosDataRuntime(manager, {
      createRegistry: (_connections, contracts) => new FakeRegistry(contracts)
    });
    runtime.configure("run-test", [
      {
        graph: "ros2",
        topic_pattern: "/<robot>/camera",
        message_types: ["sensor_msgs/msg/Image"],
        capability: "camera",
        required: false,
        availability: "UNKNOWN"
      },
      laserProfileSource()
    ]);
    ros.emit("connection");
    await vi.waitFor(() => expect(runtime.getSnapshot()).toHaveLength(2));
    const camera = runtime.getSnapshot().find((source) => source.capability === "camera")!;
    const laser = runtime.getSnapshot().find((source) => source.capability === "laser")!;
    runtime.markObserved(camera);
    const release = runtime.subscribe(laser, () => undefined);

    vi.advanceTimersByTime(500);

    expect(runtime.getSnapshot().find((source) => source.capability === "camera")?.availability).toBe("ONLINE");
    runtime.markDisconnected(camera);
    expect(runtime.getSnapshot().find((source) => source.capability === "camera")?.availability).toBe("DISCONNECTED");
    release();
    vi.useRealTimers();
  });
});

describe("TF and video adapters", () => {
  it("subscribes only to explicitly requested TF frames and releases all of them", () => {
    const subscribed: string[] = [];
    const unsubscribed: string[] = [];
    const client = new TfClient(new FakeRos(), {
      fixedFrame: "map",
      createClient: (_ros, options) => {
        expect(options).toMatchObject({ fixedFrame: "map", rate: 10 });
        return {
          subscribe(frame: string) {
            subscribed.push(frame);
          },
          unsubscribe(frame: string) {
            unsubscribed.push(frame);
          },
          dispose() {}
        };
      }
    });

    const release = client.subscribe(
      ["robot_0/base_link", "robot_1/base_link", "robot_0/base_link"],
      () => undefined
    );
    expect(subscribed).toEqual(["robot_0/base_link", "robot_1/base_link"]);
    release();
    expect(unsubscribed).toEqual(["robot_0/base_link", "robot_1/base_link"]);
  });

  it("clears the previous video request synchronously before selecting another", () => {
    const assignments: string[] = [];
    const target = {
      get src() {
        return assignments.at(-1) ?? "";
      },
      set src(value: string) {
        assignments.push(value);
      },
      removeAttribute(name: string) {
        if (name === "src") assignments.push("REMOVED");
      }
    };
    const video = new VideoClient("http://platform.test:18090");

    video.select(target, { graph: "ros1", topic: "/robot_0/camera" });
    video.select(target, { graph: "ros1", topic: "/robot_1/camera" });

    expect(assignments).toEqual([
      "http://platform.test:18090/video/ros1/stream?topic=%2Frobot_0%2Fcamera&type=mjpeg&quality=70",
      "",
      "REMOVED",
      "http://platform.test:18090/video/ros1/stream?topic=%2Frobot_1%2Fcamera&type=mjpeg&quality=70"
    ]);
  });
});

describe("profile-approved message decoders", () => {
  it("keeps only finite LaserScan values inside the declared sensor range", () => {
    expect(
      decodeLaserScan({
        range_min: 0.1,
        range_max: 8,
        ranges: [0.05, 0.4, Number.POSITIVE_INFINITY, 2.2, 9]
      })
    ).toEqual({ beamCount: 5, validCount: 2, closestMeters: 0.4, farthestMeters: 2.2 });
    expect(decodeLaserScan({ ranges: "invalid" })).toBeNull();
  });

  it("decodes large LaserScan arrays without spreading them onto the call stack", () => {
    const ranges = Array.from({ length: 200_000 }, (_, index) => index === 123_456 ? 0.25 : 4);
    expect(decodeLaserScan({ range_min: 0.1, range_max: 8, ranges })).toMatchObject({
      beamCount: 200_000,
      validCount: 200_000,
      closestMeters: 0.25,
      farthestMeters: 4
    });
  });

  it("validates Odometry and Twist shapes before exposing numeric values", () => {
    expect(
      decodeOdometry({
        pose: {
          pose: {
            position: { x: 1.5, y: -2 },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
          }
        }
      })
    ).toMatchObject({ x: 1.5, y: -2 });
    expect(
      decodeTwist({
        linear: { x: 3, y: 4, z: 0 },
        angular: { z: 0.25 }
      })
    ).toEqual({ linearSpeed: 5, angularZ: 0.25 });
    expect(decodeOdometry({ pose: {} })).toBeNull();
    expect(decodeTwist({ linear: { x: Number.NaN } })).toBeNull();
  });
});

class FakeRegistry {
  readonly healthByContract = new Map<string, TopicHealth>();
  activeSubscriptions = 0;
  closeCalls = 0;

  constructor(contracts: TopicContract[]) {
    for (const contract of contracts) {
      this.healthByContract.set(contractKey(contract), new TopicHealth(contract.staleAfterMs ?? 2000));
    }
  }

  subscribe(contract: TopicContract, _callback: (message: unknown) => void) {
    if (!this.healthByContract.has(contractKey(contract))) throw new TopicTypeMismatch(contract);
    this.activeSubscriptions += 1;
    let active = true;
    return () => {
      if (!active) return;
      active = false;
      this.activeSubscriptions -= 1;
    };
  }

  health(contract: TopicContract) {
    const health = this.healthByContract.get(contractKey(contract));
    if (!health) throw new TopicTypeMismatch(contract);
    return health;
  }

  close() {
    this.closeCalls += 1;
    this.activeSubscriptions = 0;
  }
}

function laserProfileSource() {
  return {
    graph: "ros2" as const,
    topic_pattern: "/<robot>/scan",
    message_types: ["sensor_msgs/msg/LaserScan"],
    capability: "laser",
    required: true,
    availability: "UNKNOWN" as const
  };
}

function contractKey(contract: TopicContract): string {
  return [contract.graph, contract.topic, contract.messageType].join(":");
}
