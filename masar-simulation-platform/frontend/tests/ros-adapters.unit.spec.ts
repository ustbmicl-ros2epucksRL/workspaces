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
import {
  decodeAgentStatus,
  decodeBoolean,
  decodeEmergencyTask,
  decodeEmergencyTasks,
  decodeEmergencyObjects,
  decodeFrontiers,
  decodeGazeboModelPoses,
  decodeGlobalInfoRobotPoses,
  decodeLaserScan,
  decodeImu,
  decodeOdometry,
  decodeOpinion,
  decodeRange,
  decodeResponseStatus,
  decodeScalar,
  decodeString,
  decodeTaskAllocation,
  decodeTwist
} from "../src/ros/decoders";
import {
  robotKeyForSource,
  robotOptions,
  robotSupportsCapability,
  sourceForRobot
} from "../src/ros/robotSources";
import {
  appendTrajectoryPoint,
  projectTrajectoryPoint,
  trajectoryBounds
} from "../src/ros/trajectoryPlot";
import { runScopedSourceKey } from "../src/ros/sourceIdentity";

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
  constructor(private result: { topics: string[]; types: string[] }) {
    super();
  }

  getTopics(callback: (result: { topics: string[]; types: string[] }) => void) {
    callback(this.result);
  }

  setTopics(result: { topics: string[]; types: string[] }) {
    this.result = result;
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

describe("simulator world pose decoders", () => {
  it("decodes Webots Supervisor positions from the global-info JSON envelope", () => {
    expect(decodeGlobalInfoRobotPoses({
      data: JSON.stringify({
        timestamp: 12.5,
        robots: {
          robot_0: { position: { x: -1.25, y: 2.5, z: 0.1 } },
          robot_4: { position: { x: 3.75, y: -0.5, z: 0.1 } }
        },
        objects: {}
      })
    })).toEqual({
      "0": { x: -1.25, y: 2.5 },
      "4": { x: 3.75, y: -0.5 }
    });
  });

  it("decodes Gazebo model positions in the shared world frame and ignores non-robots", () => {
    expect(decodeGazeboModelPoses({
      name: ["ground_plane", "tb_0", "transport_box", "tb_3"],
      pose: [
        { position: { x: 0, y: 0 } },
        { position: { x: 3.7, y: 16.7 } },
        { position: { x: 5, y: 6 } },
        { position: { x: 4.3, y: 16.1 } }
      ]
    })).toEqual({
      "0": { x: 3.7, y: 16.7 },
      "3": { x: 4.3, y: 16.1 }
    });
  });
});

describe("RosConnectionManager", () => {
  it.each(["success", "error", "throw"] as const)(
    "cleans up the discovery timeout when getTopics finishes with %s",
    async (outcome) => {
      vi.useFakeTimers();
      const getTopics: NonNullable<RosLike["getTopics"]> = (callback, failedCallback) => {
        if (outcome === "throw") throw new Error("discovery failed");
        if (outcome === "error") failedCallback?.("discovery failed");
        else callback({ topics: ["/global_info"], types: ["std_msgs/msg/String"] });
      };
      const ros = Object.assign(new FakeRos(), { getTopics });
      const manager = new RosConnectionManager({ createRos: () => ros });
      try {
        const discovery = manager.topics("ros2");
        if (outcome === "success") {
          await expect(discovery).resolves.toEqual([
            { topic: "/global_info", messageType: "std_msgs/msg/String" }
          ]);
        } else {
          await expect(discovery).rejects.toThrow("discovery failed");
        }
        expect(vi.getTimerCount()).toBe(0);
      } finally {
        manager.close();
        vi.useRealTimers();
      }
    }
  );

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
  it("retries discovery after a lost rosapi response and resolves world poses", async () => {
    vi.useFakeTimers();
    const ros = new DiscoverableFakeRos({
      topics: ["/global_info"],
      types: ["std_msgs/msg/String"]
    });
    const getTopics = vi.spyOn(ros, "getTopics").mockImplementationOnce(() => undefined);
    const manager = new RosConnectionManager({ createRos: () => ros });
    const runtime = new RosDataRuntime(manager, {
      createRegistry: (_connections, contracts) => new FakeRegistry(contracts)
    });
    try {
      runtime.configure("run-test", [{
        graph: "ros2",
        topic_pattern: "/global_info",
        message_types: ["std_msgs/msg/String"],
        capability: "global_info",
        required: false,
        availability: "UNKNOWN"
      }]);
      ros.emit("connection");
      await vi.advanceTimersByTimeAsync(9999);
      expect(getTopics).toHaveBeenCalledTimes(1);
      expect(runtime.getSnapshot()[0]?.topic).toBeUndefined();

      await vi.advanceTimersByTimeAsync(2001);

      expect(getTopics).toHaveBeenCalledTimes(2);
      expect(runtime.getSnapshot()[0]).toMatchObject({
        topic: "/global_info",
        message_type: "std_msgs/msg/String",
        availability: "WAITING"
      });
    } finally {
      runtime.configure(null, []);
      manager.close();
      vi.useRealTimers();
    }
  });

  it("attaches a subscription requested before rosapi discovery completes", async () => {
    const ros = new DiscoverableFakeRos({
      topics: ["/percent_completed"],
      types: ["std_msgs/Float32"]
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
    const progressSource = {
      graph: "ros1" as const,
      topic_pattern: "/percent_completed",
      topic: "/percent_completed",
      message_types: ["std_msgs/Float32"],
      message_type: "std_msgs/Float32",
      capability: "exploration_progress",
      required: false,
      availability: "WAITING" as const
    };

    const release = runtime.subscribe(progressSource, () => undefined);
    runtime.configure("run-test", [progressSource]);
    ros.emit("connection");
    await vi.waitFor(() => expect(registries).toHaveLength(1));

    expect(registries[0].activeSubscriptions).toBe(1);
    release();
    expect(registries[0].activeSubscriptions).toBe(0);
  });

  it("rediscovers topics that appear after the ROS graph first connects", async () => {
    vi.useFakeTimers();
    const ros = new DiscoverableFakeRos({ topics: [], types: [] });
    const manager = new RosConnectionManager({ createRos: () => ros });
    const runtime = new RosDataRuntime(manager);
    runtime.configure("run-test", [laserProfileSource()]);
    ros.emit("connection");
    await vi.waitFor(() => expect(runtime.getSnapshot()[0]?.availability).toBe("WAITING"));

    ros.setTopics({
      topics: ["/robot_0/scan"],
      types: ["sensor_msgs/msg/LaserScan"]
    });
    await vi.advanceTimersByTimeAsync(2000);

    expect(runtime.getSnapshot()[0]).toMatchObject({
      topic: "/robot_0/scan",
      availability: "WAITING"
    });
    runtime.configure(null, []);
    expect(vi.getTimerCount()).toBe(0);
    vi.useRealTimers();
  });

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
    expect(vi.getTimerCount()).toBe(2);

    release();

    expect(vi.getTimerCount()).toBe(1);
    runtime.configure(null, []);
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
      "http://platform.test:18090/video/ros1/stream?topic=/robot_0/camera&type=mjpeg&quality=70",
      "",
      "REMOVED",
      "http://platform.test:18090/video/ros1/stream?topic=/robot_1/camera&type=mjpeg&quality=70"
    ]);
  });

  it("uses a distinct URL only when a caller requests an empty-stream retry", () => {
    const assignments: string[] = [];
    const target = {
      set src(value: string) {
        assignments.push(value);
      },
      removeAttribute(name: string) {
        if (name === "src") assignments.push("REMOVED");
      }
    };
    const video = new VideoClient("http://platform.test:18090");
    const source = { graph: "ros1" as const, topic: "/task_allocation_image" };

    video.select(target, source);
    video.select(target, source, 1);

    expect(assignments).toEqual([
      "http://platform.test:18090/video/ros1/stream?topic=/task_allocation_image&type=mjpeg&quality=70",
      "",
      "REMOVED",
      "http://platform.test:18090/video/ros1/stream?topic=/task_allocation_image&type=mjpeg&quality=70&retry=1"
    ]);
  });

  it("uses cache-keyed snapshots without changing the camera stream transport", () => {
    const assignments: string[] = [];
    const target = {
      set src(value: string) {
        assignments.push(value);
      },
      removeAttribute() {}
    };
    const video = new VideoClient("http://platform.test:18090");
    const source = { graph: "ros1" as const, topic: "/task_allocation_image" };

    video.snapshot(target, source);
    video.snapshot(target, source, 1);

    expect(assignments).toEqual([
      "http://platform.test:18090/video/ros1/snapshot?topic=/task_allocation_image&quality=70",
      "http://platform.test:18090/video/ros1/snapshot?topic=/task_allocation_image&quality=70&retry=1"
    ]);
  });
});

describe("profile-approved message decoders", () => {
  it("decodes task assignments, frontiers and active search-agent status", () => {
    expect(decodeTaskAllocation({
      id: ["agent_0", "agent_4"],
      startx: [1, -2],
      starty: [3, 4],
      goalx: [5, 6],
      goaly: [7, 8],
      goal_type: [1, 2],
      goal_id: [11, 12]
    })).toEqual([
      { robotId: "agent_0", start: { x: 1, y: 3 }, goal: { x: 5, y: 7 }, goalType: 1, goalId: 11 },
      { robotId: "agent_4", start: { x: -2, y: 4 }, goal: { x: 6, y: 8 }, goalType: 2, goalId: 12 }
    ]);
    expect(decodeTaskAllocation({ id: ["agent_0"], startx: [] })).toBeNull();
    expect(decodeFrontiers({
      points: [{ x: 1, y: 2, z: 0 }, { x: 3, y: 4, z: 0 }],
      infoGain: [0.2, 0.8]
    })).toEqual({ count: 2, maximumInformationGain: 0.8 });
    expect(decodeAgentStatus({
      robot_id: ["agent_0", "agent_1", "agent_2"],
      ip_adress: ["10.0.0.1", "10.0.0.2", "10.0.0.3"],
      battery_lvl: [98, 80, 70],
      feedback_freq: [10, 10, 0],
      status: ["ROBOT_STATUS_ACTIVE", "ROBOT_STATUS_INACTIVE", "ROBOT_STATUS_COMM_FAIL"]
    })).toEqual({ total: 3, active: 2 });
  });

  it("keeps only discovered emergency objects with finite map positions", () => {
    expect(decodeEmergencyObjects({
      objects: [
        emergencyObject(1, 7, true, 1.25, -2.5, "agent_3"),
        emergencyObject(2, 8, true, 4, 5, "agent_1"),
        emergencyObject(3, 9, true, 6, 7, "agent_0", { x: 8, y: 9 }),
        emergencyObject(1, 10, false, 20, 30, "")
      ]
    })).toEqual([
      { kind: "victim", id: 7, detectorRobot: "agent_3", position: { x: 1.25, y: -2.5 }, goal: null },
      { kind: "fire", id: 8, detectorRobot: "agent_1", position: { x: 4, y: 5 }, goal: null },
      { kind: "box", id: 9, detectorRobot: "agent_0", position: { x: 6, y: 7 }, goal: { x: 8, y: 9 } }
    ]);
    expect(decodeEmergencyObjects({ objects: [{ object_type: 1, discovered: true }] })).toEqual([]);
    expect(decodeEmergencyObjects({ objects: "invalid" })).toBeNull();
  });

  it("decodes the response task queue, active task and continuous group status", () => {
    const activeTask = {
      task_id: 1,
      task_type: 3,
      status: 4,
      object_id: 7,
      detector_robot: "agent_1",
      scene_name: "tepper",
      staging_pose: { pose: { position: { x: 10.5, y: 14.1 } } },
      goal_pose: { pose: { position: { x: 10.5, y: 16.4 } } },
      has_goal_pose: true,
      hold_seconds: 20,
      behavior_type: "TASK_TRANSPORT",
      assigned_robots: ["tb_0", "tb_1", "tb_2", "tb_3"]
    };
    expect(decodeEmergencyTask(activeTask)).toEqual({
      taskId: 1,
      kind: "box",
      status: "acting",
      objectId: 7,
      detectorRobot: "agent_1",
      staging: { x: 10.5, y: 14.1 },
      goal: { x: 10.5, y: 16.4 },
      behaviorType: "TASK_TRANSPORT",
      assignedRobots: ["tb_0", "tb_1", "tb_2", "tb_3"]
    });
    expect(decodeEmergencyTasks({ tasks: [activeTask, { ...activeTask, task_id: 2, status: 1 }] }))
      .toHaveLength(2);
    expect(decodeEmergencyTask({ ...activeTask, task_id: "1" })).toBeNull();

    expect(decodeResponseStatus({
      state: 2,
      active_task_id: 1,
      active_task_type: 3,
      behavior_type: "TASK_TRANSPORT",
      detail: "TRACKING",
      formation_center: { pose: { position: { x: 9.68, y: 13.93 } } },
      target_pose: { pose: { position: { x: 10.53, y: 14.07 } } },
      hold_remaining: 0.8,
      assigned_robots: ["tb_0", "tb_1", "tb_2", "tb_3"],
      planning_state: "TRACKING",
      longitudinal_scale: 0.7433,
      lateral_scale: 0.7433,
      rms_slot_error: 0.5852,
      max_slot_error: 0.874,
      minimum_clearance: 0.3646,
      motion_enabled: true
    })).toEqual({
      state: "navigating",
      activeTaskId: 1,
      activeTaskKind: "box",
      behaviorType: "TASK_TRANSPORT",
      detail: "TRACKING",
      formationCenter: { x: 9.68, y: 13.93 },
      target: { x: 10.53, y: 14.07 },
      holdRemaining: 0.8,
      assignedRobots: ["tb_0", "tb_1", "tb_2", "tb_3"],
      planningState: "TRACKING",
      longitudinalScale: 0.7433,
      lateralScale: 0.7433,
      rmsSlotError: 0.5852,
      maxSlotError: 0.874,
      minimumClearance: 0.3646,
      motionEnabled: true
    });
    expect(decodeString({ data: "NAV_FORMATION" })).toBe("NAV_FORMATION");
    expect(decodeBoolean({ data: false })).toBe(false);
  });

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

  it("decodes Burger IMU acceleration and angular velocity", () => {
    const snapshot = decodeImu({
      linear_acceleration: { x: 0.3, y: 0.4, z: 9.8 },
      angular_velocity: { x: 0, y: 0, z: -0.25 }
    });
    expect(snapshot?.acceleration).toBeCloseTo(9.81275, 5);
    expect(snapshot?.angularSpeed).toBe(0.25);
    expect(decodeImu({ linear_acceleration: { x: "invalid" } })).toBeNull();
  });

  it("decodes discussed-dispersion opinion and distance messages", () => {
    expect(decodeOpinion({ id: 2, opinion: 3 })).toEqual({ robotId: 2, opinion: 3 });
    expect(decodeOpinion({ id: "2", opinion: 3 })).toBeNull();
    expect(decodeScalar({ data: 1.4 })).toBe(1.4);
    expect(decodeScalar({ data: Number.NaN })).toBeNull();
  });

  it("decodes one e-puck infrared Range observation", () => {
    expect(decodeRange({ range: 0.07, min_range: 0, max_range: 0.08 })).toEqual({
      meters: 0.07,
      minMeters: 0,
      maxMeters: 0.08
    });
    expect(decodeRange({ range: Number.POSITIVE_INFINITY })).toBeNull();
  });
});

function emergencyObject(
  objectType: number,
  objectId: number,
  discovered: boolean,
  x: number,
  y: number,
  detectorRobot: string,
  goal?: { x: number; y: number }
) {
  return {
    object_type: objectType,
    object_id: objectId,
    discovered,
    detector_robot: detectorRobot,
    precise_pose: { pose: { position: { x, y } } },
    goal_pose: { pose: { position: goal ?? { x: 0, y: 0 } } },
    has_goal_pose: Boolean(goal)
  };
}

describe("cross-simulator robot sources", () => {
  const sources = [
    {
      graph: "ros1" as const,
      topic_pattern: "/robosar_agent_bringup_node/<agent>/camera/image_raw",
      topic: "/robosar_agent_bringup_node/agent_0/camera/image_raw",
      message_types: ["sensor_msgs/Image"],
      message_type: "sensor_msgs/Image",
      capability: "camera",
      required: false,
      availability: "ONLINE" as const
    },
    {
      graph: "ros2" as const,
      topic_pattern: "/<robot>/scan",
      topic: "/robot_0/scan",
      message_types: ["sensor_msgs/msg/LaserScan"],
      message_type: "sensor_msgs/msg/LaserScan",
      capability: "laser",
      required: false,
      availability: "ONLINE" as const
    },
    {
      graph: "ros1" as const,
      topic_pattern: "/tb_i/odom",
      topic: "/tb_1/odom",
      message_types: ["nav_msgs/Odometry"],
      message_type: "nav_msgs/Odometry",
      capability: "odometry",
      required: true,
      availability: "ONLINE" as const
    }
  ];

  it("groups ROS1 agent, ROS2 robot and response-controller names by numeric robot id", () => {
    expect(robotKeyForSource(sources[0])).toBe("0");
    expect(robotKeyForSource(sources[1])).toBe("0");
    expect(robotKeyForSource(sources[2])).toBe("1");
    expect(robotOptions(sources)).toEqual([
      { key: "0", label: "机器人 0" },
      { key: "1", label: "机器人 1" }
    ]);
  });

  it("keeps every robot declared by the Run even when ROS discovery is partial", () => {
    expect(robotOptions(sources.slice(0, 1), 5)).toEqual([
      { key: "0", label: "机器人 0" },
      { key: "1", label: "机器人 1" },
      { key: "2", label: "机器人 2" },
      { key: "3", label: "机器人 3" },
      { key: "4", label: "机器人 4" }
    ]);
  });

  it("describes the real e-puck and Burger sensor capabilities", () => {
    expect(robotSupportsCapability("e_puck", "camera")).toBe(true);
    expect(robotSupportsCapability("e_puck", "infrared")).toBe(true);
    expect(robotSupportsCapability("burger", "camera")).toBe(false);
    expect(robotSupportsCapability("burger", "laser")).toBe(true);
    expect(robotSupportsCapability("burger", "imu")).toBe(true);
  });

  it("keeps the profile order when selecting one robot capability", () => {
    expect(sourceForRobot(sources, "camera", "0")?.topic).toBe(
      "/robosar_agent_bringup_node/agent_0/camera/image_raw"
    );
    expect(sourceForRobot(sources, "laser", "0")?.topic).toBe("/robot_0/scan");
    expect(sourceForRobot(sources, "camera", "1")).toBeUndefined();
  });

  it("prefers an observed Webots source over a silent ROS1 bridge duplicate", () => {
    const duplicateOdometry = [
      {
        ...sources[0],
        graph: "ros1" as const,
        topic_pattern: "/robosar_agent_bringup_node/<agent>/odom_data_euler",
        topic: "/robosar_agent_bringup_node/agent_0/odom_data_euler",
        message_types: ["nav_msgs/Odometry"],
        message_type: "nav_msgs/Odometry",
        capability: "odometry",
        availability: "WAITING" as const
      },
      {
        ...sources[0],
        graph: "ros2" as const,
        topic_pattern: "/<robot>/odom",
        topic: "/robot_0/odom",
        message_types: ["nav_msgs/msg/Odometry"],
        message_type: "nav_msgs/msg/Odometry",
        capability: "odometry",
        availability: "ONLINE" as const
      }
    ];

    expect(sourceForRobot(duplicateOdometry, "odometry", "0")?.topic).toBe("/robot_0/odom");
  });

  it("splits exploration and response robots when rescue sources share numeric ids", () => {
    const rescueSources = [
      {
        ...sources[0],
        topic: "/robosar_agent_bringup_node/agent_0/camera/image_raw",
        capability: "camera"
      },
      {
        ...sources[0],
        topic: "/robosar_agent_bringup_node/agent_0/feedback/scan",
        capability: "laser"
      },
      {
        ...sources[0],
        topic_pattern: "/tb_i/camera/color/image_raw",
        topic: "/tb_0/camera/color/image_raw",
        capability: "camera"
      },
      {
        ...sources[0],
        topic_pattern: "/tb_i/scan",
        topic: "/tb_0/scan",
        capability: "laser"
      },
      {
        ...sources[2],
        topic: "/tb_1/odom",
        capability: "odometry"
      }
    ];

    expect(robotOptions(rescueSources, 2)).toEqual([
      { key: "agent:0", label: "探索机器人 0" },
      { key: "agent:1", label: "探索机器人 1" },
      { key: "tb:0", label: "响应机器人 0" },
      { key: "tb:1", label: "响应机器人 1" }
    ]);
    expect(sourceForRobot(rescueSources, "camera", "agent:0")?.topic).toBe(
      "/robosar_agent_bringup_node/agent_0/camera/image_raw"
    );
    expect(sourceForRobot(rescueSources, "camera", "tb:0")?.topic).toBe("/tb_0/camera/color/image_raw");
    expect(sourceForRobot(rescueSources, "laser", "agent:0")?.topic).toBe(
      "/robosar_agent_bringup_node/agent_0/feedback/scan"
    );
    expect(sourceForRobot(rescueSources, "laser", "tb:0")?.topic).toBe("/tb_0/scan");
  });

  it("splits rescue robot groups from unresolved profile patterns", () => {
    const rescueSources = [
      {
        ...sources[0],
        topic_pattern: "/robosar_agent_bringup_node/<agent>/camera/image_raw",
        topic: undefined,
        capability: "camera"
      },
      {
        ...sources[0],
        topic_pattern: "/tb_i/camera/color/image_raw",
        topic: undefined,
        capability: "camera"
      },
      {
        ...sources[2],
        topic_pattern: "/tb_i/odom",
        topic: "/tb_0/odom",
        capability: "odometry"
      }
    ];

    expect(robotOptions(rescueSources, 2)).toEqual([
      { key: "agent:0", label: "探索机器人 0" },
      { key: "agent:1", label: "探索机器人 1" },
      { key: "tb:0", label: "响应机器人 0" }
    ]);
  });
});

describe("multi-robot trajectory plot", () => {
  it("keeps a bounded ordered trail for each robot", () => {
    const initial = {
      "0": [{ x: 0, y: 0 }],
      "1": [{ x: -1, y: 2 }]
    };

    const updated = appendTrajectoryPoint(initial, "0", { x: 1, y: 0.5 }, 2);
    const bounded = appendTrajectoryPoint(updated, "0", { x: 2, y: 1 }, 2);

    expect(bounded).toEqual({
      "0": [{ x: 1, y: 0.5 }, { x: 2, y: 1 }],
      "1": [{ x: -1, y: 2 }]
    });
    expect(initial["0"]).toEqual([{ x: 0, y: 0 }]);
  });

  it("uses one equal-scale XY viewport for all robot trails", () => {
    const bounds = trajectoryBounds({
      "0": [{ x: -2, y: -1 }, { x: 2, y: 1 }],
      "1": [{ x: 0, y: 3 }]
    });

    expect(bounds).toEqual({ minX: -2.4, maxX: 2.4, minY: -1.4, maxY: 3.4 });
    expect(projectTrajectoryPoint({ x: 0, y: 1 }, bounds, 480, 480, 24)).toEqual({
      x: 240,
      y: 240
    });
  });
});

describe("Run-scoped ROS subscriptions", () => {
  it("changes identity when a new Run reuses the same ROS topic", () => {
    const source = sourcesForIdentity()[0];
    expect(runScopedSourceKey("run-first", source)).not.toBe(
      runScopedSourceKey("run-second", source)
    );
  });
});

function sourcesForIdentity() {
  return [{
    graph: "ros2" as const,
    topic_pattern: "/<robot>/odom",
    topic: "/robot_0/odom",
    message_types: ["nav_msgs/msg/Odometry"],
    message_type: "nav_msgs/msg/Odometry",
    capability: "odometry",
    required: true,
    availability: "WAITING" as const
  }];
}

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
