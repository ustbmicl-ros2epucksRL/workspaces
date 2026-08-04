import { expect, test, type Page, type WebSocketRoute } from "@playwright/test";
import { installMockApi } from "./mockApi";

async function installMockRosbridge(
  page: Page,
  sources: Array<{ topic: string; message_type: string }>
) {
  await page.addInitScript((contracts) => {
    type RosbridgeWindow = Window & {
      __masarRosSubscriptions: Set<string>;
      __masarRosPublish: (topic: string, message: unknown) => void;
      __masarRosSetContracts: (next: typeof contracts) => void;
    };
    const bridgeWindow = window as RosbridgeWindow;
    const sockets = new Set<MockWebSocket>();
    let currentContracts = [...contracts];
    bridgeWindow.__masarRosSubscriptions = new Set<string>();
    bridgeWindow.__masarRosSetContracts = (next) => {
      currentContracts = [...next];
    };

    class MockWebSocket {
      static readonly CONNECTING = 0;
      static readonly OPEN = 1;
      static readonly CLOSING = 2;
      static readonly CLOSED = 3;
      readonly url: string;
      readyState = MockWebSocket.CONNECTING;
      binaryType = "blob";
      private openHandler: ((event: Event) => void) | null = null;
      onclose: ((event: CloseEvent) => void) | null = null;
      onerror: ((event: Event) => void) | null = null;
      onmessage: ((event: MessageEvent) => void) | null = null;

      constructor(url: string | URL) {
        this.url = String(url);
        sockets.add(this);
      }

      get onopen() {
        return this.openHandler;
      }

      set onopen(handler: ((event: Event) => void) | null) {
        this.openHandler = handler;
        if (!handler || this.readyState !== MockWebSocket.CONNECTING) return;
        window.setTimeout(() => {
          if (this.readyState !== MockWebSocket.CONNECTING) return;
          this.readyState = MockWebSocket.OPEN;
          this.openHandler?.(new Event("open"));
        }, 0);
      }

      send(data: string) {
        const payload = JSON.parse(data) as { op?: string; topic?: string; service?: string; id?: string };
        if (payload.op === "subscribe" && payload.topic) {
          bridgeWindow.__masarRosSubscriptions.add(payload.topic);
        }
        if (payload.op === "call_service" && payload.service === "rosapi/topics" && payload.id) {
          window.setTimeout(() => this.receive({
            op: "service_response",
            id: payload.id,
            service: payload.service,
            result: true,
            values: {
              topics: currentContracts.map((source) => source.topic),
              types: currentContracts.map((source) => source.message_type)
            }
          }), 0);
        }
      }

      close() {
        this.readyState = MockWebSocket.CLOSED;
        sockets.delete(this);
        this.onclose?.(new CloseEvent("close"));
      }

      receive(payload: unknown) {
        this.onmessage?.(new MessageEvent("message", { data: JSON.stringify(payload) }));
      }
    }

    bridgeWindow.__masarRosPublish = (topic, message) => {
      for (const socket of sockets) socket.receive({ op: "publish", topic, msg: message });
    };
    Object.defineProperty(window, "WebSocket", { configurable: true, value: MockWebSocket });
  }, sources);
}

async function publishRos(page: Page, topic: string, message: unknown) {
  await page.evaluate(({ topic: publishedTopic, message: publishedMessage }) => {
    const bridgeWindow = window as Window & {
      __masarRosPublish: (topic: string, message: unknown) => void;
    };
    bridgeWindow.__masarRosPublish(publishedTopic, publishedMessage);
  }, { topic, message });
}

function explorationSource(topic: string, messageType: string, capability: string) {
  return {
    graph: "ros1",
    topic_pattern: topic,
    topic,
    message_types: [messageType],
    message_type: messageType,
    capability,
    required: false,
    availability: "WAITING"
  };
}

function emergencyObjectMessage(
  objectType: number,
  objectId: number,
  x: number,
  y: number,
  detectorRobot: string,
  goal?: { x: number; y: number }
) {
  return {
    object_type: objectType,
    object_id: objectId,
    discovered: true,
    detector_robot: detectorRobot,
    precise_pose: { pose: { position: { x, y } } },
    goal_pose: { pose: { position: goal ?? { x: 0, y: 0 } } },
    has_goal_pose: Boolean(goal)
  };
}

function odometryMessage(x: number, y: number) {
  return {
    pose: {
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 }
      }
    },
    twist: { twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } } }
  };
}

test("panel resize, maximize, restore and popout reuse one run", async ({ page, context }) => {
  await installMockApi(page, { activeRun: true });
  let createRequests = 0;
  page.on("request", (request) => {
    if (request.method() === "POST" && request.url().endsWith("/api/v1/runs")) createRequests += 1;
  });
  await page.goto("/?workspace=exploration");
  await expect(page.getByTestId("run-id")).toHaveText("run-layout-test");

  const panel = page.getByTestId("panel-simulation");
  await panel.getByRole("button", { name: "调整尺寸" }).click();
  await expect(panel).toHaveAttribute("data-size", "wide");
  await panel.getByRole("button", { name: "最大化" }).click();
  await expect(panel).toHaveAttribute("data-maximized", "true");
  await panel.getByRole("button", { name: "恢复" }).click();
  await expect(panel).toHaveAttribute("data-maximized", "false");

  const popupPromise = context.waitForEvent("page");
  await panel.getByRole("button", { name: "弹出" }).click();
  const popup = await popupPromise;
  await popup.waitForLoadState("domcontentloaded");
  expect(new URL(popup.url()).searchParams.get("run_id")).toBe("run-layout-test");
  await expect(popup.getByTestId("workbench")).toBeVisible();
  expect(createRequests).toBe(0);
  await popup.close();
});

test("missing and unhealthy data is explicit and never fabricated", async ({ page }) => {
  await installMockApi(page, { activeRun: true });
  await page.goto("/?workspace=exploration");

  await expect(page.getByTestId("sensor-camera-state")).toHaveText("未发现相机话题");
  const health = page.getByTestId("panel-health");
  await expect(health.getByTestId("topic-state-ONLINE")).toBeVisible();
  await expect(health.getByTestId("topic-state-STALE")).toBeVisible();
  await expect(health.getByTestId("topic-state-TYPE_MISMATCH")).toBeVisible();
  await expect(health.getByTestId("topic-state-UNAVAILABLE")).toBeVisible();
  await expect(health.getByTestId("topic-state-WAITING")).toBeVisible();
  await expect(health.getByTestId("topic-state-DISCONNECTED").first()).toBeVisible();
  await expect(page.getByTestId("panel-sensors")).not.toContainText("0.00");
});

test("idle group behavior does not present planned robots as observed runtime robots", async ({ page }) => {
  await installMockApi(page, { activeRun: false });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByTestId("trajectory-robot-row")).toHaveCount(0);
  await expect(page.getByTestId("health-robot-row")).toHaveCount(0);
  await expect(page.getByLabel("选择观测机器人").locator("option")).toHaveText(["未发现机器人"]);
  await expect(page.getByTestId("sensor-camera-state")).toHaveText("等待运行");
});

test("completed group behavior removes live robot selectors and health rows", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runState: "COMPLETED",
    runParameters: { robot: "burger", number_robots: 5 },
    dataSources: Array.from({ length: 5 }, (_, index) => ({
      graph: "ros2",
      topic_pattern: "/<robot>/odom",
      topic: `/robot_${index}/odom`,
      message_types: ["nav_msgs/msg/Odometry"],
      message_type: "nav_msgs/msg/Odometry",
      capability: "odometry",
      required: true,
      availability: "ONLINE"
    }))
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByTestId("trajectory-robot-row")).toHaveCount(0);
  await expect(page.getByTestId("health-robot-row")).toHaveCount(0);
  await expect(page.getByLabel("选择观测机器人").locator("option")).toHaveText(["未发现机器人"]);
});

test("Webots group trajectory uses shared Supervisor world poses instead of local odometry", async ({ page }) => {
  const dataSources = [
    {
      graph: "ros2",
      topic_pattern: "/global_info",
      topic: "/global_info",
      message_types: ["std_msgs/msg/String"],
      message_type: "std_msgs/msg/String",
      capability: "global_info",
      required: false,
      availability: "ONLINE"
    },
    ...[0, 1].map((index) => ({
      graph: "ros2",
      topic_pattern: "/<robot>/odom",
      topic: `/robot_${index}/odom`,
      message_types: ["nav_msgs/msg/Odometry"],
      message_type: "nav_msgs/msg/Odometry",
      capability: "odometry",
      required: true,
      availability: "ONLINE"
    }))
  ];
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runParameters: { robot: "burger", number_robots: 2 },
    dataSources
  });
  await installMockRosbridge(page, dataSources);
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect.poll(() => page.evaluate(() => (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/global_info"))).toBe(true);
  await publishRos(page, "/robot_0/odom", odometryMessage(99, 99));
  await publishRos(page, "/global_info", { data: JSON.stringify({ robots: {
    robot_0: { position: { x: -1.5, y: 2.25, z: 0.08 } },
    robot_1: { position: { x: 3.5, y: -4.25, z: 0.08 } }
  }, objects: {} }) });
  await publishRos(page, "/global_info", { data: JSON.stringify({ robots: {
    robot_0: { position: { x: -1.25, y: 2.5, z: 0.08 } },
    robot_1: { position: { x: 3.25, y: -4, z: 0.08 } }
  }, objects: {} }) });

  const rows = page.getByTestId("trajectory-robot-row");
  await expect(rows.nth(0)).toContainText("-1.25, 2.50");
  await expect(rows.nth(1)).toContainText("3.25, -4.00");
  await expect(page.getByTestId("trajectory-path-0")).toBeVisible();
  await expect(page.getByTestId("panel-trajectory")).not.toContainText("99.00");
});

test("Gazebo group trajectory uses model states in the shared world frame", async ({ page }) => {
  const dataSources = [
    {
      graph: "ros1",
      topic_pattern: "/gazebo/model_states",
      topic: "/gazebo/model_states",
      message_types: ["gazebo_msgs/ModelStates"],
      message_type: "gazebo_msgs/ModelStates",
      capability: "gazebo_model_states",
      required: false,
      availability: "ONLINE"
    },
    ...[0, 1].map((index) => ({
      graph: "ros1",
      topic_pattern: "/tb_i/odom",
      topic: `/tb_${index}/odom`,
      message_types: ["nav_msgs/Odometry"],
      message_type: "nav_msgs/Odometry",
      capability: "odometry",
      required: true,
      availability: "ONLINE"
    }))
  ];
  await installMockApi(page, {
    activeRun: true,
    runScenario: "control.gazebo.response-deform",
    dataSources
  });
  await installMockRosbridge(page, dataSources);
  await page.goto("/?workspace=group_behavior&scenario=control.gazebo.response-deform");

  await expect.poll(() => page.evaluate(() => (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/gazebo/model_states"))).toBe(true);
  await publishRos(page, "/tb_0/odom", odometryMessage(88, 88));
  await publishRos(page, "/gazebo/model_states", {
    name: ["ground_plane", "tb_0", "tb_1", "transport_box"],
    pose: [
      { position: { x: 0, y: 0 } },
      { position: { x: 3.7, y: 16.7 } },
      { position: { x: 4.3, y: 16.1 } },
      { position: { x: 5, y: 10 } }
    ]
  });
  await publishRos(page, "/gazebo/model_states", {
    name: ["ground_plane", "tb_0", "tb_1", "transport_box"],
    pose: [
      { position: { x: 0, y: 0 } },
      { position: { x: 3.8, y: 16.6 } },
      { position: { x: 4.4, y: 16 } },
      { position: { x: 5, y: 10 } }
    ]
  });

  const rows = page.getByTestId("trajectory-robot-row");
  await expect(rows.nth(0)).toContainText("3.80, 16.60");
  await expect(rows.nth(1)).toContainText("4.40, 16.00");
  await expect(page.getByTestId("trajectory-path-0")).toBeVisible();
  await expect(page.getByTestId("panel-trajectory")).not.toContainText("88.00");
});

test("exploration uses the Qt allocation feed and live mission data instead of a task-map placeholder", async ({ page }) => {
  const dataSources = [
    explorationSource("/task_allocation_image", "sensor_msgs/Image", "task_allocation_image"),
    explorationSource("/task_allocation", "robosar_messages/task_allocation", "task_allocation"),
    explorationSource("/percent_completed", "std_msgs/Float32", "exploration_progress"),
    explorationSource("/frontier_filter/filtered_frontiers", "robosar_messages/PointArray", "frontiers"),
    explorationSource("/robosar_agent_bringup_node/all_agent_status", "robosar_messages/agents_status", "agent_status"),
    explorationSource("/robosar_sim/emergency_objects", "robosar_messages/EmergencyObjectArray", "emergency_objects"),
    ...[0, 4].flatMap((index) => [
      explorationSource(
        `/robosar_agent_bringup_node/agent_${index}/camera/image_raw`,
        "sensor_msgs/Image",
        "camera"
      ),
      explorationSource(
        `/robosar_agent_bringup_node/agent_${index}/feedback/scan`,
        "sensor_msgs/LaserScan",
        "laser"
      ),
      explorationSource(
        `/robosar_agent_bringup_node/agent_${index}/odom_data_euler`,
        "nav_msgs/Odometry",
        "odometry"
      )
    ])
  ];
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources
  });
  await installMockRosbridge(page, dataSources);

  await page.goto("/?workspace=exploration&scenario=exploration.gazebo.tepper");

  const allocation = page.getByTestId("panel-map");
  await expect(allocation.getByRole("heading", { name: "任务分配与建图" })).toBeVisible();
  await expect(allocation.getByTestId("task-allocation-image")).toHaveAttribute(
    "src",
    /\/video\/ros1\/snapshot\?topic=\/task_allocation_image&quality=70/
  );
  await expect(page.getByText("任务地图", { exact: true })).toHaveCount(0);

  const selector = page.getByLabel("选择观测机器人");
  await expect(selector.locator("option")).toHaveCount(5);
  await selector.selectOption("4");
  await expect(page.getByTestId("sensor-source-topic")).toHaveText(
    "/robosar_agent_bringup_node/agent_4/camera/image_raw"
  );

  await expect.poll(() => page.evaluate(() => (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/percent_completed"))).toBe(true);
  await publishRos(page, "/percent_completed", { data: 0.375 });
  await publishRos(page, "/task_allocation", {
      id: ["agent_0", "agent_4"], startx: [0, 1], starty: [0, 1],
      goalx: [4, 8], goaly: [5, 9], goal_type: [1, 2], goal_id: [10, 11]
  });
  await publishRos(page, "/frontier_filter/filtered_frontiers", {
    points: [{ x: 1, y: 2, z: 0 }, { x: 3, y: 4, z: 0 }], infoGain: [0.4, 0.8]
  });
  await publishRos(page, "/robosar_agent_bringup_node/all_agent_status", {
      robot_id: ["agent_0", "agent_1", "agent_2", "agent_3", "agent_4"],
      ip_adress: ["", "", "", "", ""], battery_lvl: [100, 100, 100, 100, 100],
      feedback_freq: [10, 10, 10, 10, 10],
      status: Array(5).fill("ROBOT_STATUS_ACTIVE")
  });
  await publishRos(page, "/robosar_sim/emergency_objects", { objects: [
      emergencyObjectMessage(1, 2, 1.2, 3.4, "agent_0"),
      emergencyObjectMessage(2, 4, 5.6, 7.8, "agent_4"),
      emergencyObjectMessage(3, 6, 9, 10, "agent_2", { x: 12, y: 13 })
  ] });

  const mission = page.getByTestId("panel-mission");
  await expect(allocation.getByTestId("map-coverage")).toHaveText("37.5%");
  await expect(allocation.getByTestId("map-assignment-count")).toHaveText("2");
  await expect(allocation.getByText("本轮分配", { exact: true })).toBeVisible();
  await expect(mission.getByTestId("active-agent-count")).toHaveText("5/5");
  await expect(mission.getByTestId("exploration-progress")).toHaveCount(0);
  await expect(mission.getByTestId("assignment-count")).toHaveCount(0);
  await expect(mission.getByTestId("completed-task-count")).toHaveCount(0);
  await expect(mission.getByTestId("emergency-count-victim")).toHaveText("1");
  await expect(mission.getByTestId("emergency-count-fire")).toHaveText("1");
  await expect(mission.getByTestId("emergency-count-box")).toHaveText("1");
  await expect(mission).toContainText("(5.60, 7.80)");
  await expect(allocation.getByTestId("frontier-count")).toHaveText("2");
});

test("mission object feed can scroll every rendered object inside the panel", async ({ page }) => {
  const objectsTopic = "/robosar_sim/emergency_objects";
  const dataSources = [
    explorationSource(objectsTopic, "robosar_messages/EmergencyObjectArray", "emergency_objects")
  ];
  await installMockApi(page, {
    activeRun: true,
    runScenario: "rescue.gazebo.tepper-response",
    dataSources
  });
  await installMockRosbridge(page, dataSources);
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto("/?workspace=rescue&scenario=rescue.gazebo.tepper-response");

  await expect.poll(() => page.evaluate((topic) => (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has(topic), objectsTopic)).toBe(true);
  await publishRos(page, objectsTopic, {
    objects: Array.from({ length: 9 }, (_, index) => emergencyObjectMessage(
      (index % 3) + 1,
      index + 1,
      index + 0.25,
      index + 0.75,
      `agent_${index % 5}`
    ))
  });

  const mission = page.getByTestId("panel-mission");
  const objectList = mission.locator(".emergency-feed ol");
  await expect(objectList.locator("li")).toHaveCount(6);
  await objectList.evaluate((element) => element.scrollTo({ top: element.scrollHeight }));

  const panelBox = await mission.boundingBox();
  const lastObjectBox = await objectList.locator("li").last().boundingBox();
  expect(panelBox).not.toBeNull();
  expect(lastObjectBox).not.toBeNull();
  expect(lastObjectBox!.y + lastObjectBox!.height).toBeLessThanOrEqual(
    panelBox!.y + panelBox!.height - 1
  );
});

test("exploration derives active robots but waits for real allocation and emergency snapshots", async ({ page }) => {
  const dataSources = [
    explorationSource("/task_allocation", "robosar_messages/task_allocation", "task_allocation"),
    explorationSource("/tasks_completed", "std_msgs/Int32", "tasks_completed"),
    explorationSource(
      "/robosar_sim/emergency_objects",
      "robosar_messages/EmergencyObjectArray",
      "emergency_objects"
    ),
    explorationSource(
      "/robosar_agent_bringup_node/all_agent_status",
      "robosar_messages/agents_status",
      "agent_status"
    ),
    ...Array.from({ length: 5 }, (_, index) => explorationSource(
      `/robosar_agent_bringup_node/agent_${index}/odom_data_euler`,
      "nav_msgs/Odometry",
      "odometry"
    ))
  ];
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources
  });
  await installMockRosbridge(page, dataSources);
  await page.goto("/?workspace=exploration&scenario=exploration.gazebo.tepper");

  await expect.poll(() => page.evaluate(() => (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/robosar_agent_bringup_node/agent_4/odom_data_euler"))).toBe(true);
  for (let index = 0; index < 5; index += 1) {
    await publishRos(
      page,
      `/robosar_agent_bringup_node/agent_${index}/odom_data_euler`,
      odometryMessage(index, index)
    );
  }

  await expect(page.getByTestId("active-agent-count")).toHaveText("5/5");
  await expect(page.getByTestId("map-assignment-count")).toHaveText("—");
  await expect(page.getByTestId("completed-task-count")).toHaveCount(0);
  await expect(page.getByTestId("emergency-count-victim")).toHaveText("—");
  await expect(page.getByTestId("emergency-count-fire")).toHaveText("—");
  await expect(page.getByTestId("emergency-count-box")).toHaveText("—");
  await expect(page.getByTestId("panel-mission")).toContainText("等待搜救对象快照");

  await publishRos(page, "/robosar_sim/emergency_objects", { objects: [] });
  await expect(page.getByTestId("emergency-count-victim")).toHaveText("0");
  await expect(page.getByTestId("emergency-count-fire")).toHaveText("0");
  await expect(page.getByTestId("emergency-count-box")).toHaveText("0");
  await expect(page.getByTestId("panel-mission")).toContainText("尚未发现搜救对象");
});

test("Webots uses live ROS2 odometry when silent ROS1 duplicates and a failed status snapshot exist", async ({ page }) => {
  const dataSources = [
    explorationSource(
      "/robosar_agent_bringup_node/all_agent_status",
      "robosar_messages/agents_status",
      "agent_status"
    ),
    ...Array.from({ length: 5 }, (_, index) => [
      explorationSource(
        `/robosar_agent_bringup_node/agent_${index}/odom_data_euler`,
        "nav_msgs/Odometry",
        "odometry"
      ),
      {
        graph: "ros2" as const,
        topic_pattern: `/<robot>/odom`,
        topic: `/robot_${index}/odom`,
        message_types: ["nav_msgs/msg/Odometry"],
        message_type: "nav_msgs/msg/Odometry",
        capability: "odometry",
        required: false,
        availability: "WAITING" as const
      }
    ]).flat()
  ];
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.webots.robosar",
    dataSources
  });
  await installMockRosbridge(page, dataSources);
  await page.goto("/?workspace=exploration&scenario=exploration.webots.robosar");

  await expect.poll(() => page.evaluate(() => (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/robot_4/odom") && (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/robosar_agent_bringup_node/all_agent_status"))).toBe(true);
  await publishRos(page, "/robosar_agent_bringup_node/all_agent_status", {
    robot_id: ["agent_0", "agent_1", "agent_2", "agent_3", "agent_4"],
    ip_adress: ["", "", "", "", ""],
    battery_lvl: [100, 100, 100, 100, 100],
    feedback_freq: [0, 0, 0, 0, 0],
    status: Array(5).fill("ROBOT_STATUS_COMM_FAIL")
  });
  await expect(page.getByTestId("active-agent-count")).toHaveText("0/5");
  for (let index = 0; index < 5; index += 1) {
    await publishRos(page, `/robot_${index}/odom`, odometryMessage(index + 0.5, index + 1.5));
  }

  await expect(page.getByTestId("active-agent-count")).toHaveText("5/5");
  await publishRos(page, "/robot_0/odom", odometryMessage(0.5, 1.5));
  await expect(page.getByTestId("panel-sensors")).toContainText("x 0.50 m · y 1.50 m");
  await expect(page.getByTestId("health-robot-row").first()).toContainText("位姿在线");
});

test("complete rescue renders real response state, task queue and formation quality", async ({ page }) => {
  const dataSources = [
    explorationSource("/response_group/status", "robosar_messages/ResponseGroupStatus", "response_status"),
    explorationSource("/emergency/tasks", "robosar_messages/EmergencyTaskArray", "response_tasks"),
    explorationSource("/response_group/active_task", "robosar_messages/EmergencyTask", "response_active_task"),
    explorationSource("/response_group/behavior_mode", "std_msgs/String", "behavior_mode"),
    explorationSource("/response_group/motion_enabled", "std_msgs/Bool", "response_motion_enabled"),
    explorationSource("/response_group/task_result", "robosar_messages/EmergencyTask", "task_result")
  ];
  await installMockApi(page, {
    activeRun: true,
    runScenario: "rescue.gazebo.tepper-response",
    dataSources
  });
  await installMockRosbridge(page, dataSources);
  await page.goto("/?workspace=rescue&scenario=rescue.gazebo.tepper-response");

  await expect.poll(() => page.evaluate(() => (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/response_group/status") && (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/emergency/tasks"))).toBe(true);

  const activeTask = {
    task_id: 1,
    task_type: 3,
    status: 4,
    object_id: 1,
    detector_robot: "agent_1",
    scene_name: "tepper",
    staging_pose: { pose: { position: { x: 10.53, y: 14.05 } } },
    goal_pose: { pose: { position: { x: 10.53, y: 16.37 } } },
    has_goal_pose: true,
    hold_seconds: 20,
    behavior_type: "TASK_TRANSPORT",
    assigned_robots: ["tb_0", "tb_1", "tb_2", "tb_3"]
  };
  await publishRos(page, "/response_group/status", {
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
  });
  await publishRos(page, "/emergency/tasks", {
    tasks: [activeTask, { ...activeTask, task_id: 2, status: 1, object_id: 0 }]
  });
  await publishRos(page, "/response_group/active_task", activeTask);
  await publishRos(page, "/response_group/behavior_mode", { data: "NAV_FORMATION" });
  await publishRos(page, "/response_group/motion_enabled", { data: true });
  await publishRos(page, "/response_group/task_result", { ...activeTask, status: 5 });

  const response = page.getByTestId("panel-response");
  await expect(response.getByTestId("response-state")).toHaveText("导航中 · 路径跟踪");
  await expect(response.getByTestId("response-motion")).toHaveText("已启用");
  await expect(response.getByTestId("response-active-task")).toHaveText("#1 · 待推送障碍 · 执行中");
  await expect(response.getByTestId("response-queue")).toHaveText("2 项 · 1 执行中 · 1 待分配");
  await expect(response.getByTestId("response-formation-error")).toHaveText("RMS 0.585 m · 最大 0.874 m");
  await expect(response.getByTestId("response-clearance")).toHaveText("0.365 m");
  await expect(response.getByTestId("response-behavior")).toHaveCount(0);
  await expect(response.getByTestId("response-planning")).toHaveCount(0);
  await expect(response.getByTestId("response-scale")).toHaveCount(0);
  await expect(response.getByTestId("response-assigned")).toHaveCount(0);
  await expect(response.getByTestId("response-result")).toHaveCount(0);
  await expect(response).not.toContainText("未提供");
});

test("allocation image retries when a snapshot request produces no frame or DOM error", async ({ page }) => {
  const dataSources = [
    explorationSource("/task_allocation_image", "sensor_msgs/Image", "task_allocation_image")
  ];
  const streamRequests: string[] = [];
  await page.route("**/video/ros1/snapshot?**", async (route) => {
    streamRequests.push(route.request().url());
    await route.fulfill({
      status: 200,
      headers: { "content-type": "multipart/x-mixed-replace; boundary=frame" },
      body: ""
    });
  });
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources
  });
  await page.goto("/?workspace=exploration&scenario=exploration.gazebo.tepper");
  await expect(page.getByTestId("task-allocation-image")).toBeVisible();
  await expect.poll(() => streamRequests.length, { timeout: 5_000 }).toBeGreaterThan(1);
  expect(new Set(streamRequests).size).toBeGreaterThan(1);
});

test("allocation image periodically refreshes a successful snapshot", async ({ page }) => {
  const dataSources = [
    explorationSource("/task_allocation_image", "sensor_msgs/Image", "task_allocation_image")
  ];
  const snapshotRequests: string[] = [];
  const onePixelPng = Buffer.from(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=",
    "base64"
  );
  await page.route("**/video/ros1/snapshot?**", async (route) => {
    snapshotRequests.push(route.request().url());
    await route.fulfill({ status: 200, contentType: "image/png", body: onePixelPng });
  });
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources
  });
  await page.goto("/?workspace=exploration&scenario=exploration.gazebo.tepper");

  await expect.poll(() => page.getByTestId("task-allocation-image").evaluate(
    (image: HTMLImageElement) => image.naturalWidth
  )).toBeGreaterThan(0);
  await expect.poll(() => snapshotRequests.length, { timeout: 6_000 }).toBeGreaterThan(1);
  expect(new Set(snapshotRequests).size).toBeGreaterThan(1);
});

test("allocation image keeps the last frame when a later snapshot refresh fails", async ({ page }) => {
  const dataSources = [
    explorationSource("/task_allocation_image", "sensor_msgs/Image", "task_allocation_image")
  ];
  const snapshotRequests: string[] = [];
  const onePixelPng = Buffer.from(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=",
    "base64"
  );
  await page.route("**/video/ros1/snapshot?**", async (route) => {
    snapshotRequests.push(route.request().url());
    if (snapshotRequests.length === 1) {
      await route.fulfill({ status: 200, contentType: "image/png", body: onePixelPng });
      return;
    }
    await route.fulfill({ status: 503, contentType: "text/plain", body: "allocator stopped" });
  });
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources
  });
  await page.goto("/?workspace=exploration&scenario=exploration.gazebo.tepper");

  const image = page.getByTestId("task-allocation-image");
  await expect.poll(() => image.evaluate((element: HTMLImageElement) => element.naturalWidth)).toBeGreaterThan(0);
  const firstSrc = await image.getAttribute("src");
  await expect.poll(() => snapshotRequests.length, { timeout: 6_000 }).toBeGreaterThan(1);

  await expect(image).toHaveAttribute("src", firstSrc ?? "");
  await expect.poll(() => image.evaluate((element: HTMLImageElement) => element.naturalWidth)).toBeGreaterThan(0);
  await expect(page.getByTestId("panel-map")).not.toContainText("任务分配图连接重试中");
});

test("allocation image keeps the last frame when the allocator topic disappears after HOME", async ({ page }) => {
  const discoveredSource = explorationSource(
    "/task_allocation_image",
    "sensor_msgs/Image",
    "task_allocation_image"
  );
  const profileSource = {
    ...discoveredSource,
    topic: undefined,
    message_type: undefined,
    availability: "UNKNOWN"
  };
  const onePixelPng = Buffer.from(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=",
    "base64"
  );
  await page.route("**/video/ros1/snapshot?**", async (route) => {
    await route.fulfill({ status: 200, contentType: "image/png", body: onePixelPng });
  });
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources: [profileSource]
  });
  await installMockRosbridge(page, [discoveredSource]);
  await page.goto("/?workspace=exploration&scenario=exploration.gazebo.tepper");

  const image = page.getByTestId("task-allocation-image");
  await expect.poll(() => image.evaluate((element: HTMLImageElement) => element.naturalWidth)).toBeGreaterThan(0);
  await page.evaluate(() => {
    const bridgeWindow = window as Window & {
      __masarRosSetContracts: (contracts: Array<{ topic: string; message_type: string }>) => void;
    };
    bridgeWindow.__masarRosSetContracts([]);
  });

  await expect(page.getByTestId("panel-map").getByTestId("topic-state-STALE")).toBeVisible({
    timeout: 5_000
  });
  await expect(image).toBeVisible();
  await expect.poll(() => image.evaluate((element: HTMLImageElement) => element.naturalWidth)).toBeGreaterThan(0);
});

test("sensor camera retries a silent first stream for the default robot", async ({ page }) => {
  const topic = "/robosar_agent_bringup_node/agent_0/camera/image_raw";
  const dataSources = [
    explorationSource(topic, "sensor_msgs/Image", "camera")
  ];
  const streamRequests: string[] = [];
  let stalledRoute: Parameters<Parameters<typeof page.route>[1]>[0] | null = null;
  const onePixelPng = Buffer.from(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=",
    "base64"
  );
  await page.route("**/video/ros1/stream?**", async (route) => {
    streamRequests.push(route.request().url());
    if (streamRequests.length === 1) {
      stalledRoute = route;
      return;
    }
    await route.fulfill({ status: 200, contentType: "image/png", body: onePixelPng });
  });
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources
  });

  try {
    await page.goto("/?workspace=exploration&scenario=exploration.gazebo.tepper");

    await expect(page.getByLabel("选择观测机器人")).toHaveValue("0");
    await expect(page.getByTestId("sensor-source-topic")).toHaveText(topic);
    await expect.poll(() => streamRequests.length, { timeout: 9_000 }).toBeGreaterThan(1);
    await expect.poll(() => page.locator("img.camera-stream").evaluate(
      (image: HTMLImageElement) => image.naturalWidth
    )).toBeGreaterThan(0);
  } finally {
    await stalledRoute?.abort("failed").catch(() => undefined);
  }
});

test("Burger D435 keeps one selected live stream instead of prewarming four cameras", async ({ page }) => {
  const topics = Array.from({ length: 4 }, (_, index) => `/tb_${index}/camera/color/image_raw`);
  const snapshotRequests: string[] = [];
  const streamTopics: string[] = [];
  const onePixelPng = Buffer.from(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=",
    "base64"
  );
  await page.route("**/video/ros1/snapshot?**", async (route) => {
    snapshotRequests.push(new URL(route.request().url()).searchParams.get("topic") ?? "");
    await route.fulfill({ status: 200, contentType: "image/png", body: onePixelPng });
  });
  await page.route("**/video/ros1/stream?**", async (route) => {
    streamTopics.push(new URL(route.request().url()).searchParams.get("topic") ?? "");
    await route.fulfill({ status: 200, contentType: "image/png", body: onePixelPng });
  });
  await installMockApi(page, {
    activeRun: true,
    runScenario: "control.gazebo.response-controller",
    runParameters: { robot: "burger" },
    dataSources: topics.map((topic) => explorationSource(topic, "sensor_msgs/Image", "camera"))
  });

  await page.goto("/?workspace=group_behavior&scenario=control.gazebo.response-controller");

  await expect.poll(() => streamTopics.at(-1)).toBe(topics[0]);
  await expect.poll(() => page.locator("img.camera-stream").evaluate(
    (image: HTMLImageElement) => image.naturalWidth
  )).toBeGreaterThan(0);
  for (let index = 1; index < 4; index += 1) {
    await page.getByLabel("选择观测机器人").selectOption(String(index));
    await expect(page.getByTestId("sensor-source-topic")).toHaveText(topics[index]);
    await expect.poll(() => streamTopics.at(-1)).toBe(topics[index]);
    await expect(page.getByTestId("sensor-camera-state")).toHaveCount(0);
    await expect.poll(() => page.locator("img.camera-stream").evaluate(
      (image: HTMLImageElement) => image.naturalWidth
    )).toBeGreaterThan(0);
  }
  expect(snapshotRequests).toEqual([]);
  expect(streamTopics).toEqual(topics);
});

test("starting a run sends the frozen recording contract", async ({ page }) => {
  await installMockApi(page);
  await page.goto("/?workspace=exploration");

  const requestPromise = page.waitForRequest(
    (request) => request.method() === "POST" && request.url().endsWith("/api/v1/runs")
  );
  await page.getByRole("button", { name: "启动环境" }).click();
  const request = await requestPromise;
  const body = request.postDataJSON() as Record<string, unknown>;

  expect(body).toMatchObject({
    scenario_id: "exploration.gazebo.tepper",
    parameters: {},
    recording: { enabled: false, profile: "default" }
  });
  await expect(page.getByTestId("run-id")).toHaveText("run-new");
  await expect(page.getByTestId("panel-health").getByTestId("topic-state-STALE")).toBeVisible();
});

test("refresh restores the persisted run id when current is temporarily empty", async ({ page }) => {
  await installMockApi(page, { restorableRun: true });
  await page.addInitScript(() => localStorage.setItem("masar.activeRunId", "run-restored"));

  await page.goto("/?workspace=exploration");

  await expect(page.getByTestId("run-id")).toHaveText("run-restored");
  await expect(page.getByTestId("workbench")).toHaveAttribute("data-workspace", "exploration");
});

test("active run restores its resolved parameters instead of scenario defaults", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runParameters: { pattern: "aggregation_pattern", number_robots: 3 }
  });

  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByRole("combobox", { name: "群行为策略" })).toHaveValue("aggregation_pattern");
  await expect(page.getByRole("spinbutton", { name: "执行机器人数量" })).toHaveValue("3");
});

test("run state follows lifecycle events from the shared event socket", async ({ page }) => {
  await installMockApi(page);
  let socket: WebSocketRoute | null = null;
  await page.routeWebSocket("**/api/v1/events*", (route) => {
    socket = route;
  });
  await page.goto("/?workspace=exploration");

  await page.getByRole("button", { name: "启动环境" }).click();
  await expect(page.locator(".run-state")).toHaveText("PREFLIGHT");
  await expect(page.getByRole("button", { name: "开始任务" })).toBeDisabled();
  await expect(page.getByRole("button", { name: "停止任务" })).toBeDisabled();
  await expect(page.getByRole("button", { name: "探索组 HOME" })).toBeDisabled();
  await expect.poll(() => socket !== null).toBe(true);

  socket!.send(
    JSON.stringify({
      type: "event",
      event: {
        sequence: 1,
        kind: "RUN_STATE_CHANGED",
        severity: "info",
        wall_time: "2026-08-02T08:00:00Z",
        payload: { state: "RUNNING", reason_code: null }
      }
    })
  );
  await expect(page.locator(".run-state")).toHaveText("RUNNING");
  await expect(page.getByRole("button", { name: "开始任务" })).toBeEnabled();
});

test("cursor recovery applies lifecycle events received before the socket opens", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runState: "PREFLIGHT",
    events: [
      {
        sequence: 3,
        kind: "RUN_STATE_CHANGED",
        severity: "info",
        wall_time: "2026-08-02T08:00:00Z",
        payload: { state: "RUNNING", reason_code: null }
      }
    ]
  });
  await page.routeWebSocket("**/api/v1/events*", () => undefined);
  await page.goto("/?workspace=exploration");

  await expect(page.locator(".run-state")).toHaveText("RUNNING");
});

test("completed runs release live frames before switching workspaces", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "control.gazebo.response-deform",
    runState: "COMPLETED"
  });
  await page.routeWebSocket("**/api/v1/events*", () => undefined);
  await page.goto("/?workspace=group_behavior");

  await expect(page.getByTestId("run-id")).toHaveText("run-layout-test");
  await expect(page.getByTitle("Gazebo 仿真画面")).toHaveCount(0);
  await page.getByRole("button", { name: /探索组运行/ }).click();

  await expect(page.getByTestId("workbench")).toHaveAttribute("data-workspace", "exploration");
  await expect(page.getByTestId("run-id")).toHaveText("尚未创建");
  await expect(page.getByTestId("panel-simulation").getByText("待启动")).toBeVisible();
});

test("Webots uses the same local noVNC viewer contract as Gazebo", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runState: "RUNNING"
  });
  await page.routeWebSocket("**/api/v1/events*", () => undefined);
  await page.goto("/?workspace=group_behavior");

  const frame = page.getByTitle("Webots 仿真画面");
  await expect(frame).toBeVisible();
  await expect(frame).toHaveAttribute("src", /\/display\/webots\/vnc\.html/);
  await expect(page.locator("webots-view")).toHaveCount(0);
  await expect(page.locator("script[src*='cyberbotics.com']")).toHaveCount(0);
});

test("sensor selection follows one robot without covering the camera image", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runParameters: { robot: "e_puck", number_robots: 5 },
    dataSources: [
      ...[0, 1].flatMap((index) => [
        {
          graph: "ros2",
          topic_pattern: "/<robot>/camera/image_color",
          topic: `/robot_${index}/camera/image_color`,
          message_types: ["sensor_msgs/msg/Image"],
          message_type: "sensor_msgs/msg/Image",
          capability: "camera",
          required: false,
          availability: "ONLINE"
        },
        {
          graph: "ros2",
          topic_pattern: "/<robot>/scan",
          topic: `/robot_${index}/scan`,
          message_types: ["sensor_msgs/msg/LaserScan"],
          message_type: "sensor_msgs/msg/LaserScan",
          capability: "laser",
          required: true,
          availability: "ONLINE"
        },
        {
          graph: "ros2",
          topic_pattern: "/<robot>/odom",
          topic: `/robot_${index}/odom`,
          message_types: ["nav_msgs/msg/Odometry"],
          message_type: "nav_msgs/msg/Odometry",
          capability: "odometry",
          required: true,
          availability: "ONLINE"
        }
      ])
    ]
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  const selector = page.getByLabel("选择观测机器人");
  await expect(selector.locator("option")).toHaveCount(5);
  await selector.selectOption("1");
  await expect(page.getByTestId("sensor-source-topic")).toHaveText("/robot_1/camera/image_color");
  await expect(page.locator(".sensor-camera .camera-reticle")).toHaveCount(0);
  await expect(page.getByTestId("panel-sensors").getByText("在线", { exact: true })).toHaveCount(0);
});

test("five-robot Run keeps all selectors and health rows when ROS discovery is partial", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runParameters: { robot: "e_puck", number_robots: 5 },
    dataSources: [0, 2].flatMap((index) => [
      {
        graph: "ros2",
        topic_pattern: "/<robot>/scan",
        topic: `/robot_${index}/scan`,
        message_types: ["sensor_msgs/msg/LaserScan"],
        message_type: "sensor_msgs/msg/LaserScan",
        capability: "laser",
        required: true,
        availability: "WAITING"
      },
      {
        graph: "ros2",
        topic_pattern: "/<robot>/odom",
        topic: `/robot_${index}/odom`,
        message_types: ["nav_msgs/msg/Odometry"],
        message_type: "nav_msgs/msg/Odometry",
        capability: "odometry",
        required: true,
        availability: index === 0 ? "ONLINE" : "WAITING"
      }
    ])
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByLabel("选择观测机器人").locator("option")).toHaveCount(5);
  await expect(page.getByTestId("health-robot-row")).toHaveCount(5);
  await expect(page.getByTestId("panel-health").getByText("机器人 4", { exact: true })).toBeVisible();
  await expect(page.getByTestId("panel-health")).not.toContainText("等待数据");
  await expect(page.getByTestId("panel-health")).toContainText("已发现");
  await expect(page.getByTestId("panel-health")).toContainText("未发现机器人话题");
});

test("complete rescue separates camera selectors and online exploration count", async ({ page }) => {
  const dataSources = [
    ...Array.from({ length: 5 }, (_, index) => [
      explorationSource(
        `/robosar_agent_bringup_node/agent_${index}/odom_data_euler`,
        "nav_msgs/Odometry",
        "odometry"
      ),
      explorationSource(
        `/robosar_agent_bringup_node/agent_${index}/camera/image_raw`,
        "sensor_msgs/Image",
        "camera"
      ),
      explorationSource(
        `/robosar_agent_bringup_node/agent_${index}/feedback/scan`,
        "sensor_msgs/LaserScan",
        "laser"
      )
    ]).flat(),
    ...Array.from({ length: 4 }, (_, index) => [
      explorationSource(`/tb_${index}/odom`, "nav_msgs/Odometry", "odometry"),
      explorationSource(`/tb_${index}/camera/color/image_raw`, "sensor_msgs/Image", "camera"),
      explorationSource(`/tb_${index}/scan`, "sensor_msgs/LaserScan", "laser"),
      explorationSource(`/tb_${index}/imu`, "sensor_msgs/Imu", "imu")
    ]).flat()
  ];
  const onePixelPng = Buffer.from(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=",
    "base64"
  );
  await page.route("**/video/ros1/snapshot?**", async (route) => {
    await route.fulfill({ status: 200, contentType: "image/png", body: onePixelPng });
  });
  await installMockApi(page, {
    activeRun: true,
    runScenario: "rescue.gazebo.tepper-response",
    dataSources
  });
  await installMockRosbridge(page, dataSources);
  await page.goto("/?workspace=rescue&scenario=rescue.gazebo.tepper-response");

  const selector = page.getByLabel("选择观测机器人");
  const laserReading = page.getByTestId("panel-sensors")
    .locator(".sensor-readouts > div")
    .filter({ hasText: "LaserScan" })
    .locator("strong");
  await expect(selector.locator("option")).toHaveCount(9);
  await expect(selector.locator("option").first()).toHaveText("探索机器人 0");
  await expect(selector.locator("option").last()).toHaveText("响应机器人 3");
  await expect(page.getByTestId("infrared-reading")).toHaveCount(0);
  await expect(page.getByTestId("imu-reading")).toHaveCount(0);
  await publishRos(page, "/robosar_agent_bringup_node/agent_0/feedback/scan", {
    range_min: 0.1,
    range_max: 8,
    ranges: [0.4, 1.2]
  });
  await expect(laserReading).toContainText("最近 0.40 m");
  await selector.selectOption("tb:0");
  await expect(page.getByTestId("sensor-source-topic")).toHaveText("/tb_0/camera/color/image_raw");
  await publishRos(page, "/tb_0/scan", {
    range_min: 0.1,
    range_max: 3.5,
    ranges: [1.6, 2.1]
  });
  await expect(laserReading).toContainText("最近 1.60 m");
  await publishRos(page, "/tb_0/imu", {
    linear_acceleration: { x: 0, y: 0, z: 9.81 },
    angular_velocity: { x: 0, y: 0, z: 0.2 }
  });
  await expect(page.getByTestId("imu-reading")).toContainText("加速度 9.81 m/s²");
  await expect(page.getByTestId("infrared-reading")).toHaveCount(0);

  await expect.poll(() => page.evaluate(() => (
    window as Window & { __masarRosSubscriptions: Set<string> }
  ).__masarRosSubscriptions.has("/robosar_agent_bringup_node/agent_4/odom_data_euler"))).toBe(true);
  for (let index = 0; index < 5; index += 1) {
    await publishRos(page, `/robosar_agent_bringup_node/agent_${index}/odom_data_euler`, odometryMessage(index, index));
  }
  for (let index = 0; index < 4; index += 1) {
    await publishRos(page, `/tb_${index}/odom`, odometryMessage(index + 10, index + 10));
  }
  await expect(page.getByTestId("active-agent-count")).toHaveText("5/5");
});

test("Burger sensor deck explains that the platform has no camera", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runParameters: { robot: "burger", number_robots: 3 },
    dataSources: [{
      graph: "ros2",
      topic_pattern: "/<robot>/scan",
      topic: "/robot_0/scan",
      message_types: ["sensor_msgs/msg/LaserScan"],
      message_type: "sensor_msgs/msg/LaserScan",
      capability: "laser",
      required: true,
      availability: "WAITING"
    }]
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByTestId("sensor-camera-state")).toHaveText("Burger 未配备相机");
  await expect(page.getByTestId("sensor-source-topic")).toHaveText("未配备");
  await expect(page.getByTestId("panel-sensors")).not.toContainText("等待数据");
});

test("group behavior uses a dedicated dynamic XY trajectory panel", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runParameters: { robot: "e_puck", number_robots: 5 }
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  const panel = page.getByTestId("panel-trajectory");
  await expect(panel.getByRole("heading", { name: "机器人运行轨迹" })).toBeVisible();
  await expect(panel.getByTestId("trajectory-plot")).toBeVisible();
  await expect(panel.getByTestId("trajectory-robot-row")).toHaveCount(5);
  await expect(panel.locator(".ros-map-canvas")).toHaveCount(0);
});

test("e-puck sensor deck exposes all discovered infrared channels", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runParameters: { robot: "e_puck", number_robots: 5 },
    dataSources: Array.from({ length: 8 }, (_, index) => ({
      topic_pattern: `/<robot>/ps${index}`,
      topic: `/robot_0/ps${index}`,
      capability: "infrared",
      required: false,
      availability: "ONLINE"
    }))
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByTestId("infrared-reading")).toContainText("8 路");
});

test("trajectory and health list every discovered robot instead of truncating at six", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    dataSources: Array.from({ length: 8 }, (_, index) => ({
      topic_pattern: "/<robot>/odom",
      topic: `/robot_${index}/odom`,
      capability: "odometry",
      required: true,
      availability: "ONLINE"
    }))
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByTestId("trajectory-robot-row")).toHaveCount(8);
  await expect(page.getByTestId("health-robot-row")).toHaveCount(8);
  await expect(page.getByTestId("panel-health").getByText("机器人 7", { exact: true })).toBeVisible();
});

test("process output reads the selected Run process and stream through the log cursor API", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    events: [
      {
        sequence: 1,
        kind: "STEP_STARTED",
        severity: "info",
        wall_time: "2026-08-02T08:00:00Z",
        payload: { step_id: "simulation", agent: "ros2", command_ref: "ros2.swarm" }
      }
    ],
    processLogs: {
      "simulation:stdout": "controller ready\n",
      "simulation:stderr": "webots warning\n"
    }
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByLabel("日志进程")).toHaveValue("simulation");
  await expect(page.getByTestId("process-log-output")).toContainText("controller ready");
  await page.getByRole("button", { name: "标准错误" }).click();
  await expect(page.getByTestId("process-log-output")).toContainText("webots warning");
});

test("discussed dispersion exposes its real opinion exchange sources", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    runParameters: { pattern: "discussed_dispersion_pattern", number_robots: 2 },
    dataSources: [
      {
        topic_pattern: "/<robot>/opinion",
        topic: "/robot_0/opinion",
        capability: "discussion_opinion",
        required: false,
        availability: "ONLINE"
      },
      {
        topic_pattern: "/<robot>/dispersion_distance",
        topic: "/robot_0/dispersion_distance",
        capability: "dispersion_distance",
        required: false,
        availability: "WAITING"
      },
      {
        topic_pattern: "/majority_broadcast",
        topic: "/majority_broadcast",
        capability: "discussion_broadcast",
        required: false,
        availability: "ONLINE"
      }
    ]
  });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByRole("combobox", { name: "群行为策略" })).toHaveValue("discussed_dispersion_pattern");
  const discussion = page.getByTestId("discussion-state");
  await expect(discussion).toContainText("观点交换中");
  await expect(discussion).toContainText("/robot_0/opinion");
  const health = page.getByTestId("panel-health");
  await expect(health).toContainText("讨论观点 / 分散距离");
  await expect(health).not.toContainText("discussion_opinion");
  await expect(health).not.toContainText("dispersion_distance");
});
