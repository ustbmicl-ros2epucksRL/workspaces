import type { Page, Route } from "@playwright/test";

const scenarios = [
  {
    scenario_id: "swarm.webots.ros2swarm",
    workspace: "group_behavior",
    module: "swarm",
    simulator: "webots",
    display_profile: "swarm",
    panels: ["simulation", "sensors", "health", "logs"],
    commands: ["mission.start", "mission.stop"]
  },
  {
    scenario_id: "control.gazebo.response-deform",
    workspace: "group_behavior",
    module: "control",
    simulator: "gazebo",
    display_profile: "control",
    panels: ["simulation", "sensors", "mission", "health", "logs"],
    commands: ["mission.start", "mission.stop", "response.return_home"]
  },
  {
    scenario_id: "exploration.gazebo.tepper",
    workspace: "exploration",
    module: "exploration",
    simulator: "gazebo",
    display_profile: "exploration",
    panels: ["simulation", "rviz", "map", "sensors", "mission", "health", "logs"],
    commands: ["mission.start", "mission.stop", "mission.home"]
  },
  {
    scenario_id: "exploration.webots.robosar",
    workspace: "exploration",
    module: "exploration",
    simulator: "webots",
    display_profile: "exploration",
    panels: ["simulation", "rviz", "map", "sensors", "mission", "health", "logs"],
    commands: ["mission.start", "mission.stop", "mission.home"]
  },
  {
    scenario_id: "rescue.gazebo.tepper-response",
    workspace: "rescue",
    module: "rescue",
    simulator: "gazebo",
    display_profile: "rescue",
    panels: ["simulation", "rviz", "map", "sensors", "mission", "response", "health", "logs"],
    commands: ["mission.start", "mission.stop", "mission.home", "response.return_home"]
  }
];

function detail(scenarioId: string) {
  const scenario = scenarios.find((item) => item.scenario_id === scenarioId) ?? scenarios[2];
  return {
    ...scenario,
    runtime: { ros1_master_uri: "http://127.0.0.1:11311", ros2_domain_id: 0 },
    parameters:
      scenario.module === "swarm"
        ? {
            pattern: {
              type: "string",
              default: "roundflocking_pattern",
              enum: ["aggregation_pattern", "dispersion_pattern", "roundflocking_pattern"]
            },
            number_robots: { type: "integer", default: 5, minimum: 1, maximum: 20 }
          }
        : {},
    data_profile: scenario.module === "swarm" ? "swarm_sensor" : scenario.module,
    artifact_profile: scenario.module === "rescue" ? "rescue_default" : "exploration_default",
    display_plan: {
      simulation:
        scenario.simulator === "webots"
          ? { transport: "webots", path: "/webots/", display: null, required: true }
          : { transport: "novnc", path: "/display/gazebo/", display: ":101", required: true },
      rviz: scenario.panels.includes("rviz")
        ? {
            agent: "ros1",
            process_id: "display_rviz",
            command_ref: "display.ros1.rviz",
            argv: ["rviz", "-d", "/root/workspace/masar_ws/multi_robot.rviz"],
            environment: { DISPLAY: ":102" },
            required: false
          }
        : null
    }
  };
}

function json(route: Route, body: unknown, status = 200) {
  return route.fulfill({ status, contentType: "application/json", body: JSON.stringify(body) });
}

export async function installMockApi(
  page: Page,
  options: {
    activeRun?: boolean;
    restorableRun?: boolean;
    runScenario?: string;
    dataSources?: Array<Record<string, unknown>>;
  } = {}
) {
  const context = page.context();
  await context.route("https://cyberbotics.com/**", (route) => route.abort());
  await context.route("**/display/**", (route) => route.abort());
  await context.route("**/webots/**", (route) => route.abort());
  await context.route("**/api/v1/**", async (route) => {
    const url = new URL(route.request().url());
    const path = url.pathname;
    if (path === "/api/v1/capabilities") {
      return json(route, {
        schema_version: 1,
        single_active_run: true,
        workspaces: [
          { id: "group_behavior", label: "群行为与控制验证", scenario_modules: ["swarm", "control"] },
          { id: "exploration", label: "探索组独立运行", scenario_modules: ["exploration"] },
          { id: "rescue", label: "完整搜救系统", scenario_modules: ["rescue"] }
        ],
        simulators: ["gazebo", "webots"],
        scenario_count: scenarios.length,
        event_transport: "websocket+cursor"
      });
    }
    if (path === "/api/v1/scenarios") return json(route, { scenarios });
    if (/^\/api\/v1\/scenarios\/.+/.test(path)) {
      return json(route, detail(decodeURIComponent(path.replace("/api/v1/scenarios/", ""))));
    }
    if (path === "/api/v1/runs/current") {
      return json(route, {
        run: options.activeRun
          ? {
              run_id: "run-layout-test",
              scenario_id: options.runScenario ?? "exploration.gazebo.tepper",
              state: "RUNNING",
              reason_code: null,
              created_at: "2026-08-01T10:00:00Z",
              updated_at: "2026-08-01T10:00:03Z",
              ended_at: null,
              parameters: {}
            }
          : null
      });
    }
    if (path === "/api/v1/runs/run-restored" && options.restorableRun) {
      return json(route, {
        run_id: "run-restored",
        scenario_id: "exploration.gazebo.tepper",
        state: "RUNNING",
        reason_code: null,
        created_at: "2026-08-01T10:00:00Z",
        updated_at: "2026-08-01T10:00:03Z",
        ended_at: null,
        parameters: {}
      });
    }
    if (path.endsWith("/data-sources")) {
      return json(route, {
        data_sources: options.dataSources ?? [
          { topic_pattern: "/sim_map", capability: "map", required: true, availability: "ONLINE" },
          { topic_pattern: "/robot_2/camera", capability: "camera", required: false, availability: "UNAVAILABLE" },
          { topic_pattern: "/robot_0/odom", capability: "odometry", required: true, availability: "STALE" },
          { topic_pattern: "/robot_0/scan", capability: "laser", required: true, availability: "TYPE_MISMATCH" },
          { topic_pattern: "/clock", capability: "clock", required: true, availability: "WAITING" },
          { topic_pattern: "/optional", capability: "optional", required: false, availability: "DISCONNECTED" }
        ]
      });
    }
    if (path.endsWith("/events")) return json(route, { events: [] });
    if (route.request().method() === "POST" && path === "/api/v1/runs") {
      return json(route, { run_id: "run-new", state: "CREATED" }, 202);
    }
    return json(route, { error: { code: "NOT_FOUND", message: path, details: {} } }, 404);
  });
}
