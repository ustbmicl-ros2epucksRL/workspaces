import { useEffect } from "react";
import type { ParameterDefinition, ScenarioDetail, WorkspaceId } from "./api";
import { runStore, useRunStore } from "./runStore";
import { EventLogPanel } from "../panels/EventLogPanel";
import { HealthPanel } from "../panels/HealthPanel";
import { MapPanel } from "../panels/MapPanel";
import { MissionPanel } from "../panels/MissionPanel";
import { ProcessLogPanel } from "../panels/ProcessLogPanel";
import { ResponsePanel } from "../panels/ResponsePanel";
import { RvizPanel } from "../panels/RvizPanel";
import { SensorPanel } from "../panels/SensorPanel";
import { SimulationPanel } from "../panels/SimulationPanel";
import { TrajectoryPanel } from "../panels/TrajectoryPanel";
import { useRosDataSources } from "../ros/RosDataRuntime";

const workspaceMeta: Record<
  WorkspaceId,
  { code: string; short: string; description: string }
> = {
  group_behavior: {
    code: "BEHAVIOR",
    short: "群行为验证",
    description: "势场、网络策略、编队与任务控制 demo"
  },
  exploration: {
    code: "EXPLORE",
    short: "探索组运行",
    description: "Gazebo / Webots 中的独立探索与建图"
  },
  rescue: {
    code: "RESCUE",
    short: "完整搜救",
    description: "探索组与响应组共同执行 Tepper 任务"
  }
};

const scenarioLabels: Record<string, string> = {
  "swarm.webots.ros2swarm": "ROS2swarm · Webots 群行为",
  "control.gazebo.response-deform": "响应组 · DEFORM 编队",
  "control.gazebo.response-controller": "响应组 · 控制器 Demo",
  "exploration.gazebo.tepper": "探索组 · Gazebo Tepper",
  "exploration.webots.robosar": "探索组 · Webots RoboSAR",
  "rescue.gazebo.tepper-response": "完整搜救 · Tepper + 响应组"
};

const parameterLabels: Record<string, string> = {
  pattern: "群行为策略",
  number_robots: "执行机器人数量",
  total_robots: "场景机器人总数",
  webots_world: "Webots 世界",
  robot: "机器人类型",
  sensor_type: "主传感器",
  mode: "控制器任务",
  log_level: "日志级别",
  webots_mode: "仿真速度",
  num_obstacles: "障碍物数量"
};

let initializationStarted = false;

export function App() {
  const state = useRunStore((value) => value);
  const observedSources = useRosDataSources(state.run?.run_id ?? null, state.dataSources);
  const dataSources = observedSources.length > 0 ? observedSources : state.dataSources;
  useEffect(() => {
    if (!initializationStarted) {
      initializationStarted = true;
      void runStore.initialize();
    }
  }, []);

  const popupPanel = new URLSearchParams(window.location.search).get("panel");
  if (state.loading && !state.scenario) return <LoadingScreen />;
  if (popupPanel && state.scenario) {
    return (
      <main className="popup-workbench" data-testid="workbench" data-workspace={state.selectedWorkspace}>
        {renderPanel(popupPanel, state.scenario, { ...state, dataSources })}
      </main>
    );
  }

  const workspaceScenarios = state.scenarios.filter(
    (scenario) => scenario.workspace === state.selectedWorkspace
  );
  const running = state.run !== null && !["COMPLETED", "FAILED"].includes(state.run.state);
  const panelDataSources = running ? dataSources : [];
  const expectedRobotCount = running
    ? state.robots.length || integerParameter(state.run?.parameters.number_robots ?? state.parameters.number_robots)
    : 0;
  const robotType = stringParameter(state.parameters.robot);

  return (
    <div className="app-shell">
      <header className="masthead">
        <div className="brand-lockup">
          <span className="brand-mark" aria-hidden="true">M</span>
          <div>
            <strong>MASAR</strong>
            <small>群体机器人验证台</small>
          </div>
        </div>
        <nav className="mission-track" aria-label="验证模块">
          {(Object.keys(workspaceMeta) as WorkspaceId[]).map((workspace) => {
            const meta = workspaceMeta[workspace];
            const selected = workspace === state.selectedWorkspace;
            return (
              <button
                type="button"
                key={workspace}
                className={selected ? "is-selected" : ""}
                aria-current={selected ? "page" : undefined}
                disabled={running}
                onClick={() => void runStore.selectWorkspace(workspace)}
              >
                <i aria-hidden="true" />
                <span>{meta.code}</span>
                <strong>{meta.short}</strong>
              </button>
            );
          })}
        </nav>
        <div className="system-presence">
          <i className={state.controlPlaneStatus === "offline" ? "is-error" : ""} aria-hidden="true" />
          <span>
            {state.controlPlaneStatus === "checking"
              ? "控制面连接中"
              : state.controlPlaneStatus === "offline"
                ? "控制面异常"
                : "控制面在线"}
          </span>
        </div>
      </header>

      <CommandToolbar scenario={state.scenario} running={running} />

      <main
        className={`workbench workbench--${state.selectedWorkspace}`}
        data-testid="workbench"
        data-workspace={state.selectedWorkspace}
      >
        <aside className="scenario-rail">
          <div className="rail-intro">
            <span>{workspaceMeta[state.selectedWorkspace].code}</span>
            <h1>{workspaceMeta[state.selectedWorkspace].short}</h1>
            <p>{workspaceMeta[state.selectedWorkspace].description}</p>
          </div>
          <label className="field-block">
            <span>运行场景</span>
            <select
              value={state.selectedScenarioId ?? ""}
              disabled={running}
              onChange={(event) => void runStore.selectScenario(event.target.value)}
            >
              {workspaceScenarios.map((scenario) => (
                <option key={scenario.scenario_id} value={scenario.scenario_id}>
                  {scenarioLabels[scenario.scenario_id] ?? scenario.scenario_id}
                </option>
              ))}
            </select>
          </label>
          {state.scenario ? <ParameterEditor scenario={state.scenario} values={state.parameters} locked={state.run !== null} /> : null}
          <div className="runtime-contract">
            <span>运行图身份</span>
            <dl>
              <div><dt>ROS1</dt><dd>{state.scenario?.runtime.ros1_master_uri.replace("http://", "") ?? "—"}</dd></div>
              <div><dt>ROS2 Domain</dt><dd>{state.scenario?.runtime.ros2_domain_id ?? "—"}</dd></div>
              <div><dt>仿真器</dt><dd>{state.scenario?.simulator.toUpperCase() ?? "—"}</dd></div>
            </dl>
          </div>
        </aside>

        {state.scenario ? (
          <section className="panel-grid" aria-label="同步观测面板">
            <SimulationPanel scenario={state.scenario} run={state.run} />
            {state.scenario.panels.includes("rviz") ? (
              <RvizPanel scenario={state.scenario} run={state.run} />
            ) : (
              <TrajectoryPanel
                runId={state.run?.run_id ?? null}
                sources={panelDataSources}
                expectedRobotCount={expectedRobotCount}
                running={running}
              />
            )}
            {state.scenario.panels.includes("map") ? (
              <MapPanel runId={state.run?.run_id ?? null} sources={panelDataSources} running={running} />
            ) : null}
            <SensorPanel
              runId={state.run?.run_id ?? null}
              sources={panelDataSources}
              running={running}
              expectedRobotCount={expectedRobotCount}
              robotType={robotType}
            />
          </section>
        ) : null}

        <aside className="status-rail">
          {state.scenario?.panels.includes("mission") ? (
            <MissionPanel
              runId={state.run?.run_id ?? null}
              sources={panelDataSources}
              running={running}
              workspace={state.selectedWorkspace}
              expectedRobotCount={expectedRobotCount}
            />
          ) : null}
          {state.scenario?.panels.includes("response") ? (
            <ResponsePanel runId={state.run?.run_id ?? null} sources={panelDataSources} running={running} />
          ) : null}
          <HealthPanel
            runId={state.run?.run_id ?? null}
            sources={panelDataSources}
            running={running}
            expectedRobotCount={expectedRobotCount}
          />
          <EventLogPanel events={state.events} />
          <ProcessLogPanel runId={state.run?.run_id ?? null} events={state.events} running={running} />
        </aside>
      </main>

      {state.error ? (
        <div className="error-strip" role="alert">
          <strong>无法完成操作</strong>
          <span>{state.error}</span>
          {state.controlPlaneStatus === "offline" ? (
            <button type="button" onClick={() => runStore.reconnect()}>重新连接</button>
          ) : null}
        </div>
      ) : null}
    </div>
  );
}

function CommandToolbar({ scenario, running }: { scenario: ScenarioDetail | null; running: boolean }) {
  const run = useRunStore((state) => state.run);
  const commandsEnabled = run?.state === "RUNNING";
  return (
    <div className="command-toolbar" data-testid="command-toolbar">
      <div className="run-presence">
        <span className={`run-state run-state--${run?.state.toLowerCase() ?? "idle"}`}>
          {run?.state ?? "IDLE"}
        </span>
        <div>
          <small>RUN ID</small>
          <strong data-testid="run-id">{run?.run_id ?? "尚未创建"}</strong>
        </div>
      </div>
      <div className="command-actions">
        {!run ? (
          <button className="command-primary" type="button" disabled={!scenario} onClick={() => void runStore.startRun()}>
            启动环境
          </button>
        ) : ["COMPLETED", "FAILED"].includes(run.state) ? (
          <button className="command-primary" type="button" onClick={() => runStore.clearCompletedRun()}>
            新建运行
          </button>
        ) : (
          <>
            {scenario?.commands.includes("mission.start") ? (
              <button type="button" disabled={!commandsEnabled} onClick={() => void runStore.issueCommand("mission.start")}>开始任务</button>
            ) : null}
            {scenario?.commands.includes("mission.stop") ? (
              <button type="button" disabled={!commandsEnabled} onClick={() => void runStore.issueCommand("mission.stop")}>停止任务</button>
            ) : null}
            {scenario?.commands.includes("mission.home") ? (
              <button type="button" disabled={!commandsEnabled} onClick={() => void runStore.issueCommand("mission.home")}>探索组 HOME</button>
            ) : null}
            {scenario?.commands.includes("response.return_home") ? (
              <button type="button" disabled={!commandsEnabled} onClick={() => void runStore.issueCommand("response.return_home")}>响应组返航</button>
            ) : null}
            <button className="command-danger" type="button" disabled={!running} onClick={() => void runStore.stopRun()}>
              停止运行
            </button>
          </>
        )}
      </div>
      <div className="clock-block" aria-label="平台时钟">
        <span>LOCAL</span>
        <strong>{new Date().toLocaleTimeString("zh-CN", { hour12: false, hour: "2-digit", minute: "2-digit" })}</strong>
      </div>
    </div>
  );
}

function ParameterEditor({
  scenario,
  values,
  locked
}: {
  scenario: ScenarioDetail;
  values: Record<string, unknown>;
  locked: boolean;
}) {
  const entries = Object.entries(scenario.parameters);
  if (!entries.length) {
    return (
      <div className="parameter-empty">
        <span>场景参数</span>
        <p>使用已验收的仓库默认配置</p>
      </div>
    );
  }
  const preferred = entries
    .filter(([name, definition]) => name in parameterLabels && definition.enum?.length !== 1)
    .slice(0, 6);
  return (
    <div className="parameter-list">
      <div className="section-label"><span>场景参数</span><small>{preferred.length} 项可调</small></div>
      {preferred.map(([name, definition]) => (
        <ParameterField key={name} name={name} definition={definition} value={values[name]} locked={locked} />
      ))}
    </div>
  );
}

function ParameterField({
  name,
  definition,
  value,
  locked
}: {
  name: string;
  definition: ParameterDefinition;
  value: unknown;
  locked: boolean;
}) {
  const label = parameterLabels[name] ?? name;
  if (definition.type === "boolean") {
    return (
      <label className="toggle-field">
        <span>{label}</span>
        <input
          type="checkbox"
          checked={Boolean(value)}
          disabled={locked}
          onChange={(event) => runStore.setParameter(name, event.target.checked)}
        />
      </label>
    );
  }
  if (definition.enum) {
    return (
      <label className="field-block field-block--compact">
        <span>{label}</span>
        <select disabled={locked} value={String(value ?? "")} onChange={(event) => runStore.setParameter(name, event.target.value)}>
          {definition.enum.map((option) => (
            <option key={String(option)} value={String(option)}>{formatParameterOption(name, String(option))}</option>
          ))}
        </select>
      </label>
    );
  }
  return (
    <label className="field-block field-block--compact">
      <span>{label}</span>
      <input
        type="number"
        min={definition.minimum}
        max={definition.maximum}
        disabled={locked}
        value={String(value ?? "")}
        onChange={(event) => runStore.setParameter(name, Number(event.target.value))}
      />
    </label>
  );
}

function humanizeOption(value: string): string {
  return value.replace(/_pattern(?:_neat)?$/, "").replaceAll("_", " · ");
}

function formatParameterOption(name: string, value: string): string {
  if (name === "mode" && value === "transport") return "transport-synchronized_slots";
  return name === "robot" ? value : humanizeOption(value);
}

function renderPanel(
  panelId: string,
  scenario: ScenarioDetail,
  state: ReturnType<typeof runStore.getState>
) {
  const running = state.run !== null && !["COMPLETED", "FAILED"].includes(state.run.state);
  const panelSources = running ? state.dataSources : [];
  const expectedRobotCount = running
    ? state.robots.length || integerParameter(state.run?.parameters.number_robots ?? state.parameters.number_robots)
    : 0;
  const robotType = stringParameter(state.parameters.robot);
  switch (panelId) {
    case "simulation": return <SimulationPanel scenario={scenario} run={state.run} />;
    case "rviz": return <RvizPanel scenario={scenario} run={state.run} />;
    case "map": return <MapPanel runId={state.run?.run_id ?? null} sources={panelSources} running={running} />;
    case "trajectory": return (
      <TrajectoryPanel
        runId={state.run?.run_id ?? null}
        sources={panelSources}
        expectedRobotCount={expectedRobotCount}
        running={running}
      />
    );
    case "sensors": return (
      <SensorPanel
        runId={state.run?.run_id ?? null}
        sources={panelSources}
        running={running}
        expectedRobotCount={expectedRobotCount}
        robotType={robotType}
      />
    );
    case "mission": return (
      <MissionPanel
        runId={state.run?.run_id ?? null}
        sources={panelSources}
        running={running}
        workspace={state.selectedWorkspace}
        expectedRobotCount={expectedRobotCount}
      />
    );
    case "response": return (
      <ResponsePanel runId={state.run?.run_id ?? null} sources={panelSources} running={running} />
    );
    case "health": return (
      <HealthPanel
        runId={state.run?.run_id ?? null}
        sources={panelSources}
        running={running}
        expectedRobotCount={expectedRobotCount}
      />
    );
    case "events": return <EventLogPanel events={state.events} />;
    case "logs": return <ProcessLogPanel runId={state.run?.run_id ?? null} events={state.events} running={running} />;
    default: return <SimulationPanel scenario={scenario} run={state.run} />;
  }
}

function integerParameter(value: unknown): number {
  return typeof value === "number" && Number.isInteger(value) && value > 0 ? value : 0;
}

function stringParameter(value: unknown): string | null {
  return typeof value === "string" ? value : null;
}

function LoadingScreen() {
  return (
    <main className="loading-screen">
      <span className="brand-mark">M</span>
      <div><strong>正在建立控制面</strong><small>读取场景与运行状态</small></div>
    </main>
  );
}
