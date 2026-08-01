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
        {renderPanel(popupPanel, state.scenario, state)}
      </main>
    );
  }

  const workspaceScenarios = state.scenarios.filter(
    (scenario) => scenario.workspace === state.selectedWorkspace
  );
  const running = state.run !== null && !["COMPLETED", "FAILED"].includes(state.run.state);

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
          <i className={state.error ? "is-error" : ""} aria-hidden="true" />
          <span>{state.error ? "控制面异常" : "控制面在线"}</span>
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
              onChange={(event) => void runStore.selectScenario(event.target.value)}
            >
              {workspaceScenarios.map((scenario) => (
                <option key={scenario.scenario_id} value={scenario.scenario_id}>
                  {scenarioLabels[scenario.scenario_id] ?? scenario.scenario_id}
                </option>
              ))}
            </select>
          </label>
          {state.scenario ? <ParameterEditor scenario={state.scenario} values={state.parameters} /> : null}
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
              <MapPanel sources={state.dataSources} trajectoryOnly />
            )}
            {state.scenario.panels.includes("map") ? <MapPanel sources={state.dataSources} /> : null}
            <SensorPanel sources={state.dataSources} running={running} />
          </section>
        ) : null}

        <aside className="status-rail">
          {state.selectedWorkspace !== "group_behavior" || state.scenario?.module === "control" ? (
            <MissionPanel run={state.run} events={state.events} workspace={state.selectedWorkspace} />
          ) : null}
          {state.selectedWorkspace === "rescue" ? <ResponsePanel sources={state.dataSources} /> : null}
          <HealthPanel sources={state.dataSources} running={running} />
          <EventLogPanel events={state.events} />
          <ProcessLogPanel running={running} />
        </aside>
      </main>

      {state.error ? (
        <div className="error-strip" role="alert">
          <strong>无法完成操作</strong>
          <span>{state.error}</span>
        </div>
      ) : null}
    </div>
  );
}

function CommandToolbar({ scenario, running }: { scenario: ScenarioDetail | null; running: boolean }) {
  const run = useRunStore((state) => state.run);
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
              <button type="button" onClick={() => void runStore.issueCommand("mission.start")}>开始任务</button>
            ) : null}
            {scenario?.commands.includes("mission.stop") ? (
              <button type="button" onClick={() => void runStore.issueCommand("mission.stop")}>停止任务</button>
            ) : null}
            {scenario?.commands.includes("mission.home") ? (
              <button type="button" onClick={() => void runStore.issueCommand("mission.home")}>探索组 HOME</button>
            ) : null}
            {scenario?.commands.includes("response.return_home") ? (
              <button type="button" onClick={() => void runStore.issueCommand("response.return_home")}>响应组返航</button>
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

function ParameterEditor({ scenario, values }: { scenario: ScenarioDetail; values: Record<string, unknown> }) {
  const entries = Object.entries(scenario.parameters);
  if (!entries.length) {
    return (
      <div className="parameter-empty">
        <span>场景参数</span>
        <p>使用已验收的仓库默认配置</p>
      </div>
    );
  }
  const preferred = entries.filter(([name]) => name in parameterLabels).slice(0, 6);
  return (
    <div className="parameter-list">
      <div className="section-label"><span>场景参数</span><small>{preferred.length} 项可调</small></div>
      {preferred.map(([name, definition]) => (
        <ParameterField key={name} name={name} definition={definition} value={values[name]} />
      ))}
    </div>
  );
}

function ParameterField({
  name,
  definition,
  value
}: {
  name: string;
  definition: ParameterDefinition;
  value: unknown;
}) {
  const label = parameterLabels[name] ?? name;
  if (definition.type === "boolean") {
    return (
      <label className="toggle-field">
        <span>{label}</span>
        <input
          type="checkbox"
          checked={Boolean(value)}
          onChange={(event) => runStore.setParameter(name, event.target.checked)}
        />
      </label>
    );
  }
  if (definition.enum) {
    return (
      <label className="field-block field-block--compact">
        <span>{label}</span>
        <select value={String(value ?? "")} onChange={(event) => runStore.setParameter(name, event.target.value)}>
          {definition.enum.map((option) => (
            <option key={String(option)} value={String(option)}>{humanizeOption(String(option))}</option>
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
        value={String(value ?? "")}
        onChange={(event) => runStore.setParameter(name, Number(event.target.value))}
      />
    </label>
  );
}

function humanizeOption(value: string): string {
  return value.replace(/_pattern(?:_neat)?$/, "").replaceAll("_", " · ");
}

function renderPanel(
  panelId: string,
  scenario: ScenarioDetail,
  state: ReturnType<typeof runStore.getState>
) {
  const running = state.run !== null && !["COMPLETED", "FAILED"].includes(state.run.state);
  switch (panelId) {
    case "simulation": return <SimulationPanel scenario={scenario} run={state.run} />;
    case "rviz": return <RvizPanel scenario={scenario} run={state.run} />;
    case "map": return <MapPanel sources={state.dataSources} />;
    case "trajectory": return <MapPanel sources={state.dataSources} trajectoryOnly />;
    case "sensors": return <SensorPanel sources={state.dataSources} running={running} />;
    case "mission": return <MissionPanel run={state.run} events={state.events} workspace={state.selectedWorkspace} />;
    case "response": return <ResponsePanel sources={state.dataSources} />;
    case "health": return <HealthPanel sources={state.dataSources} running={running} />;
    case "events": return <EventLogPanel events={state.events} />;
    case "logs": return <ProcessLogPanel running={running} />;
    default: return <SimulationPanel scenario={scenario} run={state.run} />;
  }
}

function LoadingScreen() {
  return (
    <main className="loading-screen">
      <span className="brand-mark">M</span>
      <div><strong>正在建立控制面</strong><small>读取场景与运行状态</small></div>
    </main>
  );
}
