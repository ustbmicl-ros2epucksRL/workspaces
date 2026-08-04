import { useSyncExternalStore } from "react";
import {
  api,
  ApiError,
  eventSocketUrl,
  type Capabilities,
  type DataSource,
  type RunEvent,
  type RunRecord,
  type RobotInventoryItem,
  type RunState,
  type ScenarioDetail,
  type ScenarioSummary,
  type WorkspaceId
} from "./api";

export type PanelSize = "normal" | "wide";
export type ControlPlaneStatus = "checking" | "online" | "offline";

export interface PanelState {
  size: PanelSize;
  maximized: boolean;
}

export interface WorkbenchState {
  loading: boolean;
  controlPlaneStatus: ControlPlaneStatus;
  error: string | null;
  capabilities: Capabilities | null;
  scenarios: ScenarioSummary[];
  selectedWorkspace: WorkspaceId;
  selectedScenarioId: string | null;
  scenario: ScenarioDetail | null;
  parameters: Record<string, unknown>;
  run: RunRecord | null;
  dataSources: DataSource[];
  robots: RobotInventoryItem[];
  events: RunEvent[];
  eventCursor: number;
  panels: Record<string, PanelState>;
}

const ACTIVE_RUN_KEY = "masar.activeRunId";
const CURSOR_PREFIX = "masar.eventCursor.";
const defaultPanel = (): PanelState => ({ size: "normal", maximized: false });

function requestedWorkspace(): WorkspaceId {
  const value = new URLSearchParams(window.location.search).get("workspace");
  return value === "exploration" || value === "rescue" ? value : "group_behavior";
}

class RunStore {
  private state: WorkbenchState = {
    loading: true,
    controlPlaneStatus: "checking",
    error: null,
    capabilities: null,
    scenarios: [],
    selectedWorkspace: requestedWorkspace(),
    selectedScenarioId: null,
    scenario: null,
    parameters: {},
    run: null,
    dataSources: [],
    robots: [],
    events: [],
    eventCursor: 0,
    panels: {}
  };
  private listeners = new Set<() => void>();
  private eventSocket: WebSocket | null = null;

  subscribe = (listener: () => void): (() => void) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };

  getState = (): WorkbenchState => this.state;

  async initialize(): Promise<void> {
    this.set({
      loading: this.state.scenario === null,
      controlPlaneStatus: "checking",
      error: null
    });
    try {
      const [capabilities, scenarios, currentRun] = await Promise.all([
        api.capabilities(),
        api.scenarios(),
        api.currentRun()
      ]);
      const params = new URLSearchParams(window.location.search);
      const requestedScenario = params.get("scenario");
      const requestedRunId = params.get("run_id") ?? localStorage.getItem(ACTIVE_RUN_KEY);
      let run = currentRun;
      if (!run && requestedRunId) {
        try {
          run = await api.run(requestedRunId);
        } catch {
          localStorage.removeItem(ACTIVE_RUN_KEY);
        }
      }
      let workspace = this.state.selectedWorkspace;
      if (run) {
        workspace = scenarios.find((item) => item.scenario_id === run.scenario_id)?.workspace ?? workspace;
      }
      const candidates = scenarios.filter((item) => item.workspace === workspace);
      const selected =
        candidates.find((item) => item.scenario_id === run?.scenario_id) ??
        candidates.find((item) => item.scenario_id === requestedScenario) ??
        candidates[0] ??
        scenarios[0];
      const scenario = selected ? await api.scenario(selected.scenario_id) : null;
      const parameters = scenario ? { ...defaultsFor(scenario), ...(run?.parameters ?? {}) } : {};
      this.set({
        loading: false,
        controlPlaneStatus: "online",
        error: null,
        capabilities,
        scenarios,
        selectedWorkspace: workspace,
        selectedScenarioId: selected?.scenario_id ?? null,
        scenario,
        parameters,
        run
      });
      if (run) await this.attachRun(run);
    } catch (error) {
      this.set({ loading: false, controlPlaneStatus: "offline", error: messageFor(error) });
    }
  }

  reconnect(): void {
    void this.initialize();
  }

  async selectWorkspace(workspace: WorkspaceId): Promise<void> {
    if (this.hasActiveRun()) {
      this.set({ error: "当前运行结束后才能切换验证模块" });
      return;
    }
    this.clearCompletedRun();
    const selected = this.state.scenarios.find((item) => item.workspace === workspace);
    this.set({ selectedWorkspace: workspace, error: null });
    updateLocation({ workspace, scenario: selected?.scenario_id ?? null });
    if (selected) await this.loadScenario(selected.scenario_id);
  }

  async selectScenario(scenarioId: string): Promise<void> {
    if (this.hasActiveRun()) {
      this.set({ error: "当前运行结束后才能切换场景" });
      return;
    }
    this.clearCompletedRun();
    updateLocation({ workspace: this.state.selectedWorkspace, scenario: scenarioId });
    await this.loadScenario(scenarioId);
  }

  setParameter(name: string, value: unknown): void {
    this.set({ parameters: { ...this.state.parameters, [name]: value } });
  }

  async startRun(): Promise<void> {
    if (!this.state.scenario || this.state.run) return;
    this.set({ error: null });
    try {
      const created = await api.createRun(this.state.scenario.scenario_id, this.state.parameters);
      const now = new Date().toISOString();
      const run: RunRecord = {
        run_id: created.run_id,
        scenario_id: this.state.scenario.scenario_id,
        state: created.state,
        reason_code: null,
        created_at: now,
        updated_at: now,
        ended_at: null,
        parameters: this.state.parameters
      };
      this.set({ run });
      await this.attachRun(run);
    } catch (error) {
      this.set({ error: messageFor(error) });
    }
  }

  async stopRun(): Promise<void> {
    if (!this.state.run) return;
    try {
      const run = await api.stopRun(this.state.run.run_id);
      this.set({ run });
      if (run.state === "COMPLETED" || run.state === "FAILED") this.closeEventSocket();
    } catch (error) {
      this.set({ error: messageFor(error) });
    }
  }

  async issueCommand(command: string): Promise<void> {
    if (!this.state.run) return;
    try {
      await api.command(this.state.run.run_id, command);
    } catch (error) {
      this.set({ error: messageFor(error) });
    }
  }

  clearCompletedRun(): void {
    if (this.state.run && !this.hasActiveRun()) {
      localStorage.removeItem(ACTIVE_RUN_KEY);
      this.set({ run: null, dataSources: [], robots: [], events: [], eventCursor: 0, error: null });
    }
  }

  togglePanelSize(panelId: string): void {
    const current = this.state.panels[panelId] ?? defaultPanel();
    this.setPanel(panelId, { ...current, size: current.size === "normal" ? "wide" : "normal" });
  }

  maximizePanel(panelId: string): void {
    const panels = Object.fromEntries(
      Object.entries(this.state.panels).map(([id, value]) => [id, { ...value, maximized: false }])
    );
    panels[panelId] = { ...(panels[panelId] ?? defaultPanel()), maximized: true };
    this.set({ panels });
  }

  restorePanel(panelId: string): void {
    this.setPanel(panelId, { ...(this.state.panels[panelId] ?? defaultPanel()), maximized: false });
  }

  panelState(panelId: string): PanelState {
    return this.state.panels[panelId] ?? defaultPanel();
  }

  popoutPanel(panelId: string): void {
    const params = new URLSearchParams({ panel: panelId });
    if (this.state.run) params.set("run_id", this.state.run.run_id);
    params.set("workspace", this.state.selectedWorkspace);
    if (this.state.selectedScenarioId) params.set("scenario", this.state.selectedScenarioId);
    window.open(`/?${params.toString()}`, `masar-panel-${panelId}`, "popup,width=1040,height=720");
  }

  private async loadScenario(scenarioId: string): Promise<void> {
    this.set({ loading: true, selectedScenarioId: scenarioId, error: null });
    try {
      const scenario = await api.scenario(scenarioId);
      this.set({ loading: false, scenario, parameters: defaultsFor(scenario) });
    } catch (error) {
      this.set({ loading: false, error: messageFor(error) });
    }
  }

  private async attachRun(run: RunRecord): Promise<void> {
    localStorage.setItem(ACTIVE_RUN_KEY, run.run_id);
    const cursor = Number(localStorage.getItem(CURSOR_PREFIX + run.run_id) ?? "0");
    try {
      const [dataSources, robots, events] = await Promise.all([
        api.dataSources(run.run_id),
        api.robots(run.run_id),
        api.events(run.run_id, Number.isFinite(cursor) ? cursor : 0)
      ]);
      const eventCursor = events.at(-1)?.sequence ?? (Number.isFinite(cursor) ? cursor : 0);
      const nextRun = events.reduce<RunRecord | null>(applyRunEvent, this.state.run ?? run);
      this.set({ dataSources, robots, events, eventCursor, run: nextRun });
      localStorage.setItem(CURSOR_PREFIX + run.run_id, String(eventCursor));
      if (this.hasActiveRun()) this.connectEvents(run.run_id, eventCursor);
    } catch (error) {
      this.set({ error: messageFor(error), dataSources: [], robots: [] });
    }
  }

  private connectEvents(runId: string, after: number): void {
    this.closeEventSocket();
    const socket = new WebSocket(eventSocketUrl(runId, after));
    this.eventSocket = socket;
    socket.onmessage = (message) => {
      const envelope = JSON.parse(String(message.data)) as { type: string; event?: RunEvent };
      if (envelope.type !== "event" || !envelope.event) return;
      const event = envelope.event;
      const run = this.state.run;
      const nextRun = run?.run_id === runId ? applyRunEvent(run, event) : run;
      this.set({ events: [...this.state.events, event].slice(-200), eventCursor: event.sequence, run: nextRun });
      localStorage.setItem(CURSOR_PREFIX + runId, String(event.sequence));
      if (nextRun && isTerminalState(nextRun.state)) this.closeEventSocket();
    };
  }

  private closeEventSocket(): void {
    this.eventSocket?.close();
    this.eventSocket = null;
  }

  private setPanel(panelId: string, panel: PanelState): void {
    this.set({ panels: { ...this.state.panels, [panelId]: panel } });
  }

  private hasActiveRun(): boolean {
    return this.state.run !== null && !["COMPLETED", "FAILED"].includes(this.state.run.state);
  }

  private set(patch: Partial<WorkbenchState>): void {
    this.state = { ...this.state, ...patch };
    for (const listener of this.listeners) listener();
  }
}

function defaultsFor(scenario: ScenarioDetail): Record<string, unknown> {
  return Object.fromEntries(
    Object.entries(scenario.parameters).map(([name, definition]) => [name, definition.default])
  );
}

function updateLocation(values: { workspace: WorkspaceId; scenario: string | null }): void {
  const params = new URLSearchParams(window.location.search);
  params.set("workspace", values.workspace);
  if (values.scenario) params.set("scenario", values.scenario);
  else params.delete("scenario");
  history.replaceState(null, "", `${window.location.pathname}?${params.toString()}`);
}

function messageFor(error: unknown): string {
  if (error instanceof ApiError) return `${error.code}：${error.message}`;
  if (error instanceof Error) return `平台连接失败：${error.message}`;
  return "平台连接失败：未知错误";
}

function runStateFromPayload(payload: Record<string, unknown>): RunState | null {
  const value = payload.state;
  return typeof value === "string" && ["PREFLIGHT", "STARTING", "RUNNING", "STOPPING", "COMPLETED", "FAILED"].includes(value)
    ? (value as RunState)
    : null;
}

function applyRunEvent(run: RunRecord | null, event: RunEvent): RunRecord | null {
  const nextState = event.kind === "RUN_STATE_CHANGED" ? runStateFromPayload(event.payload) : null;
  if (!run || !nextState) return run;
  return {
    ...run,
    state: nextState,
    reason_code: stringOrNull(event.payload.reason_code),
    updated_at: event.wall_time,
    ended_at: isTerminalState(nextState) ? event.wall_time : run.ended_at
  };
}

function stringOrNull(value: unknown): string | null {
  return typeof value === "string" ? value : null;
}

function isTerminalState(state: RunState): boolean {
  return state === "COMPLETED" || state === "FAILED";
}

export const runStore = new RunStore();

export function useRunStore<T>(selector: (state: WorkbenchState) => T): T {
  return useSyncExternalStore(runStore.subscribe, () => selector(runStore.getState()));
}
