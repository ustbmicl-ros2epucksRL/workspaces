export type WorkspaceId = "group_behavior" | "exploration" | "rescue";
export type RunState =
  | "CREATED"
  | "VALIDATING"
  | "STARTING"
  | "RUNNING"
  | "STOPPING"
  | "COMPLETED"
  | "FAILED";
export type DataAvailability =
  | "UNAVAILABLE"
  | "WAITING"
  | "ONLINE"
  | "STALE"
  | "TYPE_MISMATCH"
  | "DISCONNECTED"
  | "UNKNOWN";

export interface WorkspaceCapability {
  id: WorkspaceId;
  label: string;
  scenario_modules: string[];
}

export interface Capabilities {
  schema_version: number;
  single_active_run: boolean;
  workspaces: WorkspaceCapability[];
  simulators: string[];
  scenario_count: number;
  event_transport: string;
}

export interface ScenarioSummary {
  scenario_id: string;
  workspace: WorkspaceId;
  module: string;
  simulator: "gazebo" | "webots";
  display_profile: string;
  panels: string[];
  commands: string[];
}

export interface ParameterDefinition {
  type: "string" | "integer" | "number" | "boolean";
  default: string | number | boolean;
  enum?: Array<string | number>;
  minimum?: number;
  maximum?: number;
  description?: string;
}

export interface SimulationDisplay {
  transport: "webots" | "novnc";
  path: string;
  display: string | null;
  required: boolean;
}

export interface ScenarioDetail extends ScenarioSummary {
  runtime: { ros1_master_uri: string; ros2_domain_id: number };
  parameters: Record<string, ParameterDefinition>;
  data_profile: string;
  artifact_profile: string;
  display_plan: {
    simulation: SimulationDisplay;
    rviz: null | {
      agent: string;
      process_id: string;
      command_ref: string;
      argv: string[];
      environment: Record<string, string>;
      required: boolean;
    };
  };
}

export interface RunRecord {
  run_id: string;
  scenario_id: string;
  state: RunState;
  reason_code: string | null;
  created_at: string;
  updated_at: string;
  ended_at: string | null;
  parameters: Record<string, unknown>;
}

export interface DataSource {
  topic_pattern: string;
  message_types?: string[];
  graph?: "ros1" | "ros2";
  capability: string;
  required: boolean;
  availability: DataAvailability;
  last_age_seconds?: number;
}

export interface RunEvent {
  sequence: number;
  kind: string;
  severity: string;
  timestamp: string;
  payload: Record<string, unknown>;
}

export class ApiError extends Error {
  constructor(
    readonly status: number,
    readonly code: string,
    message: string
  ) {
    super(message);
  }
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(`/api/v1${path}`, {
    ...init,
    headers: {
      Accept: "application/json",
      ...(init?.body ? { "Content-Type": "application/json" } : {}),
      ...init?.headers
    }
  });
  const payload = (await response.json()) as T & {
    error?: { code?: string; message?: string };
  };
  if (!response.ok) {
    throw new ApiError(
      response.status,
      payload.error?.code ?? "REQUEST_FAILED",
      payload.error?.message ?? `请求失败 (${response.status})`
    );
  }
  return payload;
}

export const api = {
  async capabilities(): Promise<Capabilities> {
    return request<Capabilities>("/capabilities");
  },
  async scenarios(): Promise<ScenarioSummary[]> {
    return (await request<{ scenarios: ScenarioSummary[] }>("/scenarios")).scenarios;
  },
  scenario(scenarioId: string): Promise<ScenarioDetail> {
    return request<ScenarioDetail>(`/scenarios/${encodeURIComponent(scenarioId)}`);
  },
  async currentRun(): Promise<RunRecord | null> {
    return (await request<{ run: RunRecord | null }>("/runs/current")).run;
  },
  run(runId: string): Promise<RunRecord> {
    return request<RunRecord>(`/runs/${encodeURIComponent(runId)}`);
  },
  async createRun(
    scenarioId: string,
    parameters: Record<string, unknown>
  ): Promise<{ run_id: string; state: RunState }> {
    return request("/runs", {
      method: "POST",
      body: JSON.stringify({
        scenario_id: scenarioId,
        parameters,
        recording: { enabled: false, profile: "default" }
      })
    });
  },
  stopRun(runId: string): Promise<RunRecord> {
    return request(`/runs/${encodeURIComponent(runId)}/stop`, { method: "POST" });
  },
  async dataSources(runId: string): Promise<DataSource[]> {
    return (
      await request<{ data_sources: DataSource[] }>(
        `/runs/${encodeURIComponent(runId)}/data-sources`
      )
    ).data_sources;
  },
  async events(runId: string, after: number): Promise<RunEvent[]> {
    return (
      await request<{ events: RunEvent[] }>(
        `/runs/${encodeURIComponent(runId)}/events?after=${after}`
      )
    ).events;
  },
  command(runId: string, command: string): Promise<Record<string, unknown>> {
    const routes: Record<string, string> = {
      "mission.start": "mission-commands/start",
      "mission.stop": "mission-commands/stop",
      "mission.home": "mission-commands/home",
      "response.return_home": "response-commands/return-home"
    };
    const route = routes[command];
    if (!route) throw new ApiError(400, "COMMAND_NOT_SUPPORTED", "当前场景不支持该命令");
    return request(`/runs/${encodeURIComponent(runId)}/${route}`, { method: "POST" });
  }
};

export function eventSocketUrl(runId: string, after: number): string {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${window.location.host}/api/v1/events?run_id=${encodeURIComponent(
    runId
  )}&after=${after}`;
}
