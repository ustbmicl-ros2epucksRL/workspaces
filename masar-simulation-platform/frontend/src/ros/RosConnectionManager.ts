import { Ros } from "roslib";

export type RosGraph = "ros1" | "ros2";
export type RosConnectionState = "CONNECTING" | "ONLINE" | "ERROR" | "DISCONNECTED";

export interface RosLike {
  connect(url: string): void | Promise<void>;
  close(): void;
  on(event: string, callback: (...args: unknown[]) => void): unknown;
  getTopics?(
    callback: (result: { topics: string[]; types: string[] }) => void,
    failedCallback?: (error: string) => void
  ): void;
}

export interface RosTopicDescription {
  topic: string;
  messageType: string;
}

interface RosConnectionManagerOptions {
  origin?: string;
  createRos?: (graph: RosGraph) => RosLike;
}

type StateListener = (state: RosConnectionState) => void;
const RECONNECT_DELAYS_MS = [1000, 2000, 5000, 10_000] as const;
const TOPIC_DISCOVERY_TIMEOUT_MS = 10_000;

/**
 * Owns the two read-only browser connections used by the workbench.
 *
 * This class intentionally has no publish API. Commands continue to travel
 * through the authenticated control-plane REST API instead of rosbridge.
 */
export class RosConnectionManager {
  private readonly origin: string;
  private readonly createRos: (graph: RosGraph) => RosLike;
  private readonly connections = new Map<RosGraph, RosLike>();
  private readonly states = new Map<RosGraph, RosConnectionState>();
  private readonly listeners = new Map<RosGraph, Set<StateListener>>();
  private readonly reconnectTimers = new Map<RosGraph, ReturnType<typeof setTimeout>>();
  private readonly reconnectAttempts = new Map<RosGraph, number>();
  private readonly intentionallyClosed = new Set<RosGraph>();

  constructor(options: RosConnectionManagerOptions = {}) {
    this.origin = options.origin ?? browserOrigin();
    this.createRos = options.createRos ?? (() => new Ros() as unknown as RosLike);
  }

  get(graph: RosGraph): RosLike {
    const existing = this.connections.get(graph);
    if (existing) return existing;

    const connection = this.createRos(graph);
    this.intentionallyClosed.delete(graph);
    this.connections.set(graph, connection);
    connection.on("connection", () => {
      this.cancelReconnect(graph);
      this.reconnectAttempts.set(graph, 0);
      this.setState(graph, "ONLINE");
    });
    connection.on("error", () => {
      this.setState(graph, "ERROR");
      this.scheduleReconnect(graph);
    });
    connection.on("close", () => {
      this.setState(graph, "DISCONNECTED");
      this.scheduleReconnect(graph);
    });
    this.connectGraph(graph, connection);
    return connection;
  }

  state(graph: RosGraph): RosConnectionState | undefined {
    return this.states.get(graph);
  }

  subscribeState(graph: RosGraph, listener: StateListener): () => void {
    const graphListeners = this.listeners.get(graph) ?? new Set<StateListener>();
    graphListeners.add(listener);
    this.listeners.set(graph, graphListeners);
    return () => graphListeners.delete(listener);
  }

  topics(graph: RosGraph): Promise<RosTopicDescription[]> {
    const connection = this.get(graph);
    if (!connection.getTopics) {
      return Promise.reject(new Error(`${graph} 连接不支持 rosapi topic discovery`));
    }
    return new Promise((resolve, reject) => {
      // A lost service response must release RosDataRuntime's discovery guard
      // so its next poll can discover topics without requiring a page reload.
      const timeout = setTimeout(() => {
        reject(new Error(`${graph} rosapi topic discovery timed out`));
      }, TOPIC_DISCOVERY_TIMEOUT_MS);
      try {
        connection.getTopics!(
          (result) => {
            clearTimeout(timeout);
            const topics = Array.isArray(result.topics) ? result.topics : [];
            const types = Array.isArray(result.types) ? result.types : [];
            resolve(
              topics.flatMap((topic, index) => {
                const messageType = types[index];
                return typeof topic === "string" && typeof messageType === "string"
                  ? [{ topic, messageType }]
                  : [];
              })
            );
          },
          (error) => {
            clearTimeout(timeout);
            reject(new Error(String(error)));
          }
        );
      } catch (error) {
        clearTimeout(timeout);
        reject(error);
      }
    });
  }

  close(graph?: RosGraph): void {
    const graphs: RosGraph[] = graph ? [graph] : ["ros1", "ros2"];
    for (const target of graphs) {
      this.intentionallyClosed.add(target);
      this.cancelReconnect(target);
      this.connections.get(target)?.close();
      this.connections.delete(target);
      this.reconnectAttempts.delete(target);
      this.setState(target, "DISCONNECTED");
    }
  }

  private connectGraph(graph: RosGraph, connection: RosLike): void {
    if (this.intentionallyClosed.has(graph)) return;
    this.setState(graph, "CONNECTING");
    try {
      const result = connection.connect(webSocketUrl(this.origin, graph));
      if (result instanceof Promise) {
        void result.catch(() => {
          this.setState(graph, "ERROR");
          this.scheduleReconnect(graph);
        });
      }
    } catch {
      this.setState(graph, "ERROR");
      this.scheduleReconnect(graph);
    }
  }

  private scheduleReconnect(graph: RosGraph): void {
    if (
      this.intentionallyClosed.has(graph) ||
      this.reconnectTimers.has(graph) ||
      !this.connections.has(graph)
    ) {
      return;
    }
    const attempt = this.reconnectAttempts.get(graph) ?? 0;
    const delay = RECONNECT_DELAYS_MS[Math.min(attempt, RECONNECT_DELAYS_MS.length - 1)];
    this.reconnectAttempts.set(graph, attempt + 1);
    this.reconnectTimers.set(graph, setTimeout(() => {
      this.reconnectTimers.delete(graph);
      const connection = this.connections.get(graph);
      if (connection) this.connectGraph(graph, connection);
    }, delay));
  }

  private cancelReconnect(graph: RosGraph): void {
    const timer = this.reconnectTimers.get(graph);
    if (timer !== undefined) clearTimeout(timer);
    this.reconnectTimers.delete(graph);
  }

  private setState(graph: RosGraph, state: RosConnectionState): void {
    if (this.states.get(graph) === state) return;
    this.states.set(graph, state);
    for (const listener of this.listeners.get(graph) ?? []) listener(state);
  }
}

function browserOrigin(): string {
  if (typeof window === "undefined") return "http://127.0.0.1";
  return window.location.origin;
}

function webSocketUrl(origin: string, graph: RosGraph): string {
  const url = new URL(origin);
  url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
  url.pathname = `/${graph}/`;
  url.search = "";
  url.hash = "";
  return url.toString();
}
