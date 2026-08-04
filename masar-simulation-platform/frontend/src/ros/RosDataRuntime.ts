import { useEffect, useSyncExternalStore } from "react";
import type { DataAvailability, DataSource } from "../app/api";
import {
  RosConnectionManager,
  type RosConnectionState,
  type RosGraph,
  type RosTopicDescription
} from "./RosConnectionManager";
import { TopicRegistry, type TopicContract } from "./TopicRegistry";

type Listener = () => void;

interface TopicRegistryAdapter {
  subscribe(contract: TopicContract, callback: (message: unknown) => void): () => void;
  health(contract: TopicContract): { snapshot(): {
    availability: DataAvailability;
    lastAgeMs: number | null;
    observedHz: number | null;
  } };
  close(): void;
}

interface RosDataRuntimeOptions {
  createRegistry?: (
    connections: RosConnectionManager,
    contracts: TopicContract[]
  ) => TopicRegistryAdapter;
}

interface TopicSubscription {
  contract: TopicContract;
  callback: (message: unknown) => void;
  releaseRegistry: (() => void) | null;
}

/**
 * Resolves profile patterns against rosapi and exposes only exact, type-checked
 * topics to panels. It is the sole owner of TopicRegistry in the browser.
 */
export class RosDataRuntime {
  private sources: DataSource[] = [];
  private profileSources: DataSource[] = [];
  private readonly discoveries = new Map<RosGraph, RosTopicDescription[]>();
  private expectedGraphs = new Set<RosGraph>();
  private readonly listeners = new Set<Listener>();
  private connectionReleases: Array<() => void> = [];
  private registry: TopicRegistryAdapter | null = null;
  private registryContractsKey = "";
  private registryContractKeys = new Set<string>();
  private readonly topicSubscriptions = new Map<number, TopicSubscription>();
  private nextTopicSubscriptionId = 1;
  private discoveryTimer: ReturnType<typeof setInterval> | null = null;
  private readonly discoveringGraphs = new Set<RosGraph>();
  private healthTimer: ReturnType<typeof setInterval> | null = null;
  private activeTopicSubscriptions = 0;
  private configurationKey: string | null = null;
  private generation = 0;
  private readonly createRegistry: NonNullable<RosDataRuntimeOptions["createRegistry"]>;

  constructor(
    private readonly connections = new RosConnectionManager(),
    options: RosDataRuntimeOptions = {}
  ) {
    this.createRegistry = options.createRegistry ?? ((manager, contracts) => new TopicRegistry(manager, contracts));
  }

  subscribeStore = (listener: Listener): (() => void) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };

  getSnapshot = (): DataSource[] => this.sources;

  configure(runId: string | null, sources: DataSource[]): void {
    const configurationKey = runId ? `${runId}:${fingerprint(sources)}` : null;
    if (configurationKey === this.configurationKey) return;
    this.configurationKey = configurationKey;
    this.generation += 1;
    const generation = this.generation;
    this.releaseConfiguration();
    this.profileSources = sources.map((source) => ({ ...source }));
    this.sources = this.profileSources;
    this.emit();
    if (!runId) return;

    const graphs = [...new Set(
      this.profileSources
        .filter(hasTopicContract)
        .map((source) => source.graph as RosGraph)
    )];
    this.expectedGraphs = new Set(graphs);
    for (const graph of graphs) {
      this.connectionReleases.push(
        this.connections.subscribeState(graph, (state) => {
          if (generation !== this.generation) return;
          this.handleConnectionState(graph, state, generation);
        })
      );
      this.connections.get(graph);
      const state = this.connections.state(graph);
      if (state === "ONLINE") void this.discover(graph, generation);
      else if (state === "ERROR" || state === "DISCONNECTED") {
        this.markGraph(graph, "DISCONNECTED");
      }
    }
    this.discoveryTimer = setInterval(() => {
      if (generation !== this.generation) return;
      for (const graph of this.expectedGraphs) {
        if (this.connections.state(graph) === "ONLINE") void this.discover(graph, generation);
      }
    }, 2000);
  }

  subscribe(source: DataSource, callback: (message: unknown) => void): () => void {
    const contract = contractFor(source);
    if (!contract || source.availability === "TYPE_MISMATCH") {
      return () => undefined;
    }
    const subscriptionId = this.nextTopicSubscriptionId;
    this.nextTopicSubscriptionId += 1;
    const subscription: TopicSubscription = {
      contract,
      callback,
      releaseRegistry: null
    };
    this.topicSubscriptions.set(subscriptionId, subscription);
    this.attachTopicSubscription(subscription);
    this.activeTopicSubscriptions = this.topicSubscriptions.size;
    this.ensureHealthTimer();
    let active = true;
    return () => {
      if (!active) return;
      active = false;
      const current = this.topicSubscriptions.get(subscriptionId);
      current?.releaseRegistry?.();
      this.topicSubscriptions.delete(subscriptionId);
      this.activeTopicSubscriptions = this.topicSubscriptions.size;
      if (this.activeTopicSubscriptions === 0) this.stopHealthTimer();
      this.refreshHealth();
    };
  }

  private handleConnectionState(
    graph: RosGraph,
    state: RosConnectionState,
    generation: number
  ): void {
    if (state === "ONLINE") {
      void this.discover(graph, generation);
      return;
    }
    if (state === "ERROR" || state === "DISCONNECTED") {
      this.markGraph(graph, "DISCONNECTED");
    }
  }

  private async discover(graph: RosGraph, generation: number): Promise<void> {
    if (this.discoveringGraphs.has(graph)) return;
    this.discoveringGraphs.add(graph);
    try {
      const topics = await this.connections.topics(graph);
      if (generation !== this.generation) return;
      this.discoveries.set(graph, topics);
      if ([...this.expectedGraphs].every((target) => this.discoveries.has(target))) {
        this.rebuildResolvedSources();
      }
    } catch {
      if (generation === this.generation) {
        this.discoveries.set(graph, []);
        if ([...this.expectedGraphs].every((target) => this.discoveries.has(target))) {
          this.rebuildResolvedSources();
        }
        this.markGraph(graph, "DISCONNECTED");
      }
    } finally {
      this.discoveringGraphs.delete(graph);
    }
  }

  private rebuildResolvedSources(): void {
    const resolved = this.profileSources.flatMap((source) => {
      if (!hasTopicContract(source)) return [{ ...source }];
      const discovered = this.discoveries.get(source.graph);
      if (!discovered) return [{ ...source, availability: "WAITING" as const }];
      return resolveSource(source, discovered);
    });
    const contracts = resolved.flatMap((source) => {
      const contract = contractFor(source);
      return contract && source.availability !== "TYPE_MISMATCH" ? [contract] : [];
    });
    const contractsKey = contractFingerprint(contracts);
    if (!this.registry || contractsKey !== this.registryContractsKey) {
      for (const subscription of this.topicSubscriptions.values()) {
        subscription.releaseRegistry?.();
        subscription.releaseRegistry = null;
      }
      this.registry?.close();
      this.registry = this.createRegistry(this.connections, contracts);
      this.registryContractsKey = contractsKey;
      this.registryContractKeys = new Set(contracts.map(topicContractKey));
    }
    for (const subscription of this.topicSubscriptions.values()) {
      this.attachTopicSubscription(subscription);
    }
    this.sources = resolved;
    this.refreshHealth();
    this.emit();
  }

  private attachTopicSubscription(subscription: TopicSubscription): void {
    if (
      subscription.releaseRegistry ||
      !this.registry ||
      !this.registryContractKeys.has(topicContractKey(subscription.contract))
    ) {
      return;
    }
    subscription.releaseRegistry = this.registry.subscribe(subscription.contract, (message) => {
      subscription.callback(message);
      this.refreshHealth();
    });
  }

  private refreshHealth(): void {
    if (!this.registry) return;
    let changed = false;
    const next = this.sources.map((source) => {
      const contract = contractFor(source);
      if (!contract || source.availability === "TYPE_MISMATCH") return source;
      const snapshot = this.registry!.health(contract).snapshot();
      const lastMessageAgeMs = snapshot.lastAgeMs ?? undefined;
      const observedHz = snapshot.observedHz ?? undefined;
      if (
        source.availability === snapshot.availability &&
        source.last_message_age_ms === lastMessageAgeMs &&
        source.observed_hz === observedHz
      ) {
        return source;
      }
      changed = true;
      return {
        ...source,
        availability: snapshot.availability,
        last_message_age_ms: lastMessageAgeMs,
        last_age_seconds: snapshot.lastAgeMs === null ? undefined : snapshot.lastAgeMs / 1000,
        observed_hz: observedHz
      };
    });
    if (changed) {
      this.sources = next;
      this.emit();
    }
  }

  private ensureHealthTimer(): void {
    if (this.healthTimer !== null) return;
    this.healthTimer = setInterval(() => this.refreshHealth(), 500);
  }

  private stopHealthTimer(): void {
    if (this.healthTimer !== null) clearInterval(this.healthTimer);
    this.healthTimer = null;
  }

  private markGraph(graph: RosGraph, availability: DataAvailability): void {
    let changed = false;
    const next = this.sources.map((source) => {
      if (source.graph !== graph || source.availability === availability) return source;
      changed = true;
      return { ...source, availability };
    });
    if (changed) {
      this.sources = next;
      this.emit();
    }
  }

  private releaseConfiguration(): void {
    for (const release of this.connectionReleases) release();
    this.connectionReleases = [];
    this.discoveries.clear();
    this.discoveringGraphs.clear();
    this.expectedGraphs.clear();
    for (const subscription of this.topicSubscriptions.values()) {
      subscription.releaseRegistry?.();
      subscription.releaseRegistry = null;
    }
    this.registry?.close();
    this.registry = null;
    this.registryContractsKey = "";
    this.registryContractKeys.clear();
    this.activeTopicSubscriptions = this.topicSubscriptions.size;
    if (this.discoveryTimer !== null) clearInterval(this.discoveryTimer);
    this.discoveryTimer = null;
    if (this.activeTopicSubscriptions === 0) this.stopHealthTimer();
  }

  private emit(): void {
    for (const listener of this.listeners) listener();
  }
}

function hasTopicContract(source: DataSource): source is DataSource & {
  graph: RosGraph;
  message_types: string[];
} {
  return Boolean(source.graph && source.message_types?.length);
}

function resolveSource(source: DataSource & { graph: RosGraph; message_types: string[] }, topics: RosTopicDescription[]): DataSource[] {
  const topicPattern = compileTopicPattern(source.topic_pattern);
  const nameMatches = topics.filter((candidate) => topicPattern.test(candidate.topic));
  const accepted = nameMatches.filter((candidate) => source.message_types.includes(candidate.messageType));
  if (accepted.length > 0) {
    return accepted.map((candidate) => ({
      ...source,
      topic: candidate.topic,
      message_type: candidate.messageType,
      availability: "WAITING"
    }));
  }
  if (nameMatches.length > 0) {
    return [{
      ...source,
      topic: nameMatches[0].topic,
      message_type: nameMatches[0].messageType,
      availability: "TYPE_MISMATCH"
    }];
  }
  return [{ ...source, availability: "WAITING" }];
}

function compileTopicPattern(pattern: string): RegExp {
  const escaped = pattern.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
  return new RegExp(
    `^${escaped
      .replace("<robot>", "[^/]+")
      .replace("<agent>", "[^/]+")
      .replace("tb_i", "tb_[0-9]+")}$`
  );
}

function contractFor(source: DataSource): TopicContract | null {
  if (!source.graph || !source.topic || !source.message_type) return null;
  return {
    graph: source.graph,
    topic: source.topic,
    messageType: source.message_type,
    staleAfterMs: staleThreshold(source.capability)
  };
}

function staleThreshold(capability: string): number {
  if (capability === "map") return 10_000;
  if (capability === "camera") return 3_000;
  if ([
    "information_map",
    "frontiers",
    "task_allocation",
    "task_allocation_image",
    "exploration_progress",
    "agent_status",
    "emergency_objects",
    "emergency_events"
  ].includes(capability)) return 60_000;
  return 2_000;
}

function fingerprint(sources: DataSource[]): string {
  return sources
    .map((source) => [source.graph, source.topic_pattern, source.message_types?.join(","), source.capability].join("|"))
    .join(";");
}

function contractFingerprint(contracts: TopicContract[]): string {
  return contracts
    .map((contract) => [contract.graph, contract.topic, contract.messageType, contract.staleAfterMs].join("|"))
    .sort()
    .join(";");
}

function topicContractKey(contract: TopicContract): string {
  return [contract.graph, contract.topic, contract.messageType, contract.staleAfterMs].join("|");
}

export const rosDataRuntime = new RosDataRuntime();

export function useRosDataSources(runId: string | null, profileSources: DataSource[]): DataSource[] {
  const sources = useSyncExternalStore(rosDataRuntime.subscribeStore, rosDataRuntime.getSnapshot);
  useEffect(() => {
    rosDataRuntime.configure(runId, profileSources);
  }, [runId, profileSources]);
  return sources;
}
