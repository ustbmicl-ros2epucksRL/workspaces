import { Topic, type Ros } from "roslib";
import type { RosGraph, RosLike } from "./RosConnectionManager";
import { RosConnectionManager } from "./RosConnectionManager";
import { TopicHealth } from "./TopicHealth";

export interface TopicContract {
  graph: RosGraph;
  topic: string;
  messageType: string;
  staleAfterMs?: number;
}

export interface TopicLike {
  subscribe(callback: (message: unknown) => void): void;
  unsubscribe(callback?: (message: unknown) => void): void;
}

interface TopicRegistryOptions {
  createTopic?: (
    ros: RosLike,
    options: {
      name: string;
      messageType: string;
      compression: string;
      throttle_rate: number;
      queue_length: number;
      reconnect_on_close: boolean;
    }
  ) => TopicLike;
}

interface RegistryEntry {
  topic: TopicLike;
  health: TopicHealth;
  callbacks: Set<(message: unknown) => void>;
  bridgeCallback: (message: unknown) => void;
  lastDeliveredAt: number | null;
  pendingMessage: unknown;
  hasPendingMessage: boolean;
  timer: ReturnType<typeof setTimeout> | null;
}

export class TopicTypeMismatch extends Error {
  constructor(contract: TopicContract) {
    super(`话题未声明或类型不匹配：${contract.graph}:${contract.topic} (${contract.messageType})`);
    this.name = "TopicTypeMismatch";
  }
}

export class TopicRegistry {
  private readonly contracts = new Map<string, Required<TopicContract>>();
  private readonly entries = new Map<string, RegistryEntry>();
  private readonly createTopic: NonNullable<TopicRegistryOptions["createTopic"]>;

  constructor(
    private readonly connections: RosConnectionManager,
    allowedContracts: TopicContract[],
    options: TopicRegistryOptions = {}
  ) {
    for (const contract of allowedContracts) {
      this.contracts.set(keyFor(contract), {
        ...contract,
        staleAfterMs: contract.staleAfterMs ?? 2000
      });
    }
    this.createTopic = options.createTopic ?? defaultTopicFactory;
  }

  subscribe(contract: TopicContract, callback: (message: unknown) => void): () => void {
    const key = keyFor(contract);
    const declared = this.contracts.get(key);
    if (!declared) throw new TopicTypeMismatch(contract);

    let entry = this.entries.get(key);
    if (!entry) {
      entry = this.createEntry(declared);
      this.entries.set(key, entry);
      entry.topic.subscribe(entry.bridgeCallback);
    }
    entry.callbacks.add(callback);

    let active = true;
    return () => {
      if (!active) return;
      active = false;
      const current = this.entries.get(key);
      if (!current) return;
      current.callbacks.delete(callback);
      if (current.callbacks.size > 0) return;
      current.topic.unsubscribe(current.bridgeCallback);
      if (current.timer !== null) clearTimeout(current.timer);
      this.entries.delete(key);
    };
  }

  health(contract: TopicContract): TopicHealth {
    const key = keyFor(contract);
    const declared = this.contracts.get(key);
    if (!declared) throw new TopicTypeMismatch(contract);
    return this.entries.get(key)?.health ?? new TopicHealth(declared.staleAfterMs);
  }

  close(): void {
    for (const entry of this.entries.values()) {
      entry.topic.unsubscribe(entry.bridgeCallback);
      if (entry.timer !== null) clearTimeout(entry.timer);
    }
    this.entries.clear();
  }

  private createEntry(contract: Required<TopicContract>): RegistryEntry {
    const health = new TopicHealth(contract.staleAfterMs);
    const entry: RegistryEntry = {
      topic: this.createTopic(this.connections.get(contract.graph), {
        name: contract.topic,
        messageType: contract.messageType,
        compression: "none",
        throttle_rate: 100,
        queue_length: 1,
        reconnect_on_close: true
      }),
      health,
      callbacks: new Set(),
      bridgeCallback: () => undefined,
      lastDeliveredAt: null,
      pendingMessage: undefined,
      hasPendingMessage: false,
      timer: null
    };
    entry.bridgeCallback = (message) => this.acceptMessage(entry!, message);
    return entry;
  }

  private acceptMessage(entry: RegistryEntry, message: unknown): void {
    entry.health.observeMessage();
    const now = Date.now();
    if (entry.lastDeliveredAt === null || now - entry.lastDeliveredAt >= 100) {
      this.deliver(entry, message, now);
      return;
    }

    entry.pendingMessage = message;
    entry.hasPendingMessage = true;
    if (entry.timer !== null) return;
    const delay = Math.max(0, 100 - (now - entry.lastDeliveredAt));
    entry.timer = setTimeout(() => {
      entry.timer = null;
      if (!entry.hasPendingMessage) return;
      const pending = entry.pendingMessage;
      entry.pendingMessage = undefined;
      entry.hasPendingMessage = false;
      this.deliver(entry, pending, Date.now());
    }, delay);
  }

  private deliver(entry: RegistryEntry, message: unknown, at: number): void {
    entry.lastDeliveredAt = at;
    for (const callback of entry.callbacks) callback(message);
  }
}

function keyFor(contract: TopicContract): string {
  return `${contract.graph}\u0000${contract.topic}\u0000${contract.messageType}`;
}

function defaultTopicFactory(ros: RosLike, options: Parameters<NonNullable<TopicRegistryOptions["createTopic"]>>[1]): TopicLike {
  return new Topic({ ros: ros as Ros, ...options });
}
