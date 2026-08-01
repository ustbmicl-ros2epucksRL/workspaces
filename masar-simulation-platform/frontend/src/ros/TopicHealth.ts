import type { DataAvailability } from "../app/api";

export interface TopicHealthSnapshot {
  availability: DataAvailability;
  lastAgeMs: number | null;
  observedHz: number | null;
}

export class TopicHealth {
  private lastMessageAt: number | null = null;
  private readonly observations: number[] = [];
  private terminalState: Extract<DataAvailability, "TYPE_MISMATCH" | "DISCONNECTED"> | null = null;

  constructor(
    private readonly staleAfterMs: number,
    private readonly now: () => number = Date.now
  ) {}

  observeMessage(): void {
    const observedAt = this.now();
    this.lastMessageAt = observedAt;
    this.observations.push(observedAt);
    const windowStart = observedAt - 5000;
    while (this.observations.length > 2 && this.observations[0] < windowStart) {
      this.observations.shift();
    }
    this.terminalState = null;
  }

  markTypeMismatch(): void {
    this.terminalState = "TYPE_MISMATCH";
  }

  markDisconnected(): void {
    this.terminalState = "DISCONNECTED";
  }

  snapshot(): TopicHealthSnapshot {
    if (this.terminalState) {
      return {
        availability: this.terminalState,
        lastAgeMs: this.age(),
        observedHz: this.frequency()
      };
    }
    if (this.lastMessageAt === null) {
      return { availability: "WAITING", lastAgeMs: null, observedHz: null };
    }
    const lastAgeMs = this.age() ?? 0;
    return {
      availability: lastAgeMs > this.staleAfterMs ? "STALE" : "ONLINE",
      lastAgeMs,
      observedHz: this.frequency()
    };
  }

  private age(): number | null {
    return this.lastMessageAt === null ? null : Math.max(0, this.now() - this.lastMessageAt);
  }

  private frequency(): number | null {
    if (this.observations.length < 2) return null;
    const elapsedMs = this.observations.at(-1)! - this.observations[0];
    return elapsedMs <= 0 ? null : ((this.observations.length - 1) * 1000) / elapsedMs;
  }
}
