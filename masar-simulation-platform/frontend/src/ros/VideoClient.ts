import type { RosGraph } from "./RosConnectionManager";

interface VideoTarget {
  src: string;
  removeAttribute(name: string): void;
}

interface VideoSource {
  graph: RosGraph;
  topic: string;
}

export class VideoClient {
  private readonly selected = new WeakMap<object, string>();

  constructor(private readonly origin = browserOrigin()) {}

  select(target: VideoTarget, source: VideoSource, retryKey?: string | number): string {
    const url = this.streamUrl(source, retryKey);
    const previous = this.selected.get(target);
    if (previous === url) return url;
    if (previous !== undefined) this.clear(target);
    target.src = url;
    this.selected.set(target, url);
    return url;
  }

  snapshot(target: VideoTarget, source: VideoSource, cacheKey?: string | number): string {
    const url = this.snapshotUrl(source, cacheKey);
    if (this.selected.get(target) === url) return url;
    target.src = url;
    this.selected.set(target, url);
    return url;
  }

  clear(target: VideoTarget): void {
    target.src = "";
    target.removeAttribute("src");
    this.selected.delete(target);
  }

  streamUrl(source: VideoSource, retryKey?: string | number): string {
    const topic = source.topic.split("/").map(encodeURIComponent).join("/");
    const retry = retryKey === undefined ? "" : `&retry=${encodeURIComponent(String(retryKey))}`;
    return `${this.origin}/video/${source.graph}/stream?topic=${topic}&type=mjpeg&quality=70${retry}`;
  }

  snapshotUrl(source: VideoSource, cacheKey?: string | number): string {
    const topic = source.topic.split("/").map(encodeURIComponent).join("/");
    const cache = cacheKey === undefined ? "" : `&retry=${encodeURIComponent(String(cacheKey))}`;
    return `${this.origin}/video/${source.graph}/snapshot?topic=${topic}&quality=70${cache}`;
  }
}

function browserOrigin(): string {
  if (typeof window === "undefined") return "http://127.0.0.1";
  return window.location.origin;
}
