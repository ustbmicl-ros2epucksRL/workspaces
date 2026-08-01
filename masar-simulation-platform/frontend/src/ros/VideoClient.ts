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

  select(target: VideoTarget, source: VideoSource): string {
    const url = `${this.origin}/video/${source.graph}/stream?topic=${encodeURIComponent(
      source.topic
    )}&type=mjpeg&quality=70`;
    const previous = this.selected.get(target);
    if (previous === url) return url;
    if (previous !== undefined) this.clear(target);
    target.src = url;
    this.selected.set(target, url);
    return url;
  }

  clear(target: VideoTarget): void {
    target.src = "";
    target.removeAttribute("src");
    this.selected.delete(target);
  }
}

function browserOrigin(): string {
  if (typeof window === "undefined") return "http://127.0.0.1";
  return window.location.origin;
}
