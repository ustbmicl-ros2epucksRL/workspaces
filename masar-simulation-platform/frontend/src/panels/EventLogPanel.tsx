import type { RunEvent } from "../app/api";
import { PanelFrame } from "./PanelFrame";

export function EventLogPanel({ events }: { events: RunEvent[] }) {
  return (
    <PanelFrame panelId="events" eyebrow="EVENT CURSOR" title="运行事件">
      <div className="console-list">
        {events.length ? (
          events.slice(-20).map((event) => (
            <div key={event.sequence}>
              <span>{String(event.sequence).padStart(4, "0")}</span>
              <strong>{event.kind}</strong>
              <em>{event.severity}</em>
            </div>
          ))
        ) : (
          <p>尚无运行事件</p>
        )}
      </div>
    </PanelFrame>
  );
}
