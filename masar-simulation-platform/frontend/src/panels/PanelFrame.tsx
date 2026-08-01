import type { PropsWithChildren, ReactNode } from "react";
import { runStore, useRunStore } from "../app/runStore";

interface PanelFrameProps extends PropsWithChildren {
  panelId: string;
  title: string;
  eyebrow: string;
  status?: ReactNode;
  className?: string;
}

export function PanelFrame({
  panelId,
  title,
  eyebrow,
  status,
  className = "",
  children
}: PanelFrameProps) {
  const panel = useRunStore((state) => state.panels[panelId]);
  const resolved = panel ?? runStore.panelState(panelId);
  return (
    <section
      className={`panel panel--${panelId} ${resolved.maximized ? "panel--maximized" : ""} ${className}`}
      data-testid={`panel-${panelId}`}
      data-size={resolved.size}
      data-maximized={String(resolved.maximized)}
    >
      <header className="panel__header">
        <div className="panel__identity">
          <span className="panel__eyebrow">{eyebrow}</span>
          <h2>{title}</h2>
        </div>
        <div className="panel__header-end">
          {status ? <div className="panel__status">{status}</div> : null}
          <div className="panel__actions" aria-label={`${title}面板操作`}>
            <button
              type="button"
              aria-label="调整尺寸"
              title="调整尺寸"
              onClick={() => runStore.togglePanelSize(panelId)}
            >
              ↔
            </button>
            {resolved.maximized ? (
              <button
                type="button"
                aria-label="恢复"
                title="恢复"
                onClick={() => runStore.restorePanel(panelId)}
              >
                ◱
              </button>
            ) : (
              <button
                type="button"
                aria-label="最大化"
                title="最大化"
                onClick={() => runStore.maximizePanel(panelId)}
              >
                □
              </button>
            )}
            <button
              type="button"
              aria-label="弹出"
              title="弹出"
              onClick={() => runStore.popoutPanel(panelId)}
            >
              ↗
            </button>
          </div>
        </div>
      </header>
      <div className="panel__body">{children}</div>
    </section>
  );
}
