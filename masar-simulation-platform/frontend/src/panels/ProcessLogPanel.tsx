import { useEffect, useMemo, useState } from "react";
import { api, type RunEvent } from "../app/api";
import { PanelFrame } from "./PanelFrame";

interface ProcessOption {
  id: string;
  label: string;
}

export function ProcessLogPanel({
  runId,
  events,
  running
}: {
  runId: string | null;
  events: RunEvent[];
  running: boolean;
}) {
  const processes = useMemo(() => processOptions(events), [events]);
  const [selectedProcess, setSelectedProcess] = useState<string | null>(null);
  const processId = processes.some((process) => process.id === selectedProcess)
    ? selectedProcess
    : processes[0]?.id ?? null;
  const [stream, setStream] = useState<"stdout" | "stderr">("stdout");
  const [output, setOutput] = useState("");
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    setOutput("");
    setError(null);
    if (!runId || !processId) return;
    let active = true;
    let cursor = 0;
    let timer: ReturnType<typeof setTimeout> | null = null;
    const poll = async () => {
      try {
        const chunk = await api.logs(runId, processId, stream, cursor);
        if (!active) return;
        cursor = chunk.cursor;
        if (chunk.data) setOutput((current) => boundedLog(current + chunk.data));
        setError(null);
      } catch (reason) {
        if (active) setError(reason instanceof Error ? reason.message : "日志读取失败");
      }
      if (active) timer = setTimeout(poll, 1000);
    };
    void poll();
    return () => {
      active = false;
      if (timer !== null) clearTimeout(timer);
    };
  }, [processId, runId, stream]);

  return (
    <PanelFrame panelId="logs" eyebrow="PROCESS OUTPUT" title="节点日志">
      {processId ? (
        <div className="process-console">
          <div className="process-console__toolbar">
            <label>
              <span>进程</span>
              <select
                aria-label="日志进程"
                value={processId}
                onChange={(event) => setSelectedProcess(event.target.value)}
              >
                {processes.map((process) => (
                  <option value={process.id} key={process.id}>{process.label}</option>
                ))}
              </select>
            </label>
            <div className="process-stream-switch" aria-label="日志流">
              <button
                type="button"
                className={stream === "stdout" ? "is-selected" : ""}
                aria-label="标准输出"
                onClick={() => setStream("stdout")}
              >OUT</button>
              <button
                type="button"
                className={stream === "stderr" ? "is-selected" : ""}
                aria-label="标准错误"
                onClick={() => setStream("stderr")}
              >ERR</button>
            </div>
          </div>
          <pre data-testid="process-log-output">{output || error || "等待进程输出"}</pre>
        </div>
      ) : (
        <div className="process-placeholder">
          <span>$</span>
          <p>{running ? "等待 Run 进程启动" : "启动场景后可查看节点日志"}</p>
        </div>
      )}
    </PanelFrame>
  );
}

function processOptions(events: RunEvent[]): ProcessOption[] {
  const options = new Map<string, ProcessOption>();
  for (const event of events) {
    const id = event.kind === "STEP_STARTED"
      ? event.payload.step_id
      : event.kind === "DISPLAY_STARTED"
        ? event.payload.process_id
        : null;
    if (typeof id !== "string") continue;
    const agent = typeof event.payload.agent === "string" ? event.payload.agent.toUpperCase() : null;
    options.set(id, { id, label: agent ? `${id} · ${agent}` : id });
  }
  return [...options.values()];
}

function boundedLog(value: string): string {
  return value.length > 200_000 ? value.slice(value.length - 200_000) : value;
}
