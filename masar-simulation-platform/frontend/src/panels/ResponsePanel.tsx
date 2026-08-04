import { useEffect, useState } from "react";
import type { DataSource } from "../app/api";
import {
  decodeBoolean,
  decodeEmergencyTask,
  decodeEmergencyTasks,
  decodeResponseStatus,
  type EmergencyTaskSnapshot,
  type ResponseStatusSnapshot
} from "../ros/decoders";
import { rosDataRuntime } from "../ros/RosDataRuntime";
import { runScopedSourceKey } from "../ros/sourceIdentity";
import { PanelFrame } from "./PanelFrame";
import { sourceFor, StateBadge } from "./panelUtils";

const kindLabels = { victim: "受害者", fire: "火源", box: "待推送障碍", home: "返航" } as const;
const taskStatusLabels = {
  new: "待分配", dispatched: "已分配", en_route: "前往目标", acting: "执行中", done: "已完成", failed: "失败"
} as const;
const stateLabels = { idle: "空闲", navigating: "导航中", acting: "任务执行", done: "已完成", failed: "失败" } as const;
const planningLabels: Record<string, string> = {
  STOPPED: "停止", WAITING_FOR_GOAL: "等待目标", TRACKING: "路径跟踪", ARRIVAL_HOLD: "到达保持",
  DONE: "完成", FAILED: "失败", ERROR: "错误"
};

interface ResponsePanelProps {
  runId: string | null;
  sources: DataSource[];
  running: boolean;
}

export function ResponsePanel({ runId, sources, running }: ResponsePanelProps) {
  const statusSource = sourceFor(sources, "response_status");
  const tasksSource = sourceFor(sources, "response_tasks");
  const activeSource = sourceFor(sources, "response_active_task");
  const motionSource = sourceFor(sources, "response_motion_enabled");
  const [status, setStatus] = useState<ResponseStatusSnapshot | null>(null);
  const [tasks, setTasks] = useState<EmergencyTaskSnapshot[] | null>(null);
  const [activeTask, setActiveTask] = useState<EmergencyTaskSnapshot | null>(null);
  const [motionEnabled, setMotionEnabled] = useState<boolean | null>(null);
  const keys = [statusSource, tasksSource, activeSource, motionSource]
    .map((source) => runScopedSourceKey(runId, source)).join("|");

  useEffect(() => {
    setStatus(null);
    setTasks(null);
    setActiveTask(null);
    setMotionEnabled(null);
  }, [runId]);

  useEffect(() => {
    const releases = [
      subscribe(statusSource, (message) => {
        const decoded = decodeResponseStatus(message);
        if (decoded) setStatus(decoded);
      }),
      subscribe(tasksSource, (message) => {
        const decoded = decodeEmergencyTasks(message);
        if (decoded) setTasks(decoded);
      }),
      subscribe(activeSource, (message) => {
        const decoded = decodeEmergencyTask(message);
        if (decoded) setActiveTask(decoded);
      }),
      subscribe(motionSource, (message) => {
        const decoded = decodeBoolean(message);
        if (decoded !== null) setMotionEnabled(decoded);
      })
    ];
    return () => releases.forEach((release) => release?.());
  // Run and exact source identities control the shared subscriptions.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [keys]);

  const effectiveActiveTask = activeTask ?? taskFromStatus(status, tasks);
  const effectiveMotion = motionEnabled ?? status?.motionEnabled ?? null;
  const availability = statusSource?.availability ?? "WAITING";

  return (
    <PanelFrame
      panelId="response"
      eyebrow="RESPONSE GROUP"
      title="响应组状态"
      status={<StateBadge state={availability} />}
    >
      <div className="response-grid">
        <Metric testId="response-state" label="当前状态" value={status ? `${labelFor(stateLabels, status.state)} · ${labelFor(planningLabels, status.planningState)}` : running ? "等待状态" : "等待运行"} />
        <Metric testId="response-motion" label="运动使能" value={effectiveMotion === null ? (running ? "等待门控" : "等待运行") : effectiveMotion ? "已启用" : "已暂停"} />
        <Metric testId="response-active-task" label="当前任务" value={taskLabel(effectiveActiveTask, running)} />
        <Metric testId="response-queue" label="任务队列" value={queueLabel(tasks, running)} />
        <Metric testId="response-formation-error" label="编队误差" value={status ? `RMS ${status.rmsSlotError.toFixed(3)} m · 最大 ${status.maxSlotError.toFixed(3)} m` : running ? "等待状态" : "等待运行"} />
        <Metric testId="response-clearance" label="最小净空" value={status ? `${status.minimumClearance.toFixed(3)} m` : running ? "等待状态" : "等待运行"} />
      </div>
    </PanelFrame>
  );
}

function Metric({ testId, label, value }: { testId: string; label: string; value: string }) {
  return <div><dt>{label}</dt><dd data-testid={testId}>{value}</dd></div>;
}

function subscribe(source: DataSource | undefined, callback: (message: unknown) => void): (() => void) | undefined {
  return source?.topic ? rosDataRuntime.subscribe(source, callback) : undefined;
}

function labelFor<T extends string>(labels: Record<string, string>, value: T): string {
  return labels[value] ?? value;
}

function taskFromStatus(status: ResponseStatusSnapshot | null, tasks: EmergencyTaskSnapshot[] | null) {
  if (!status || status.activeTaskId === null) return null;
  return tasks?.find((task) => task.taskId === status.activeTaskId) ?? null;
}

function taskLabel(task: EmergencyTaskSnapshot | null, running: boolean): string {
  if (!task) return running ? "暂无活动任务" : "等待运行";
  return `#${task.taskId} · ${kindLabels[task.kind]} · ${taskStatusLabels[task.status]}`;
}

function queueLabel(tasks: EmergencyTaskSnapshot[] | null, running: boolean): string {
  if (!tasks) return running ? "等待任务队列" : "等待运行";
  const active = tasks.filter((task) => task.status === "acting" || task.status === "en_route").length;
  const pending = tasks.filter((task) => task.status === "new" || task.status === "dispatched").length;
  return `${tasks.length} 项 · ${active} 执行中 · ${pending} 待分配`;
}
