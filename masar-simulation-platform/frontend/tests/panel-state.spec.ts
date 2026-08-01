import { expect, test } from "@playwright/test";
import { installMockApi } from "./mockApi";

test("panel resize, maximize, restore and popout reuse one run", async ({ page, context }) => {
  await installMockApi(page, { activeRun: true });
  let createRequests = 0;
  page.on("request", (request) => {
    if (request.method() === "POST" && request.url().endsWith("/api/v1/runs")) createRequests += 1;
  });
  await page.goto("/?workspace=exploration");
  await expect(page.getByTestId("run-id")).toHaveText("run-layout-test");

  const panel = page.getByTestId("panel-simulation");
  await panel.getByRole("button", { name: "调整尺寸" }).click();
  await expect(panel).toHaveAttribute("data-size", "wide");
  await panel.getByRole("button", { name: "最大化" }).click();
  await expect(panel).toHaveAttribute("data-maximized", "true");
  await panel.getByRole("button", { name: "恢复" }).click();
  await expect(panel).toHaveAttribute("data-maximized", "false");

  const popupPromise = context.waitForEvent("page");
  await panel.getByRole("button", { name: "弹出" }).click();
  const popup = await popupPromise;
  await popup.waitForLoadState("domcontentloaded");
  expect(new URL(popup.url()).searchParams.get("run_id")).toBe("run-layout-test");
  await expect(popup.getByTestId("workbench")).toBeVisible();
  expect(createRequests).toBe(0);
  await popup.close();
});

test("missing and unhealthy data is explicit and never fabricated", async ({ page }) => {
  await installMockApi(page, { activeRun: true });
  await page.goto("/?workspace=exploration");

  await expect(page.getByTestId("sensor-camera-state")).toHaveText("未提供");
  const health = page.getByTestId("panel-health");
  await expect(health.getByTestId("topic-state-ONLINE")).toBeVisible();
  await expect(health.getByTestId("topic-state-STALE")).toBeVisible();
  await expect(health.getByTestId("topic-state-TYPE_MISMATCH")).toBeVisible();
  await expect(health.getByTestId("topic-state-UNAVAILABLE")).toBeVisible();
  await expect(health.getByTestId("topic-state-WAITING")).toBeVisible();
  await expect(health.getByTestId("topic-state-DISCONNECTED")).toBeVisible();
  await expect(page.getByTestId("panel-sensors")).not.toContainText("0.00");
});

test("starting a run sends the frozen recording contract", async ({ page }) => {
  await installMockApi(page);
  await page.goto("/?workspace=exploration");

  const requestPromise = page.waitForRequest(
    (request) => request.method() === "POST" && request.url().endsWith("/api/v1/runs")
  );
  await page.getByRole("button", { name: "启动环境" }).click();
  const request = await requestPromise;
  const body = request.postDataJSON() as Record<string, unknown>;

  expect(body).toMatchObject({
    scenario_id: "exploration.gazebo.tepper",
    parameters: {},
    recording: { enabled: false, profile: "default" }
  });
  await expect(page.getByTestId("run-id")).toHaveText("run-new");
  await expect(page.getByTestId("panel-health").getByTestId("topic-state-ONLINE")).toBeVisible();
});

test("refresh restores the persisted run id when current is temporarily empty", async ({ page }) => {
  await installMockApi(page, { restorableRun: true });
  await page.addInitScript(() => localStorage.setItem("masar.activeRunId", "run-restored"));

  await page.goto("/?workspace=exploration");

  await expect(page.getByTestId("run-id")).toHaveText("run-restored");
  await expect(page.getByTestId("workbench")).toHaveAttribute("data-workspace", "exploration");
});
