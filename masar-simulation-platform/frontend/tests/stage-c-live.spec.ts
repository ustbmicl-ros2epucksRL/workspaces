import { mkdir } from "node:fs/promises";
import { expect, test } from "@playwright/test";
import { expectRenderedFramebuffer } from "./liveAssertions";

test.skip(process.env.MASAR_STAGE_C_LIVE !== "1", "requires an active Stage C exploration Run");

test("exploration renders allocation, robot sensors, discoveries, RViz and simulator", async ({ page }) => {
  test.setTimeout(180_000);
  const runId = process.env.MASAR_STAGE_C_RUN_ID;
  const screenshotDir = process.env.MASAR_STAGE_C_SCREENSHOT_DIR;
  const simulator = process.env.MASAR_STAGE_C_SIMULATOR === "webots" ? "webots" : "gazebo";
  const scenarioId = simulator === "webots"
    ? "exploration.webots.robosar"
    : "exploration.gazebo.tepper";
  const simulationTitle = simulator === "webots" ? "Webots 仿真画面" : "Gazebo 仿真画面";
  const consoleErrors: string[] = [];
  const failedRequests: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });
  page.on("requestfailed", (request) => {
    failedRequests.push(`${request.method()} ${request.url()}: ${request.failure()?.errorText ?? "failed"}`);
  });
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto(`http://127.0.0.1:18090/?workspace=exploration&scenario=${scenarioId}`);

  await expect(page.getByTestId("run-id")).toHaveText(runId ?? /^run-/i, { timeout: 30_000 });
  await expect(page.locator(".run-state")).toHaveText("RUNNING");
  await page.getByRole("button", { name: "开始任务" }).click();

  const allocationImage = page.getByTestId("task-allocation-image");
  await expect
    .poll(
      () => allocationImage.evaluate((image: HTMLImageElement) => image.naturalWidth),
      { message: "task allocation stream did not render a real frame", timeout: 60_000 }
    )
    .toBeGreaterThan(0);
  await expect(page.getByTestId("map-coverage")).toHaveText(/^\d+(?:\.\d+)?%$/, {
    timeout: 60_000
  });
  await expect(page.getByTestId("active-agent-count")).toHaveText(/^\d+\/5$/, {
    timeout: 60_000
  });
  await expect(page.getByTestId("map-assignment-count")).not.toHaveText("—", { timeout: 60_000 });
  await expect(page.getByTestId("frontier-count")).not.toHaveText("—", { timeout: 60_000 });
  await expect(page.getByTestId("panel-map").getByText("本轮分配", { exact: true })).toBeVisible();
  await expect(page.getByTestId("panel-mission").getByTestId("assignment-count")).toHaveCount(0);
  await expect(page.getByTestId("completed-task-count")).toHaveCount(0);

  const selector = page.getByLabel("选择观测机器人");
  await expect(selector.locator("option")).toHaveCount(5, { timeout: 30_000 });
  for (let index = 0; index < 5; index += 1) {
    await selector.selectOption(String(index));
    const cameraTopic = simulator === "webots"
      ? new RegExp(
        `^(?:/robot_${index}/camera/image_color|/robosar_agent_bringup_node/agent_${index}/camera/image_raw)$`
      )
      : `/robosar_agent_bringup_node/agent_${index}/camera/image_raw`;
    await expect(page.getByTestId("sensor-source-topic")).toHaveText(
      cameraTopic,
      { timeout: 30_000 }
    );
    await expect(page.getByTestId("panel-sensors")).toContainText(/最近 \d+\.\d{2} m · \d+\/\d+/, {
      timeout: 30_000
    });
    await expect(page.getByTestId("panel-sensors")).toContainText(/x -?\d+\.\d+ m · y -?\d+\.\d+ m/, {
      timeout: 30_000
    });
  }

  const emergencyRow = page
    .getByTestId("panel-health")
    .locator(".health-row")
    .filter({ hasText: "/robosar_sim/emergency_objects" });
  await expect(emergencyRow.getByText("在线", { exact: true })).toBeVisible({ timeout: 60_000 });
  await expect
    .poll(async () => {
      const counts = await Promise.all([
        page.getByTestId("emergency-count-victim").textContent(),
        page.getByTestId("emergency-count-fire").textContent(),
        page.getByTestId("emergency-count-box").textContent()
      ]);
      return counts.reduce((total, value) => total + Number(value ?? 0), 0);
    }, { message: "no discovered victim, fire or push-obstacle reached the panel", timeout: 120_000 })
    .toBeGreaterThan(0);
  await expect(page.locator(".emergency-feed li").filter({ hasText: /\(-?\d+\.\d{2}, -?\d+\.\d{2}\)/ }).first())
    .toBeVisible();

  await expect(page.getByTestId("health-robot-row")).toHaveCount(5);
  const simulationCanvas = page.frameLocator(`iframe[title="${simulationTitle}"]`).locator("canvas").first();
  const rvizCanvas = page.frameLocator('iframe[title="RViz 多机器人视图"]').locator("canvas").first();
  await expect(simulationCanvas).toBeVisible({ timeout: 30_000 });
  await expect(rvizCanvas).toBeVisible({ timeout: 30_000 });
  await expectRenderedFramebuffer(simulationCanvas, simulator === "webots" ? "Webots" : "Gazebo");
  await expectRenderedFramebuffer(rvizCanvas, "RViz");

  if (screenshotDir) {
    await mkdir(screenshotDir, { recursive: true });
    await page.screenshot({ path: `${screenshotDir}/stage-c-${simulator}.png`, fullPage: true });
    await page.getByTestId("panel-map").screenshot({
      path: `${screenshotDir}/stage-c-${simulator}-allocation.png`
    });
    await page.getByTestId("panel-mission").screenshot({
      path: `${screenshotDir}/stage-c-${simulator}-mission.png`
    });
  }

  expect(consoleErrors).toEqual([]);
  expect(
    failedRequests.filter(
      (failure) => !(failure.includes("/video/ros1/stream") && failure.includes("net::ERR_ABORTED"))
    )
  ).toEqual([]);
});
