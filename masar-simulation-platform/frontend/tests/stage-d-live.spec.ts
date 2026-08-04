import { mkdir } from "node:fs/promises";
import { expect, test } from "@playwright/test";
import { expectRenderedFramebuffer } from "./liveAssertions";

test.skip(process.env.MASAR_STAGE_D_LIVE !== "1", "requires an active Stage D rescue Run");

test("complete rescue renders exploration and active response-group state", async ({ page }) => {
  test.setTimeout(300_000);
  const runId = process.env.MASAR_STAGE_D_RUN_ID;
  const screenshotDir = process.env.MASAR_STAGE_D_SCREENSHOT_DIR;
  const consoleErrors: string[] = [];
  const failedRequests: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });
  page.on("requestfailed", (request) => {
    failedRequests.push(`${request.method()} ${request.url()}: ${request.failure()?.errorText ?? "failed"}`);
  });

  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto(
    "http://127.0.0.1:18090/?workspace=rescue&scenario=rescue.gazebo.tepper-response"
  );
  await expect(page.getByTestId("run-id")).toHaveText(runId ?? /^run-/i, { timeout: 30_000 });
  await expect(page.locator(".run-state")).toHaveText("RUNNING");

  const sensorPanel = page.getByTestId("panel-sensors");
  const robotSelector = page.getByLabel("选择观测机器人");
  const laserReading = sensorPanel.locator(".sensor-readouts > div")
    .filter({ hasText: "LaserScan" })
    .locator("strong");
  await expect(robotSelector.locator("option")).toHaveCount(9, { timeout: 30_000 });
  await expect(robotSelector.locator("option").first()).toHaveText("探索机器人 0");
  await expect(robotSelector.locator("option").last()).toHaveText("响应机器人 3");
  await expect(page.getByTestId("sensor-source-topic")).toHaveText(
    "/robosar_agent_bringup_node/agent_0/camera/image_raw"
  );
  await expect(laserReading).toContainText(/^最近 \d+\.\d{2} m/, { timeout: 30_000 });
  await expect(page.getByTestId("infrared-reading")).toHaveCount(0);
  await expect(page.getByTestId("imu-reading")).toHaveCount(0);

  await robotSelector.selectOption("tb:0");
  await expect(page.getByTestId("sensor-source-topic")).toHaveText("/tb_0/camera/color/image_raw");
  await expect(laserReading).toContainText(/^最近 \d+\.\d{2} m/, { timeout: 30_000 });
  await expect(page.getByTestId("imu-reading")).toContainText(/加速度 .*角速度/, {
    timeout: 30_000
  });
  await expect(page.getByTestId("infrared-reading")).toHaveCount(0);

  await page.getByRole("button", { name: "开始任务" }).click();

  const allocationImage = page.getByTestId("task-allocation-image");
  await expect.poll(
    () => allocationImage.evaluate((image: HTMLImageElement) => image.naturalWidth),
    { message: "rescue allocation snapshot did not render", timeout: 90_000 }
  ).toBeGreaterThan(0);
  await expect(page.getByTestId("map-coverage")).toHaveText(/^\d+(?:\.\d+)?%$/, { timeout: 90_000 });
  await expect(page.getByTestId("active-agent-count")).toHaveText(/^\d+\/5$/, { timeout: 90_000 });
  await expect.poll(async () => {
    const counts = await Promise.all([
      page.getByTestId("emergency-count-victim").textContent(),
      page.getByTestId("emergency-count-fire").textContent(),
      page.getByTestId("emergency-count-box").textContent()
    ]);
    return counts.reduce((total, value) => total + Number(value ?? 0), 0);
  }, { message: "no rescue object was discovered", timeout: 180_000 }).toBeGreaterThan(0);

  const response = page.getByTestId("panel-response");
  await expect(response).not.toContainText("未提供");
  await expect(response.getByTestId("response-state")).toHaveText(/^(?!等待|空闲).+ · .+$/, {
    timeout: 180_000
  });
  await expect(response.getByTestId("response-motion")).toHaveText("已启用", { timeout: 180_000 });
  await expect(response.getByTestId("response-active-task")).toHaveText(/^#-?\d+ · .+ · .+$/, {
    timeout: 180_000
  });
  await expect(response.getByTestId("response-queue")).toHaveText(/^[1-9]\d* 项 · \d+ 执行中 · \d+ 待分配$/, {
    timeout: 180_000
  });
  await expect(response.getByTestId("response-formation-error")).toHaveText(
    /^RMS \d+\.\d{3} m · 最大 \d+\.\d{3} m$/,
    { timeout: 60_000 }
  );
  await expect(response.getByTestId("response-clearance")).toHaveText(/^\d+\.\d{3} m$/, {
    timeout: 60_000
  });
  await expect(response.getByTestId("response-behavior")).toHaveCount(0);
  await expect(response.getByTestId("response-planning")).toHaveCount(0);
  await expect(response.getByTestId("response-scale")).toHaveCount(0);
  await expect(response.getByTestId("response-assigned")).toHaveCount(0);
  await expect(response.getByTestId("response-result")).toHaveCount(0);

  const responseHealth = page.getByTestId("panel-health").locator(".health-row")
    .filter({ hasText: "/response_group/status" });
  await expect(responseHealth.getByText("在线", { exact: true })).toBeVisible({ timeout: 60_000 });

  const gazeboCanvas = page.frameLocator('iframe[title="Gazebo 仿真画面"]').locator("canvas").first();
  const rvizCanvas = page.frameLocator('iframe[title="RViz 多机器人视图"]').locator("canvas").first();
  await expect(gazeboCanvas).toBeVisible({ timeout: 30_000 });
  await expect(rvizCanvas).toBeVisible({ timeout: 30_000 });
  await expectRenderedFramebuffer(gazeboCanvas, "Gazebo rescue");
  await expectRenderedFramebuffer(rvizCanvas, "RViz rescue");

  if (screenshotDir) {
    await mkdir(screenshotDir, { recursive: true });
    await page.screenshot({ path: `${screenshotDir}/stage-d-rescue.png`, fullPage: true });
    await response.screenshot({ path: `${screenshotDir}/stage-d-response.png` });
    await page.getByTestId("panel-mission").screenshot({ path: `${screenshotDir}/stage-d-mission.png` });
  }

  expect(consoleErrors).toEqual([]);
  expect(
    failedRequests.filter(
      (failure) => !(failure.includes("/video/ros1/stream") && failure.includes("net::ERR_ABORTED"))
    )
  ).toEqual([]);
});
