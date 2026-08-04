import { mkdir } from "node:fs/promises";
import { expect, test } from "@playwright/test";
import { expectRenderedFramebuffer } from "./liveAssertions";

test.skip(process.env.MASAR_STAGE_B_LIVE !== "1", "requires an active Stage B e-puck Run");

test("e-puck swarm renders every robot sensor and a live XY trajectory", async ({ page }) => {
  test.skip(process.env.MASAR_STAGE_B_ROBOT === "burger", "e-puck acceptance only");
  test.setTimeout(60_000);
  const runId = process.env.MASAR_STAGE_B_RUN_ID;
  const expectedPattern = process.env.MASAR_STAGE_B_PATTERN ?? "aggregation_pattern";
  const expectedRobots = Number(process.env.MASAR_STAGE_B_ROBOTS ?? "5");
  const screenshotDir = process.env.MASAR_STAGE_B_SCREENSHOT_DIR;
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
    "http://127.0.0.1:18090/?workspace=group_behavior&scenario=swarm.webots.ros2swarm"
  );

  await expect(page.getByTestId("run-id")).toHaveText(runId ?? /^run-/i, { timeout: 20_000 });
  await expect(page.getByRole("combobox", { name: "群行为策略" })).toHaveValue(
    expectedPattern
  );
  await page.getByRole("button", { name: "开始任务" }).click();

  const selector = page.getByLabel("选择观测机器人");
  await expect(selector.locator("option")).toHaveCount(expectedRobots, { timeout: 20_000 });
  await selector.selectOption("2");
  await expect(page.getByTestId("sensor-source-topic")).toHaveText(
    "/robot_2/camera/image_color",
    { timeout: 20_000 }
  );
  await expect(page.getByTestId("infrared-reading")).toContainText(/8 路 · 最近/, {
    timeout: 20_000
  });
  await expect(page.locator(".sensor-camera .camera-reticle")).toHaveCount(0);
  await expect(page.locator(".sensor-camera").getByText("在线", { exact: true })).toHaveCount(0);
  await expect
    .poll(
      () => page.locator("img.camera-stream").evaluate((image: HTMLImageElement) => image.naturalWidth),
      { message: "selected e-puck camera did not render", timeout: 20_000 }
    )
    .toBeGreaterThan(0);

  await expect(page.getByTestId("trajectory-robot-row")).toHaveCount(expectedRobots);
  await expect(page.getByTestId("trajectory-robot-row").first()).not.toContainText("等待");
  await expect(page.getByTestId("health-robot-row")).toHaveCount(expectedRobots);
  await expect(page.getByTestId("panel-health").getByText("在线").first()).toBeVisible();

  await expect(page.getByLabel("日志进程")).toHaveValue("swarm_simulation");
  await expect(page.getByTestId("process-log-output")).toContainText(
    expectedPattern,
    { timeout: 20_000 }
  );

  const trajectory = page.getByTestId("trajectory-plot");
  await expect(trajectory).toBeVisible();
  await expect
    .poll(() => page.locator('[data-testid^="trajectory-path-"]').count(), {
      message: "no robot accumulated enough odometry samples to draw a trajectory",
      timeout: 20_000
    })
    .toBeGreaterThan(0);
  const trajectoryPoints = await page.locator('[data-testid^="trajectory-path-"]').evaluateAll((paths) =>
    paths.map((path) => path.getAttribute("points")?.trim().split(/\s+/).length ?? 0)
  );
  expect(Math.max(...trajectoryPoints)).toBeGreaterThan(2);
  const webotsFrame = page.frameLocator('iframe[title="Webots 仿真画面"]');
  const webotsCanvas = webotsFrame.locator("canvas").first();
  await expect(webotsCanvas).toBeVisible({ timeout: 20_000 });
  await expect(webotsFrame.getByText("Connecting...", { exact: true })).toBeHidden({ timeout: 20_000 });
  await expectRenderedFramebuffer(webotsCanvas, "Webots");

  if (screenshotDir) {
    await mkdir(screenshotDir, { recursive: true });
    await page.screenshot({ path: `${screenshotDir}/stage-b-epuck-run.png`, fullPage: true });
    await page.getByTestId("panel-sensors").screenshot({
      path: `${screenshotDir}/stage-b-epuck-sensors.png`
    });
    await page.getByTestId("panel-trajectory").screenshot({
      path: `${screenshotDir}/stage-b-epuck-trajectory.png`
    });
  }

  expect(consoleErrors).toEqual([]);
  expect(
    failedRequests.filter(
      (failure) => !(failure.includes("/video/ros2/stream") && failure.includes("net::ERR_ABORTED"))
    )
  ).toEqual([]);
});

test("Burger swarm explains the missing camera and renders its available sensors", async ({ page }) => {
  test.skip(process.env.MASAR_STAGE_B_ROBOT !== "burger", "requires an active Burger Run");
  test.setTimeout(60_000);
  const runId = process.env.MASAR_STAGE_B_RUN_ID;
  const expectedRobots = Number(process.env.MASAR_STAGE_B_ROBOTS ?? "5");
  const screenshotDir = process.env.MASAR_STAGE_B_SCREENSHOT_DIR;
  const consoleErrors: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto(
    "http://127.0.0.1:18090/?workspace=group_behavior&scenario=swarm.webots.ros2swarm"
  );

  await expect(page.getByTestId("run-id")).toHaveText(runId ?? /^run-/i, { timeout: 20_000 });
  await expect(page.getByRole("combobox", { name: "机器人类型" })).toHaveValue("burger");
  await page.getByRole("button", { name: "开始任务" }).click();

  const selector = page.getByLabel("选择观测机器人");
  await expect(selector.locator("option")).toHaveCount(expectedRobots, { timeout: 20_000 });
  await selector.selectOption("2");
  await expect(page.getByTestId("sensor-camera-state")).toHaveText("Burger 未配备相机");
  await expect(page.getByTestId("sensor-source-topic")).toHaveText("未配备");
  await expect(page.locator("img.camera-stream")).toHaveCount(0);
  await expect(page.getByTestId("imu-reading")).toContainText(/加速度 .*角速度/, {
    timeout: 20_000
  });
  await expect(page.getByTestId("panel-sensors")).toContainText(/最近 \d+\.\d+ m · \d+\/\d+/, {
    timeout: 20_000
  });
  await expect(page.getByTestId("panel-sensors")).toContainText(/x -?\d+\.\d+ m · y -?\d+\.\d+ m/, {
    timeout: 20_000
  });
  await expect(page.getByTestId("health-robot-row")).toHaveCount(expectedRobots);
  await expect(page.getByTestId("panel-health").getByText("在线")).toHaveCount(expectedRobots, {
    timeout: 20_000
  });
  await expect(page.getByTestId("trajectory-plot")).toBeVisible();

  const webotsFrame = page.frameLocator('iframe[title="Webots 仿真画面"]');
  const webotsCanvas = webotsFrame.locator("canvas").first();
  await expect(webotsCanvas).toBeVisible({ timeout: 20_000 });
  await expectRenderedFramebuffer(webotsCanvas, "Webots Burger");

  if (screenshotDir) {
    await mkdir(screenshotDir, { recursive: true });
    await page.screenshot({ path: `${screenshotDir}/stage-b-burger-run.png`, fullPage: true });
    await page.getByTestId("panel-sensors").screenshot({
      path: `${screenshotDir}/stage-b-burger-sensors.png`
    });
  }
  expect(consoleErrors).toEqual([]);
});
