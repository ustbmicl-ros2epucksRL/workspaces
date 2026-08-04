import { createHash } from "node:crypto";
import { mkdir } from "node:fs/promises";
import { expect, test } from "@playwright/test";
import { expectRenderedFramebuffer } from "./liveAssertions";

test.skip(process.env.MASAR_STAGE_A_LIVE !== "1", "requires an active Stage A Webots swarm Run");

test("active Domain 42 swarm renders changing ROS data and Webots noVNC", async ({ page }) => {
  test.skip(process.env.MASAR_STAGE_A_SCENARIO === "exploration", "swarm acceptance only");
  const runId = process.env.MASAR_STAGE_A_RUN_ID;
  const screenshotDir = process.env.MASAR_STAGE_A_SCREENSHOT_DIR;
  const consoleErrors: string[] = [];
  const failedRequests: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });
  page.on("requestfailed", (request) => {
    failedRequests.push(`${request.method()} ${request.url()}: ${request.failure()?.errorText ?? "failed"}`);
  });

  await page.goto(
    "http://127.0.0.1:18090/?workspace=group_behavior&scenario=swarm.webots.ros2swarm"
  );

  await expect(page.getByTestId("run-id")).toHaveText(runId ?? /^run-/i, { timeout: 20_000 });
  await expect(page.getByText("ROS2 Domain").locator("..").getByText("42")).toBeVisible();
  await page.getByRole("button", { name: "开始任务" }).click();
  await expect(page.getByText(/最近 \d+\.\d{2} m · \d+\/\d+/)).toBeVisible({ timeout: 20_000 });
  await expect(page.getByTestId("panel-health").getByText("在线").first()).toBeVisible({ timeout: 20_000 });

  const trajectory = page.getByTestId("trajectory-plot");
  await expect(trajectory).toBeVisible();
  const trajectoryPaths = page.locator('[data-testid^="trajectory-path-"]');
  await expect
    .poll(() => trajectoryPaths.count(), {
      message: "no robot accumulated enough odometry samples to draw a trajectory",
      timeout: 20_000
    })
    .toBeGreaterThan(0);
  const trajectoryBefore = await trajectoryPaths.evaluateAll((paths) =>
    paths.map((path) => path.getAttribute("points") ?? "").join("|")
  );
  await expect
    .poll(
      () =>
        trajectoryPaths.evaluateAll((paths) =>
          paths.map((path) => path.getAttribute("points") ?? "").join("|")
        ),
      { message: "robot SVG trajectories did not change", timeout: 20_000 }
    )
    .not.toBe(trajectoryBefore);

  const webotsFrame = page.frameLocator('iframe[title="Webots 仿真画面"]');
  const webotsCanvas = webotsFrame.locator("canvas").first();
  await expect(webotsCanvas).toBeVisible({ timeout: 20_000 });
  await expectRenderedFramebuffer(webotsCanvas, "Webots");
  const webotsBefore = await webotsCanvas.screenshot();
  const box = await webotsCanvas.boundingBox();
  expect(box).not.toBeNull();
  if (box) {
    await page.mouse.move(box.x + box.width * 0.55, box.y + box.height * 0.55);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width * 0.7, box.y + box.height * 0.45, { steps: 8 });
    await page.mouse.up();
    await page.mouse.wheel(0, -480);
  }
  await page.waitForTimeout(2_000);
  const webotsAfter = await webotsCanvas.screenshot();
  expect(createHash("sha256").update(webotsAfter).digest("hex")).not.toBe(
    createHash("sha256").update(webotsBefore).digest("hex")
  );

  if (screenshotDir) {
    await mkdir(screenshotDir, { recursive: true });
    await page.screenshot({ path: `${screenshotDir}/stage-a-swarm.png`, fullPage: true });
    await trajectory.screenshot({ path: `${screenshotDir}/stage-a-trajectory.png` });
    await webotsCanvas.screenshot({ path: `${screenshotDir}/stage-a-webots.png` });
  }

  expect(consoleErrors).toEqual([]);
  expect(failedRequests).toEqual([]);
});

test("active exploration renders task allocation, sensors, RViz and simulator", async ({ page }) => {
  test.skip(process.env.MASAR_STAGE_A_SCENARIO !== "exploration", "exploration acceptance only");
  const runId = process.env.MASAR_STAGE_A_RUN_ID;
  const screenshotDir = process.env.MASAR_STAGE_A_SCREENSHOT_DIR;
  const simulator = process.env.MASAR_STAGE_A_SIMULATOR === "gazebo" ? "gazebo" : "webots";
  const scenarioId = simulator === "gazebo" ? "exploration.gazebo.tepper" : "exploration.webots.robosar";
  const simulationTitle = simulator === "gazebo" ? "Gazebo 仿真画面" : "Webots 仿真画面";
  const consoleErrors: string[] = [];
  const failedRequests: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });
  page.on("requestfailed", (request) => {
    failedRequests.push(`${request.method()} ${request.url()}: ${request.failure()?.errorText ?? "failed"}`);
  });

  await page.goto(
    `http://127.0.0.1:18090/?workspace=exploration&scenario=${scenarioId}`
  );

  await expect(page.getByTestId("run-id")).toHaveText(runId ?? /^run-/i, { timeout: 20_000 });
  await expect(page.getByText("ROS2 Domain").locator("..").getByText("42")).toBeVisible();
  await page.getByRole("button", { name: "开始任务" }).click();
  await expect(page.getByTestId("panel-map").getByText("在线")).toBeVisible({ timeout: 20_000 });
  await expect(page.getByText(/最近 \d+\.\d{2} m · \d+\/\d+/)).toBeVisible({ timeout: 20_000 });
  await expect(page.getByTestId("sensor-source-topic")).not.toHaveText("未提供", { timeout: 20_000 });
  await expect(page.locator(".sensor-camera").getByText("在线", { exact: true })).toHaveCount(0);
  await expect
    .poll(
      () => page.locator("img.camera-stream").evaluate((image: HTMLImageElement) => image.naturalWidth),
      { message: "camera stream did not render a real frame", timeout: 20_000 }
    )
    .toBeGreaterThan(0);
  await expect(page.getByTestId("panel-health").getByText("在线").first()).toBeVisible({ timeout: 20_000 });

  const allocationImage = page.getByTestId("task-allocation-image");
  await expect(allocationImage).toBeVisible();
  await expect
    .poll(
      () => allocationImage.evaluate((image: HTMLImageElement) => image.naturalWidth),
      { message: "task allocation stream did not render a real frame", timeout: 20_000 }
    )
    .toBeGreaterThan(0);

  const webotsCanvas = page
    .frameLocator(`iframe[title="${simulationTitle}"]`)
    .locator("canvas")
    .first();
  const rvizCanvas = page
    .frameLocator('iframe[title="RViz 多机器人视图"]')
    .locator("canvas")
    .first();
  await expect(webotsCanvas).toBeVisible({ timeout: 20_000 });
  await expect(rvizCanvas).toBeVisible({ timeout: 20_000 });
  await expectRenderedFramebuffer(webotsCanvas, simulator === "gazebo" ? "Gazebo" : "Webots");
  await expectRenderedFramebuffer(rvizCanvas, "RViz");

  if (screenshotDir) {
    await mkdir(screenshotDir, { recursive: true });
    await page.screenshot({ path: `${screenshotDir}/stage-a-exploration-${simulator}.png`, fullPage: true });
    await allocationImage.screenshot({ path: `${screenshotDir}/stage-a-${simulator}-allocation.png` });
    await webotsCanvas.screenshot({ path: `${screenshotDir}/stage-a-exploration-${simulator}-simulation.png` });
    await rvizCanvas.screenshot({ path: `${screenshotDir}/stage-a-exploration-${simulator}-rviz.png` });
  }

  expect(consoleErrors).toEqual([]);
  expect(failedRequests).toEqual([]);
});
