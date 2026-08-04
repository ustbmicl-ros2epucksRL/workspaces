import { mkdir } from "node:fs/promises";
import { expect, test } from "@playwright/test";

test.skip(
  process.env.MASAR_EXPLORATION_MEDIA_LIVE !== "1",
  "requires an active Gazebo exploration Run"
);

test("default camera and allocation frame survive exploration HOME and STOP", async ({ page }) => {
  test.setTimeout(180_000);
  const runId = process.env.MASAR_EXPLORATION_MEDIA_RUN_ID;
  const screenshotDir = process.env.MASAR_EXPLORATION_MEDIA_SCREENSHOT_DIR;
  const consoleErrors: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });

  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto(
    "http://127.0.0.1:18090/?workspace=exploration&scenario=exploration.gazebo.tepper"
  );

  await expect(page.getByTestId("run-id")).toHaveText(runId ?? /^run-/i, { timeout: 30_000 });
  await expect(page.locator(".run-state")).toHaveText("RUNNING");
  const selector = page.getByLabel("选择观测机器人");
  await expect(selector).toHaveValue("0");
  await page.getByRole("button", { name: "开始任务" }).click();

  await expect(page.getByTestId("sensor-source-topic")).toHaveText(
    "/robosar_agent_bringup_node/agent_0/camera/image_raw",
    { timeout: 30_000 }
  );
  const camera = page.locator("img.camera-stream");
  await expect.poll(
    () => camera.evaluate((image: HTMLImageElement) => image.naturalWidth),
    { message: "default robot camera did not commit its first frame", timeout: 30_000 }
  ).toBeGreaterThan(0);

  const allocation = page.getByTestId("task-allocation-image");
  await expect.poll(
    () => allocation.evaluate((image: HTMLImageElement) => image.naturalWidth),
    { message: "allocation image did not commit its first frame", timeout: 60_000 }
  ).toBeGreaterThan(0);

  await page.getByRole("button", { name: "探索组 HOME" }).click();
  await page.waitForTimeout(5_000);
  await expect.poll(
    () => allocation.evaluate((image: HTMLImageElement) => image.naturalWidth)
  ).toBeGreaterThan(0);
  await expect(page.getByTestId("panel-map")).not.toContainText("任务分配图连接重试中");
  const afterHomeSrc = await allocation.getAttribute("src");

  await page.getByRole("button", { name: "开始任务" }).click();
  await expect.poll(
    () => allocation.getAttribute("src"),
    { message: "restarted allocator did not publish a new visible frame", timeout: 60_000 }
  ).not.toBe(afterHomeSrc);
  await expect.poll(
    () => allocation.evaluate((image: HTMLImageElement) => image.naturalWidth)
  ).toBeGreaterThan(0);

  await page.getByRole("button", { name: "停止任务" }).click();
  await page.waitForTimeout(5_000);
  await expect.poll(
    () => allocation.evaluate((image: HTMLImageElement) => image.naturalWidth)
  ).toBeGreaterThan(0);
  await expect(page.getByTestId("panel-map")).not.toContainText("任务分配图连接重试中");

  if (screenshotDir) {
    await mkdir(screenshotDir, { recursive: true });
    await page.screenshot({ path: `${screenshotDir}/exploration-media-after-stop.png`, fullPage: true });
    await page.getByTestId("panel-map").screenshot({
      path: `${screenshotDir}/allocation-retained-after-stop.png`
    });
    await page.getByTestId("panel-sensors").screenshot({
      path: `${screenshotDir}/default-robot-camera.png`
    });
  }

  expect(consoleErrors).toEqual([]);
});
