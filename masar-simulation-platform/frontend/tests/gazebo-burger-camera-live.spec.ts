import { expect, test } from "@playwright/test";

test.skip(
  process.env.MASAR_GAZEBO_BURGER_CAMERA_LIVE !== "1",
  "requires an active Gazebo response-controller Run"
);

test("Gazebo Burger switches one live D435 stream across all response robots", async ({ page }) => {
  test.setTimeout(180_000);
  const runId = process.env.MASAR_GAZEBO_BURGER_CAMERA_RUN_ID;
  const consoleErrors: string[] = [];
  const snapshotTopics: string[] = [];
  const streamTopics: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });
  page.on("request", (request) => {
    const url = new URL(request.url());
    if (url.pathname.includes("/video/ros1/snapshot")) {
      snapshotTopics.push(url.searchParams.get("topic") ?? "");
    }
    if (url.pathname.includes("/video/ros1/stream")) {
      streamTopics.push(url.searchParams.get("topic") ?? "");
    }
  });

  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto(
    "/?workspace=group_behavior&scenario=control.gazebo.response-controller"
  );

  await expect(page.getByTestId("run-id")).toHaveText(runId ?? /^run-/i, {
    timeout: 30_000
  });
  await expect(page.locator(".run-state")).toHaveText("RUNNING");

  const selector = page.getByLabel("选择观测机器人");
  await expect(selector).toHaveValue("0", { timeout: 30_000 });
  await expect(selector.locator("option")).toHaveCount(4);
  await expect(page.getByTestId("sensor-source-topic")).toHaveText(
    "/tb_0/camera/color/image_raw",
    { timeout: 90_000 }
  );

  const camera = page.locator("img.camera-stream");
  await expect.poll(
    () => camera.evaluate((image: HTMLImageElement) => image.naturalWidth),
    { message: "Gazebo Burger D435 did not render the first live frame", timeout: 30_000 }
  ).toBeGreaterThan(0);
  await expect(page.getByTestId("sensor-camera-state")).toHaveCount(0);
  await expect.poll(() => streamTopics.at(-1)).toBe("/tb_0/camera/color/image_raw");

  for (let index = 1; index < 4; index += 1) {
    const started = await page.evaluate(() => performance.now());
    await selector.selectOption(String(index));
    await expect(page.getByTestId("sensor-source-topic")).toHaveText(
      `/tb_${index}/camera/color/image_raw`
    );
    await expect.poll(() => streamTopics.at(-1)).toBe(`/tb_${index}/camera/color/image_raw`);
    await expect.poll(
      () => camera.evaluate((image: HTMLImageElement) => image.naturalWidth),
      { message: `Gazebo Burger live frame ${index} did not render`, timeout: 5_000 }
    ).toBeGreaterThan(0);
    const elapsed = await page.evaluate((start) => performance.now() - start, started);
    expect(elapsed).toBeLessThan(5_000);
  }

  expect(snapshotTopics).toEqual([]);
  expect(new Set(streamTopics)).toEqual(new Set(
    Array.from({ length: 4 }, (_, index) => `/tb_${index}/camera/color/image_raw`)
  ));
  expect(consoleErrors).toEqual([]);
});
