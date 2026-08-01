import { expect, test } from "@playwright/test";
import { installMockApi } from "./mockApi";

const corePanels = ["simulation", "rviz", "map", "sensors"];

test.beforeEach(async ({ page }) => {
  await installMockApi(page);
});

test("exploration and rescue show synchronized core panels at 1440x900", async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 900 });
  for (const workspace of ["exploration", "rescue"]) {
    await page.goto(`/?workspace=${workspace}`);
    await expect(page.getByTestId("workbench")).toHaveAttribute("data-workspace", workspace);
    for (const panel of corePanels) {
      const locator = page.getByTestId(`panel-${panel}`);
      await expect(locator).toBeVisible();
      const box = await locator.boundingBox();
      expect(box).not.toBeNull();
      expect(box!.y + box!.height).toBeLessThanOrEqual(900);
    }
  }
});

test("group behavior remains a separate no-RViz validation workspace", async ({ page }) => {
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");
  await expect(page.getByTestId("workbench")).toHaveAttribute("data-workspace", "group_behavior");
  await expect(page.getByTestId("panel-simulation")).toBeVisible();
  await expect(page.getByTestId("panel-rviz")).toHaveCount(0);
  await expect(page.getByText("聚合度")).toHaveCount(0);
  await expect(page.getByText("平均间距")).toHaveCount(0);
  await expect(page.getByText("速度一致性")).toHaveCount(0);
});

test("mobile layout is ordered, non-overlapping and has a compact toolbar", async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto("/?workspace=exploration");

  const toolbar = await page.getByTestId("command-toolbar").boundingBox();
  expect(toolbar).not.toBeNull();
  expect(toolbar!.height).toBeLessThanOrEqual(56);
  expect(await page.evaluate(() => document.documentElement.scrollWidth)).toBeLessThanOrEqual(390);

  let previousBottom = 0;
  for (const panel of corePanels) {
    const box = await page.getByTestId(`panel-${panel}`).boundingBox();
    expect(box).not.toBeNull();
    expect(box!.y).toBeGreaterThanOrEqual(previousBottom);
    previousBottom = box!.y + box!.height;
  }
});
