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

test("complete rescue keeps every response metric inside the response panel", async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto("/?workspace=rescue&scenario=rescue.gazebo.tepper-response");

  const responsePanel = await page.getByTestId("panel-response").boundingBox();
  const lastMetric = await page.getByTestId("response-clearance").boundingBox();
  expect(responsePanel).not.toBeNull();
  expect(lastMetric).not.toBeNull();
  expect(responsePanel!.height).toBeLessThanOrEqual(230);
  expect(lastMetric!.y + lastMetric!.height).toBeLessThanOrEqual(
    responsePanel!.y + responsePanel!.height
  );
});

test("expanded response panel does not overlap the following health panel", async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto("/?workspace=rescue&scenario=rescue.gazebo.tepper-response");

  const responsePanel = await page.getByTestId("panel-response").boundingBox();
  const healthPanel = await page.getByTestId("panel-health").boundingBox();
  expect(responsePanel).not.toBeNull();
  expect(healthPanel).not.toBeNull();
  expect(healthPanel!.y).toBeGreaterThanOrEqual(responsePanel!.y + responsePanel!.height);
});

test("status rail fills tall viewports and remains scrollable on short viewports", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "rescue.gazebo.tepper-response"
  });

  await page.setViewportSize({ width: 1440, height: 1100 });
  await page.goto("/?workspace=rescue&scenario=rescue.gazebo.tepper-response");

  const rail = page.locator(".status-rail");
  const railBox = await rail.boundingBox();
  const lastPanelBox = await page.getByTestId("panel-logs").boundingBox();
  expect(railBox).not.toBeNull();
  expect(lastPanelBox).not.toBeNull();
  expect(railBox!.y + railBox!.height - (lastPanelBox!.y + lastPanelBox!.height)).toBeLessThanOrEqual(2);

  await page.setViewportSize({ width: 1440, height: 760 });
  await expect.poll(() => rail.evaluate((element) => element.scrollHeight > element.clientHeight)).toBe(true);
  await rail.evaluate((element) => element.scrollTo({ top: element.scrollHeight }));

  const shortRailBox = await rail.boundingBox();
  const scrolledLastPanelBox = await page.getByTestId("panel-logs").boundingBox();
  expect(shortRailBox).not.toBeNull();
  expect(scrolledLastPanelBox).not.toBeNull();
  expect(scrolledLastPanelBox!.y + scrolledLastPanelBox!.height).toBeLessThanOrEqual(
    shortRailBox!.y + shortRailBox!.height + 1
  );
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

test("group behavior places Webots above trajectory and sensors", async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  const simulation = await page.getByTestId("panel-simulation").boundingBox();
  const trajectory = await page.getByTestId("panel-trajectory").boundingBox();
  const sensors = await page.getByTestId("panel-sensors").boundingBox();
  expect(simulation).not.toBeNull();
  expect(trajectory).not.toBeNull();
  expect(sensors).not.toBeNull();
  expect(simulation!.y + simulation!.height).toBeLessThanOrEqual(trajectory!.y);
  expect(simulation!.y + simulation!.height).toBeLessThanOrEqual(sensors!.y);
  expect(Math.abs(trajectory!.y - sensors!.y)).toBeLessThanOrEqual(1);
  expect(trajectory!.x + trajectory!.width).toBeLessThanOrEqual(sensors!.x);
  expect(simulation!.x).toBeCloseTo(trajectory!.x, 0);
  expect(simulation!.x + simulation!.width).toBeCloseTo(sensors!.x + sensors!.width, 0);
});

for (const { widePanel, otherPanel } of [
  { widePanel: "trajectory", otherPanel: "sensors" },
  { widePanel: "sensors", otherPanel: "trajectory" }
]) {
  test(`wide group ${widePanel} reflows the lower panels without overlap`, async ({ page }) => {
    await page.setViewportSize({ width: 1440, height: 900 });
    await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

    const wide = page.getByTestId(`panel-${widePanel}`);
    await wide.getByRole("button", { name: "调整尺寸" }).click();
    await expect(wide).toHaveAttribute("data-size", "wide");
    await expect(wide.locator(".panel__body")).toBeVisible();

    const grid = await page.locator(".panel-grid").boundingBox();
    const wideBox = await wide.boundingBox();
    const otherBox = await page.getByTestId(`panel-${otherPanel}`).boundingBox();
    expect(grid).not.toBeNull();
    expect(wideBox).not.toBeNull();
    expect(otherBox).not.toBeNull();
    expect(wideBox!.x).toBeCloseTo(grid!.x, 0);
    expect(wideBox!.x + wideBox!.width).toBeCloseTo(grid!.x + grid!.width, 0);
    expect(Math.abs(wideBox!.height - otherBox!.height)).toBeLessThanOrEqual(1);

    for (const box of [wideBox!, otherBox!]) {
      expect(box.x).toBeGreaterThanOrEqual(grid!.x);
      expect(box.y).toBeGreaterThanOrEqual(grid!.y);
      expect(box.x + box.width).toBeLessThanOrEqual(grid!.x + grid!.width);
      expect(box.y + box.height).toBeLessThanOrEqual(grid!.y + grid!.height);
    }

    const panelsOverlap = wideBox!.x < otherBox!.x + otherBox!.width
      && wideBox!.x + wideBox!.width > otherBox!.x
      && wideBox!.y < otherBox!.y + otherBox!.height
      && wideBox!.y + wideBox!.height > otherBox!.y;
    expect(panelsOverlap).toBe(false);
  });
}

test("group behavior fixes command_world and lets the operator choose the robot type", async ({ page }) => {
  await page.goto("/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByLabel("Webots 世界")).toHaveCount(0);
  const robotType = page.getByLabel("机器人类型");
  await expect(robotType).toBeVisible();
  await expect(robotType.locator("option")).toHaveText(["e_puck", "burger"]);
});

test("control demos do not show exploration mission progress", async ({ page }) => {
  await page.goto("/?workspace=group_behavior&scenario=control.gazebo.response-deform");

  await expect(page.getByText("探索任务进程")).toHaveCount(0);
});

test("controller demo exposes five distinct strategy labels", async ({ page }) => {
  await page.goto("/?workspace=group_behavior&scenario=control.gazebo.response-controller");

  const mode = page.getByLabel("控制器任务");
  await expect(mode.locator("option")).toHaveText([
    "roundflocking-checkpoint",
    "roundflocking-heuristic",
    "transport-synchronized_slots",
    "transport-checkpoint",
    "transport-heuristic"
  ]);
  await mode.selectOption({ label: "transport-synchronized_slots" });
  await expect(mode).toHaveValue("transport");
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
