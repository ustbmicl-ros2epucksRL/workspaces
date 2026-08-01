import { expect, test } from "@playwright/test";
import { installMockApi } from "./mockApi";

test.skip(process.env.MASAR_LIVE_ROS !== "1", "requires live loopback ROS sidecars and test publishers");

test("renders a real ROS1 OccupancyGrid through the same-origin gateway", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources: [
      {
        graph: "ros1",
        topic_pattern: "/sim_map",
        message_types: ["nav_msgs/OccupancyGrid"],
        capability: "map",
        required: true,
        availability: "UNKNOWN"
      }
    ]
  });
  await page.goto("http://127.0.0.1:18090/?workspace=exploration");

  await expect(page.getByTestId("panel-map").getByText("在线")).toBeVisible({ timeout: 15_000 });
  const canvas = page.getByTestId("map-canvas").locator("canvas");
  await expect(canvas).toBeVisible();
  expect(await canvas.evaluate((element) => (element as HTMLCanvasElement).toDataURL().length)).toBeGreaterThan(100);
  if (process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR) {
    await page.screenshot({ path: `${process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR}/ros1-map.png`, fullPage: true });
  }
});

test("renders ROS2 odometry and bounded LaserScan values", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "swarm.webots.ros2swarm",
    dataSources: [
      {
        graph: "ros2",
        topic_pattern: "/<robot>/odom",
        message_types: ["nav_msgs/msg/Odometry"],
        capability: "odometry",
        required: true,
        availability: "UNKNOWN"
      },
      {
        graph: "ros2",
        topic_pattern: "/<robot>/scan",
        message_types: ["sensor_msgs/msg/LaserScan"],
        capability: "laser",
        required: true,
        availability: "UNKNOWN"
      }
    ]
  });
  await page.goto("http://127.0.0.1:18090/?workspace=group_behavior&scenario=swarm.webots.ros2swarm");

  await expect(page.getByText(/最近 0\.40 m/)).toBeVisible({ timeout: 15_000 });
  const trajectory = page.getByTestId("trajectory-canvas").locator("canvas");
  await expect(trajectory).toBeVisible();
  expect(await trajectory.evaluate((element) => {
    const canvas = element as HTMLCanvasElement;
    const pixels = canvas.getContext("2d")!.getImageData(0, 0, canvas.width, canvas.height).data;
    return pixels.some((value, index) => index % 4 === 3 && value > 0);
  })).toBe(true);
  await expect(page.getByTestId("panel-health").getByText("在线").first()).toBeVisible();
  if (process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR) {
    await page.screenshot({ path: `${process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR}/ros2-swarm.png`, fullPage: true });
    await page.getByTestId("panel-trajectory").screenshot({
      path: `${process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR}/ros2-trajectory.png`
    });
    await trajectory.screenshot({
      path: `${process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR}/ros2-trajectory-canvas.png`
    });
  }
});
