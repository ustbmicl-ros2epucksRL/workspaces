import { expect, test } from "@playwright/test";
import { installMockApi } from "./mockApi";

test.skip(process.env.MASAR_LIVE_ROS !== "1", "requires live loopback ROS sidecars and test publishers");

test("renders the ROS1 task-allocation image through the same-origin video gateway", async ({ page }) => {
  await installMockApi(page, {
    activeRun: true,
    runScenario: "exploration.gazebo.tepper",
    dataSources: [
      {
        graph: "ros1",
        topic_pattern: "/task_allocation_image",
        message_types: ["sensor_msgs/Image"],
        capability: "task_allocation_image",
        required: false,
        availability: "UNKNOWN"
      }
    ]
  });
  await page.goto("http://127.0.0.1:18090/?workspace=exploration");

  await expect(page.getByTestId("panel-map").getByText("在线")).toBeVisible({ timeout: 15_000 });
  const allocationImage = page.getByTestId("task-allocation-image");
  await expect(allocationImage).toBeVisible();
  await expect
    .poll(
      () => allocationImage.evaluate((image: HTMLImageElement) => image.naturalWidth),
      { message: "task allocation stream did not render a real frame", timeout: 15_000 }
    )
    .toBeGreaterThan(0);
  if (process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR) {
    await page.screenshot({ path: `${process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR}/ros1-allocation.png`, fullPage: true });
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
  const trajectory = page.getByTestId("trajectory-plot");
  await expect(trajectory).toBeVisible();
  const paths = page.locator('[data-testid^="trajectory-path-"]');
  await expect(paths).toHaveCount(1, { timeout: 15_000 });
  const points = (await paths.first().getAttribute("points"))?.trim().split(/\s+/) ?? [];
  expect(points.length).toBeGreaterThan(1);
  await expect(page.getByTestId("panel-health").getByText("在线").first()).toBeVisible();
  if (process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR) {
    await page.screenshot({ path: `${process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR}/ros2-swarm.png`, fullPage: true });
    await page.getByTestId("panel-trajectory").screenshot({
      path: `${process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR}/ros2-trajectory.png`
    });
    await trajectory.screenshot({
      path: `${process.env.MASAR_LIVE_ROS_SCREENSHOT_DIR}/ros2-trajectory-plot.png`
    });
  }
});
