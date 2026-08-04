import { expect, test } from "@playwright/test";

test("workspace navigation cannot turn an offline control plane green", async ({ page }) => {
  await page.route("**/api/v1/**", (route) =>
    route.fulfill({ status: 502, contentType: "text/plain", body: "" })
  );
  await page.goto("/");

  await expect(page.getByText("控制面异常")).toBeVisible();
  await expect(page.getByRole("alert")).toContainText("控制面服务暂时不可用");
  await page.getByRole("button", { name: /探索组/ }).click();
  await expect(page.getByText("控制面异常")).toBeVisible();
});
