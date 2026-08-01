import { defineConfig, devices } from "@playwright/test";

process.env.NO_PROXY = "127.0.0.1,localhost,::1";
process.env.no_proxy = "127.0.0.1,localhost,::1";

export default defineConfig({
  testDir: "./tests",
  testIgnore: ["**/*.unit.spec.ts"],
  fullyParallel: false,
  timeout: 30_000,
  expect: { timeout: 5_000 },
  use: {
    baseURL: "http://127.0.0.1:13000",
    trace: "retain-on-failure",
    screenshot: "only-on-failure"
  },
  projects: [
    {
      name: "chromium",
      use: {
        ...devices["Desktop Chrome"],
        launchOptions: { executablePath: process.env.PLAYWRIGHT_CHROMIUM_PATH }
      }
    }
  ],
  webServer: {
    command: "npm run dev -- --host 127.0.0.1",
    url: "http://127.0.0.1:13000",
    reuseExistingServer: false,
    timeout: 30_000
  }
});
