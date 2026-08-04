import { afterEach, describe, expect, it, vi } from "vitest";
import { api, ApiError } from "../src/app/api";

describe("platform API errors", () => {
  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("turns an empty proxy 502 into a stable service-unavailable error", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue(new Response("", { status: 502 })));

    await expect(api.capabilities()).rejects.toEqual(
      new ApiError(502, "SERVICE_UNAVAILABLE", "控制面服务暂时不可用")
    );
  });
});
