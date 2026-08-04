import { expect, type Locator } from "@playwright/test";

export type FramebufferStats = {
  width: number;
  height: number;
  sampledPixels: number;
  quantizedColors: number;
  luminanceRange: number;
  nonDarkRatio: number;
};

export async function expectRenderedFramebuffer(
  canvas: Locator,
  name: string
): Promise<FramebufferStats> {
  let latest: FramebufferStats | null = null;
  await expect
    .poll(
      async () => {
        latest = await framebufferStats(canvas);
        return latest.quantizedColors;
      },
      { message: `${name} noVNC framebuffer remained blank`, timeout: 20_000 }
    )
    .toBeGreaterThan(24);
  expect(latest, `${name} framebuffer statistics were not sampled`).not.toBeNull();
  expect(latest!.width).toBeGreaterThanOrEqual(640);
  expect(latest!.height).toBeGreaterThanOrEqual(480);
  expect(latest!.luminanceRange).toBeGreaterThan(80);
  expect(latest!.nonDarkRatio).toBeGreaterThan(0.05);
  return latest!;
}

async function framebufferStats(canvas: Locator): Promise<FramebufferStats> {
  return canvas.evaluate((element: HTMLCanvasElement) => {
    const context = element.getContext("2d", { willReadFrequently: true });
    if (!context || element.width === 0 || element.height === 0) {
      return {
        width: element.width,
        height: element.height,
        sampledPixels: 0,
        quantizedColors: 0,
        luminanceRange: 0,
        nonDarkRatio: 0
      };
    }

    const pixels = context.getImageData(0, 0, element.width, element.height).data;
    const stepX = Math.max(1, Math.floor(element.width / 160));
    const stepY = Math.max(1, Math.floor(element.height / 90));
    const colors = new Set<number>();
    let sampledPixels = 0;
    let nonDarkPixels = 0;
    let minimumLuminance = 255;
    let maximumLuminance = 0;
    for (let y = 0; y < element.height; y += stepY) {
      for (let x = 0; x < element.width; x += stepX) {
        const offset = (y * element.width + x) * 4;
        const red = pixels[offset];
        const green = pixels[offset + 1];
        const blue = pixels[offset + 2];
        const luminance = Math.round(0.2126 * red + 0.7152 * green + 0.0722 * blue);
        colors.add(((red >> 4) << 8) | ((green >> 4) << 4) | (blue >> 4));
        minimumLuminance = Math.min(minimumLuminance, luminance);
        maximumLuminance = Math.max(maximumLuminance, luminance);
        if (luminance >= 48) nonDarkPixels += 1;
        sampledPixels += 1;
      }
    }

    return {
      width: element.width,
      height: element.height,
      sampledPixels,
      quantizedColors: colors.size,
      luminanceRange: maximumLuminance - minimumLuminance,
      nonDarkRatio: sampledPixels === 0 ? 0 : nonDarkPixels / sampledPixels
    };
  });
}
