# USTB Branding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the user-approved University of Science and Technology Beijing favicon and top-right school mark without replacing the existing MASAR `M` mark or changing platform behavior.

**Architecture:** Generate two transparent PNG assets from USTB's official public logo sources, link the favicon from `index.html`, and add the page mark as a small React portal mounted into the existing `.masthead`. Keep all new layout overrides in a dedicated stylesheet so the current dirty `App.tsx` and `layout.css` files are not overwritten or accidentally committed.

**Tech Stack:** React 19, TypeScript 5.7, Vite 6, CSS Grid, Playwright 1.62, ImageMagick 6, USTB official PNG assets.

## Global Constraints

- The browser tab uses the circular USTB seal as the favicon.
- The page's existing `M`, `MASAR`, and `群体机器人验证台` lockup remains unchanged.
- The top-right page mark uses the full seal plus Chinese and English university name with a transparent background.
- The masthead remains exactly 64px high.
- The control-plane status logic and copy remain unchanged; only its desktop position moves left.
- At widths of 1120px or less, hide the school mark and preserve the existing compact masthead behavior.
- Do not edit or commit unrelated existing worktree changes, especially `frontend/src/app/App.tsx`, `frontend/src/styles/layout.css`, and `frontend/tests/layout.spec.ts`.
- Do not add dependencies; use the existing Playwright stack and the system ImageMagick installation.

## File Structure

- Create `frontend/public/brand/ustb-favicon.png`: transparent circular favicon derived from the official seal.
- Create `frontend/public/brand/ustb-wordmark.png`: transparent stacked seal and bilingual university name.
- Modify `frontend/index.html`: register the PNG favicon.
- Create `frontend/src/app/SchoolBrandPortal.tsx`: mount the school mark into the masthead when the masthead exists.
- Create `frontend/src/styles/branding.css`: reserve the fourth desktop masthead column, style the mark, and hide it at narrow widths.
- Modify `frontend/src/main.tsx`: render the portal and import the dedicated branding stylesheet.
- Create `frontend/tests/branding.spec.ts`: cover favicon, unchanged MASAR mark, desktop positioning, and responsive overflow.

---

### Task 1: Generate Brand Assets and Register the Favicon

**Files:**
- Create: `frontend/public/brand/ustb-favicon.png`
- Create: `frontend/public/brand/ustb-wordmark.png`
- Modify: `frontend/index.html:4-7`
- Create: `frontend/tests/branding.spec.ts`

**Interfaces:**
- Consumes: USTB official sources `https://www.ustb.edu.cn/images/logo.png` and `https://www.ustb.edu.cn/images/f_logo1.png`.
- Produces: public URLs `/brand/ustb-favicon.png` and `/brand/ustb-wordmark.png` for the HTML head and React component.

- [ ] **Step 1: Write the failing favicon test**

Create `frontend/tests/branding.spec.ts` with:

```ts
import { expect, test } from "@playwright/test";
import { installMockApi } from "./mockApi";

test.beforeEach(async ({ page }) => {
  await installMockApi(page);
});

test("uses the USTB seal as favicon without replacing the MASAR mark", async ({ page }) => {
  await page.goto("/?workspace=group_behavior");

  const favicon = page.locator('link[rel="icon"]');
  await expect(favicon).toHaveAttribute("href", "/brand/ustb-favicon.png");
  await expect(favicon).toHaveAttribute("type", "image/png");

  const response = await page.request.get("/brand/ustb-favicon.png");
  expect(response.ok()).toBe(true);
  await expect(page.locator(".brand-mark")).toHaveText("M");
});
```

- [ ] **Step 2: Run the focused test and verify the expected failure**

Run:

```bash
cd frontend
npx playwright test tests/branding.spec.ts -g "uses the USTB seal"
```

Expected: FAIL because `frontend/index.html` has no `link[rel="icon"]` pointing to `/brand/ustb-favicon.png`.

- [ ] **Step 3: Generate the two transparent PNG assets from official sources**

Run from `frontend/`:

```bash
brand_tmp_dir="$(mktemp -d)"
mkdir -p public/brand
curl -L --fail --silent --show-error https://www.ustb.edu.cn/images/logo.png -o "$brand_tmp_dir/seal-source.png"
curl -L --fail --silent --show-error https://www.ustb.edu.cn/images/f_logo1.png -o "$brand_tmp_dir/name-source.png"
convert "$brand_tmp_dir/seal-source.png" -channel RGB -fill '#005bac' -colorize 100 "$brand_tmp_dir/seal-blue.png"
convert -size 256x256 xc:none -fill white -draw 'circle 128,128 128,8' "$brand_tmp_dir/favicon-disc.png"
convert "$brand_tmp_dir/seal-blue.png" -resize 232x232 "$brand_tmp_dir/favicon-seal.png"
convert "$brand_tmp_dir/favicon-disc.png" "$brand_tmp_dir/favicon-seal.png" -gravity center -composite -resize 128x128 public/brand/ustb-favicon.png
convert "$brand_tmp_dir/seal-blue.png" -resize 166x166 "$brand_tmp_dir/wordmark-seal.png"
convert "$brand_tmp_dir/wordmark-seal.png" "$brand_tmp_dir/name-source.png" -background none -gravity center -append -trim +repage public/brand/ustb-wordmark.png
```

Verify transparent corners and non-empty dimensions:

```bash
identify -format '%f %wx%h %[channels]\n' public/brand/ustb-favicon.png public/brand/ustb-wordmark.png
test "$(convert public/brand/ustb-favicon.png -format '%[fx:p{0,0}.a]' info:)" = "0"
test "$(convert public/brand/ustb-wordmark.png -format '%[fx:p{0,0}.a]' info:)" = "0"
```

Expected: both files report alpha channels; both corner-alpha checks exit 0.

- [ ] **Step 4: Register the favicon**

Add this line after the viewport metadata in `frontend/index.html`:

```html
<link rel="icon" type="image/png" href="/brand/ustb-favicon.png" />
```

Keep the existing title unchanged:

```html
<title>MASAR 群体机器人验证台</title>
```

- [ ] **Step 5: Run the focused test and verify it passes**

Run:

```bash
cd frontend
npx playwright test tests/branding.spec.ts -g "uses the USTB seal"
```

Expected: PASS; the favicon request returns HTTP 200 and `.brand-mark` still contains `M`.

- [ ] **Step 6: Commit only Task 1 files**

```bash
git add masar-simulation-platform/frontend/index.html \
  masar-simulation-platform/frontend/public/brand/ustb-favicon.png \
  masar-simulation-platform/frontend/public/brand/ustb-wordmark.png \
  masar-simulation-platform/frontend/tests/branding.spec.ts
git commit -m "feat: add USTB favicon assets"
```

---

### Task 2: Add the Top-Right School Brand and Move Status Left

**Files:**
- Create: `frontend/src/app/SchoolBrandPortal.tsx`
- Create: `frontend/src/styles/branding.css`
- Modify: `frontend/src/main.tsx:1-13`
- Modify: `frontend/tests/branding.spec.ts`

**Interfaces:**
- Consumes: `/brand/ustb-wordmark.png`, the existing `.masthead` host, `.system-presence`, and `.brand-mark` selectors.
- Produces: `SchoolBrandPortal(): ReactPortal | null`, `.school-brand`, and the four-column desktop masthead override.

- [ ] **Step 1: Add failing desktop and responsive layout tests**

Append to `frontend/tests/branding.spec.ts`:

```ts
test("places the USTB mark to the right of control-plane status on desktop", async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto("/?workspace=group_behavior");

  const masthead = await page.locator(".masthead").boundingBox();
  const status = await page.locator(".system-presence").boundingBox();
  const schoolMark = page.getByRole("img", { name: "北京科技大学" });
  await expect(schoolMark).toBeVisible();
  const schoolMarkBox = await schoolMark.boundingBox();

  expect(masthead).not.toBeNull();
  expect(status).not.toBeNull();
  expect(schoolMarkBox).not.toBeNull();
  expect(masthead!.height).toBe(64);
  expect(status!.x + status!.width).toBeLessThanOrEqual(schoolMarkBox!.x);
  expect(schoolMarkBox!.x + schoolMarkBox!.width).toBeLessThanOrEqual(
    masthead!.x + masthead!.width
  );
  expect(await page.evaluate(() => document.documentElement.scrollWidth)).toBeLessThanOrEqual(1440);
});

test("hides the USTB masthead mark at compact widths without removing the MASAR mark", async ({ page }) => {
  await page.setViewportSize({ width: 1120, height: 900 });
  await page.goto("/?workspace=group_behavior");

  await expect(page.getByRole("img", { name: "北京科技大学" })).toBeHidden();
  await expect(page.locator(".brand-mark")).toBeVisible();
  await expect(page.locator(".brand-mark")).toHaveText("M");
  expect(await page.evaluate(() => document.documentElement.scrollWidth)).toBeLessThanOrEqual(1120);
});
```

- [ ] **Step 2: Run the layout tests and verify the expected failure**

Run:

```bash
cd frontend
npx playwright test tests/branding.spec.ts -g "USTB mark"
```

Expected: FAIL because no accessible image named “北京科技大学” exists.

- [ ] **Step 3: Create the masthead portal component**

Create `frontend/src/app/SchoolBrandPortal.tsx`:

```tsx
import { useEffect, useState } from "react";
import { createPortal } from "react-dom";

export function SchoolBrandPortal() {
  const [masthead, setMasthead] = useState<HTMLElement | null>(null);

  useEffect(() => {
    const root = document.getElementById("root");
    const syncMasthead = () => {
      setMasthead(document.querySelector<HTMLElement>(".masthead"));
    };

    syncMasthead();
    if (!root) return;

    const observer = new MutationObserver(syncMasthead);
    observer.observe(root, { childList: true, subtree: true });
    return () => observer.disconnect();
  }, []);

  if (!masthead) return null;

  return createPortal(
    <div className="school-brand" aria-label="学校标识">
      <img src="/brand/ustb-wordmark.png" alt="北京科技大学" />
    </div>,
    masthead
  );
}
```

This observer is required because the existing `App` initially renders a loading screen before `.masthead` is mounted.

- [ ] **Step 4: Create the isolated branding stylesheet**

Create `frontend/src/styles/branding.css`:

```css
.masthead {
  grid-template-columns: 224px minmax(520px, 1fr) 156px 136px;
}

.system-presence {
  padding-right: 14px;
  padding-left: 14px;
}

.school-brand {
  display: flex;
  min-width: 0;
  align-items: center;
  justify-content: center;
  padding: 4px 12px;
  border-left: var(--hairline);
}

.school-brand img {
  display: block;
  width: 100%;
  max-width: 106px;
  max-height: 56px;
  object-fit: contain;
}

@media (max-width: 1120px) {
  .masthead {
    grid-template-columns: 188px minmax(480px, 1fr);
  }

  .school-brand {
    display: none;
  }
}

@media (max-width: 680px) {
  .masthead {
    grid-template-columns: 76px minmax(0, 1fr);
  }
}
```

- [ ] **Step 5: Render the portal and import its stylesheet**

Update `frontend/src/main.tsx` to:

```tsx
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./app/App";
import { SchoolBrandPortal } from "./app/SchoolBrandPortal";
import "./styles/tokens.css";
import "./styles/layout.css";
import "./styles/branding.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
    <SchoolBrandPortal />
  </StrictMode>
);
```

- [ ] **Step 6: Run the branding tests and verify they pass**

Run:

```bash
cd frontend
npx playwright test tests/branding.spec.ts
```

Expected: 3 tests passed, with no horizontal overflow at 1440px or 1120px.

- [ ] **Step 7: Run type and build verification**

Run:

```bash
cd frontend
npm run typecheck
npm run build
```

Expected: both commands exit 0 without TypeScript or Vite errors.

- [ ] **Step 8: Capture desktop and mobile visual evidence**

Run the app through Playwright and capture screenshots after the branding tests render:

```bash
cd frontend
npx playwright test tests/branding.spec.ts --trace on
```

Inspect the trace screenshots and confirm:

- the transparent blue mark is legible at 1440px;
- the status text sits immediately to its left without overlap;
- the masthead remains 64px high;
- the school mark is absent at 1120px while the `M` mark remains visible.

- [ ] **Step 9: Commit only Task 2 files**

```bash
git add masar-simulation-platform/frontend/src/app/SchoolBrandPortal.tsx \
  masar-simulation-platform/frontend/src/styles/branding.css \
  masar-simulation-platform/frontend/src/main.tsx \
  masar-simulation-platform/frontend/tests/branding.spec.ts
git commit -m "feat: show USTB branding in masthead"
```

---

### Task 3: Final Regression Check

**Files:**
- Verify: `frontend/index.html`
- Verify: `frontend/public/brand/ustb-favicon.png`
- Verify: `frontend/public/brand/ustb-wordmark.png`
- Verify: `frontend/src/app/SchoolBrandPortal.tsx`
- Verify: `frontend/src/styles/branding.css`
- Verify: `frontend/src/main.tsx`
- Verify: `frontend/tests/branding.spec.ts`

**Interfaces:**
- Consumes: all Task 1 and Task 2 outputs.
- Produces: fresh evidence that the focused feature and existing responsive layout remain valid.

- [ ] **Step 1: Run the focused branding and existing layout suites together**

```bash
cd frontend
npx playwright test tests/branding.spec.ts tests/layout.spec.ts
```

Expected: all tests pass with zero failures.

- [ ] **Step 2: Re-run the production checks from a clean command invocation**

```bash
cd frontend
npm run typecheck
npm run build
```

Expected: both commands exit 0.

- [ ] **Step 3: Verify the final diff is limited to the approved scope**

```bash
git status --short -- \
  masar-simulation-platform/frontend/index.html \
  masar-simulation-platform/frontend/public/brand \
  masar-simulation-platform/frontend/src/app/SchoolBrandPortal.tsx \
  masar-simulation-platform/frontend/src/styles/branding.css \
  masar-simulation-platform/frontend/src/main.tsx \
  masar-simulation-platform/frontend/tests/branding.spec.ts
git diff --check
```

Expected: no whitespace errors and no school-branding changes outside the listed paths.
