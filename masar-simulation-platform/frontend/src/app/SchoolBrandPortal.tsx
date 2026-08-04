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
