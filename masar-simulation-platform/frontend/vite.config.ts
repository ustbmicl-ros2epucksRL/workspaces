import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  server: {
    host: "127.0.0.1",
    port: 13000,
    strictPort: true,
    watch: {
      usePolling: true,
      interval: 500
    },
    proxy: {
      "/api": "http://127.0.0.1:18100",
      "/display": { target: "http://127.0.0.1:18090", ws: true },
      "/webots": { target: "http://127.0.0.1:18090", ws: true },
      "/video": "http://127.0.0.1:18090",
      "/ros1": { target: "http://127.0.0.1:18090", ws: true },
      "/ros2": { target: "http://127.0.0.1:18090", ws: true }
    }
  },
  build: {
    outDir: "dist",
    sourcemap: false
  }
});
