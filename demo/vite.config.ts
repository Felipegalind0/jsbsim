import path from "node:path";
import { createHash } from "node:crypto";
import { readFileSync, readdirSync } from "node:fs";
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react-swc";

function getBasePath(): string {
  const value = process.env.DEMO_BASE_PATH;
  if (!value || value === "/") {
    return "/";
  }

  return value.endsWith("/") ? value : `${value}/`;
}

const sdkRoot = path.resolve(__dirname, "public/sdk");
const metadata = JSON.parse(readFileSync(path.join(sdkRoot, "build-metadata.json"), "utf8"));
if (metadata.validation?.status !== "passed") throw new Error("Sync a checked SDK artifact before building the demo.");
const actual: Record<string, string> = {};
function inspect(relative = ""): void {
  for (const entry of readdirSync(path.join(sdkRoot, relative), { withFileTypes: true })) {
    const name = relative ? `${relative}/${entry.name}` : entry.name;
    if (entry.isDirectory()) inspect(name);
    else if (entry.isFile() && name !== "build-metadata.json") {
      actual[name] = createHash("sha256").update(readFileSync(path.join(sdkRoot, name))).digest("hex");
    } else if (!entry.isFile()) throw new Error("Unexpected demo SDK source entry: " + name);
  }
}
inspect();
if (JSON.stringify(Object.entries(actual).sort()) !== JSON.stringify(Object.entries(metadata.files).sort())) {
  throw new Error("Synced demo SDK artifact changed; synchronize a verified candidate again.");
}

export default defineConfig({
  base: getBasePath(),
  plugins: [react()],
  resolve: {
    alias: {
      "@sdk": path.join(sdkRoot, "index.js")
    }
  },
  server: {
    fs: {
      allow: [path.resolve(__dirname, "..")]
    }
  }
});
