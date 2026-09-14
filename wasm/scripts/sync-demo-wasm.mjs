#!/usr/bin/env node
import { mkdir, mkdtemp, readFile, rename, rm } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { contentManifest, copyManifest, sha256, verifyContent } from "./build-system/source.mjs";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const selected = JSON.parse(await readFile(path.join(root, "build/last-build.json"), "utf8"));
const dist = path.join(selected.artifactRoot, "dist");
const metadata = JSON.parse(await readFile(path.join(dist, "build-metadata.json"), "utf8"));
const manifest = await contentManifest(dist);
const actual = Object.fromEntries(manifest.filter(file => file.path !== "build-metadata.json").map(file => [file.path, file.sha256]));
if (metadata.validation?.status !== "passed" || JSON.stringify(actual) !== JSON.stringify(metadata.files) ||
    sha256(JSON.stringify({ identity: metadata.identity, files: actual })) !== selected.artifactSha256) {
  throw new Error("Demo artifact failed identity verification.");
}
for (const file of ["index.js", "index.d.ts", "wasm/jsbsim_wasm.mjs", "wasm/jsbsim_wasm.wasm"]) {
  if (!metadata.files[file]) throw new Error("Missing required demo artifact: " + file);
}
const parent = path.join(root, "demo/public");
await mkdir(parent, { recursive: true });
const staging = await mkdtemp(path.join(parent, ".sdk-sync-"));
await copyManifest(dist, staging, manifest);
await verifyContent(staging, manifest);
const target = path.join(parent, "sdk");
await mkdir(path.join(root, "build/demo-history"), { recursive: true });
const history = await mkdtemp(path.join(root, "build/demo-history", "previous-"));
let previous = false;
try { await rename(target, path.join(history, "sdk")); previous = true; }
catch (error) { if (error.code !== "ENOENT") throw error; }
try { await rename(staging, target); }
catch (error) {
  if (previous) await rename(path.join(history, "sdk"), target);
  throw error;
}
if (!previous) await rm(history, { recursive: true });
console.log("Synced matching SDK, declarations, WASM and metadata " + selected.artifactSha256 + " to demo/public/sdk");
