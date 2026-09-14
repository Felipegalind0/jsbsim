#!/usr/bin/env node
import { readFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { contentManifest } from "./build-system/source.mjs";

// Metadata belongs to a completed frozen build. Never infer it from live Git.
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const candidate = JSON.parse(await readFile(path.join(root, "build/last-build.json"), "utf8"));
const metadata = JSON.parse(await readFile(path.join(candidate.artifactRoot, "dist/build-metadata.json"), "utf8"));
const actual = Object.fromEntries((await contentManifest(path.join(candidate.artifactRoot, "dist")))
  .filter(file => file.path !== "build-metadata.json").map(file => [file.path, file.sha256]));
if (JSON.stringify(actual) !== JSON.stringify(metadata.files)) throw new Error("Artifact content differs from build metadata.");
console.log(JSON.stringify(metadata, null, 2));
