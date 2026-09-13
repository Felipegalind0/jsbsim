#!/usr/bin/env node
import { mkdtemp, readFile, rm } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { contentManifest, extractSourceArchive, manifestSha256, safeRelative, sha256, validateSourceLock, writeJson } from "./build-system/source.mjs";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const options = {};
for (const argument of process.argv.slice(2)) {
  const match = /^--(origin|commit|archive)=(.+)$/.exec(argument);
  if (!match) throw new Error("Expected explicit --origin= --commit= --archive= options.");
  if (options[match[1]]) throw new Error("Duplicate lock-update option.");
  options[match[1]] = match[2];
}
if (!options.origin || !options.commit || !options.archive) throw new Error("All lock-update options are required; no latest/ref fallback exists.");
const archivePath = safeRelative(options.archive);
const scratch = await mkdtemp(path.join(os.tmpdir(), "jsbsim-archive-review-"));
try {
  await extractSourceArchive(path.join(root, archivePath), scratch);
  const lock = validateSourceLock({ schemaVersion: 1, origin: options.origin, commit: options.commit,
    archive: { path: archivePath, sha256: sha256(await readFile(path.join(root, archivePath))) },
    contentSha256: manifestSha256(await contentManifest(scratch)) });
  await writeJson(path.join(root, "jsbsim-source.lock.json"), lock);
  console.log(JSON.stringify(lock, null, 2));
} finally { await rm(scratch, { recursive: true, force: true }); }
