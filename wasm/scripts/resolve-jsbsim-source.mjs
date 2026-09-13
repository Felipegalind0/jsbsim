#!/usr/bin/env node
import path from "node:path";
import { fileURLToPath } from "node:url";
import { resolveNativeSource } from "./build-system/source.mjs";

const sdkRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
let mode = "pinned", localSource = null;
for (const argument of process.argv.slice(2)) {
  if (argument.startsWith("--jsbsim-source=")) { localSource = argument.slice(16); mode = "local"; }
  else throw new Error("Unknown source option: " + argument);
}
if (process.env.JSBSIM_SOURCE_DIR || process.env.JSBSIM_SOURCE_ROOT || process.env.JSBSIM_BUILD_DESCRIPTOR) {
  throw new Error("Conflicting legacy source environment; use explicit source options.");
}
const source = await resolveNativeSource(sdkRoot, { mode, localSource, cacheRoot: path.join(sdkRoot, "build", "sources") });
console.log(JSON.stringify(source, null, 2));
