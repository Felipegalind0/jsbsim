#!/usr/bin/env node
import path from "node:path";
import { fileURLToPath } from "node:url";
import { captureRepository } from "./build-system/source.mjs";
const sdkRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
if (process.argv.slice(2).some(arg => arg !== "--allow-dirty")) throw new Error("Only --allow-dirty is supported; the enclosing repository supplies both engine and SDK.");
if (process.env.JSBSIM_SOURCE_DIR || process.env.JSBSIM_SOURCE_ROOT || process.env.JSBSIM_BUILD_DESCRIPTOR) throw new Error("External source overrides cannot select an in-tree build.");
console.log(JSON.stringify(await captureRepository(sdkRoot, { allowDirty: process.argv.includes("--allow-dirty"), cacheRoot: path.join(sdkRoot, "build/sources") }), null, 2));
