#!/usr/bin/env node
import { readFile } from "node:fs/promises";
import { validateSourceLock } from "./build-system/source.mjs";
const packageJson = JSON.parse(await readFile(new URL("../package.json", import.meta.url), "utf8"));
const native = validateSourceLock(JSON.parse(await readFile(new URL("../jsbsim-source.lock.json", import.meta.url), "utf8")));
if (process.argv.length > 2) throw new Error("Implicit tag/version resolution is retired; update explicit source and package locks.");
console.log(JSON.stringify({ package: { name: packageJson.name, version: packageJson.version }, native }, null, 2));
