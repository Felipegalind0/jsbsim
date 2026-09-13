#!/usr/bin/env node
import { readFile } from "node:fs/promises";
import path from "node:path";
const root = new URL("../", import.meta.url);

if (process.argv.length > 2) throw new Error("Metadata now describes the exact packed candidate; no live version/native overrides are accepted.");
console.log(await readFile(new URL("build/last-package.json", root), "utf8"));
