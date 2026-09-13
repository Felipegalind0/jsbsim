#!/usr/bin/env node
import { spawnSync } from "node:child_process";
import { copyFile, mkdir, mkdtemp, readFile, readdir, realpath, rename, stat, writeFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { buildInputSha256, captureSource, contentManifest, copyManifest, generatedBuildIdentity, manifestSha256, resolveNativeSource,
  SDK_INPUT_ROOTS, sdkCaptureOptions, sha256, verifyContent, verifyGeneratedBindings, verifyResolvedSourceLock, verifyWorkspaceInputs, writeJson } from "./build-system/source.mjs";

import { checkToolchain, rejectAmbientCompilerOverrides } from "./build-system/toolchain.mjs";

const sdkRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");

function argumentsFor(argv) {
  rejectAmbientCompilerOverrides(process.env);
  const options = { mode: "pinned", localSource: null, allowNetwork: false, prepareOnly: false };
  for (const arg of argv) {
    if (arg.startsWith("--mode=")) options.mode = arg.slice(7);
    else if (arg.startsWith("--jsbsim-source=")) options.localSource = arg.slice(16);
    else if (arg === "--allow-network") options.allowNetwork = true;
    else if (arg === "--prepare-only") options.prepareOnly = true;
    else throw new Error("Unknown build option: " + arg);
  }
  if (process.env.JSBSIM_SOURCE_DIR || process.env.JSBSIM_SOURCE_ROOT || process.env.JSBSIM_BUILD_DESCRIPTOR) {
    throw new Error("Use explicit build CLI source selection; legacy source/descriptor environment overrides cannot choose top-level build inputs.");
  }
  if (!["pinned", "local"].includes(options.mode) || (options.mode === "local") !== Boolean(options.localSource)) {
    throw new Error("Use npm run build for the locked archive, or npm run build:local -- --jsbsim-source=../jsbsim.");
  }
  return options;
}

function command(executable, args, options = {}) {
  const result = spawnSync(executable, args, { stdio: "inherit", ...options });
  if (result.error) throw result.error;
  if (result.status !== 0) throw new Error(executable + " " + args.join(" ") + " failed with " + (result.signal ?? result.status));
  return result;
}
function version(executable, args = ["--version"]) {
  const result = command(executable, args, { encoding: "utf8", stdio: ["ignore", "pipe", "pipe"] });
  return (result.stdout + result.stderr).trim();
}
async function toolchain() {
  const emscripten = version("em++").split("\n")[0];
  const verbose = version("em++", ["-v"]);
  const configPath = version("em-config", ["EM_CONFIG"]);
  const configBytes = await readFile(configPath);
  return { configuration: { path: configPath, sha256: sha256(configBytes), bytes: configBytes }, identity: { node: process.version, npm: version("npm"), emscripten,
    cmake: version("cmake").split("\n")[0],
    clang: verbose.split("\n").find(line => /clang version/.test(line)) ?? emscripten,
    platform: process.platform, arch: process.arch, emscriptenConfigSha256: sha256(configBytes) } };
}

function publicNative(source) {
  return { origin: source.origin, commit: source.commit, contentSha256: source.contentSha256, dirty: source.dirty };
}
async function createDescriptor(options) {
  const cache = path.join(sdkRoot, "build", "sources");
  const native = await resolveNativeSource(sdkRoot, { mode: options.mode, localSource: options.localSource, cacheRoot: cache });
  const sdk = await captureSource(sdkRoot, path.join(cache, "sdk"), sdkCaptureOptions);
  if (options.mode === "pinned" && sdk.dirty) throw new Error("Pinned builds require a clean SDK revision. Use explicit build:local for preserved development inputs.");
  verifyResolvedSourceLock(native, JSON.parse(await readFile(path.join(sdk.root, "jsbsim-source.lock.json"), "utf8")));
  const packageJson = JSON.parse(await readFile(path.join(sdk.root, "package.json"), "utf8"));
  const lock = JSON.parse(await readFile(path.join(sdk.root, "package-lock.json"), "utf8"));
  if (lock.name !== packageJson.name || lock.version !== packageJson.version ||
      lock.packages?.[""]?.name !== packageJson.name || lock.packages?.[""]?.version !== packageJson.version) {
    throw new Error("SDK package and package-lock identities disagree.");
  }
  const measuredToolchain = await toolchain();
  const tools = measuredToolchain.identity;
  checkToolchain(tools, JSON.parse(await readFile(path.join(sdk.root, "build-toolchain.lock.json"), "utf8")));
  const recipe = { package: { name: packageJson.name, version: packageJson.version }, native: publicNative(native),
    sdk: { commit: sdk.commit, contentSha256: sdk.contentSha256, dirty: sdk.dirty },
    mode: options.mode, toolchain: tools, options: { buildType: "Release", cxxStandard: 17, sdkTarget: "es2022" } };
  const inputSha256 = buildInputSha256({ ...recipe, build: { mode: recipe.mode, toolchain: recipe.toolchain, options: recipe.options } });
  const identity = { schemaVersion: 1, package: recipe.package, native: recipe.native, sdk: recipe.sdk,
    build: { mode: recipe.mode, inputSha256, toolchain: recipe.toolchain, options: recipe.options } };
  await mkdir(path.join(sdkRoot, "build", "attempts"), { recursive: true });
  const attemptRoot = await mkdtemp(path.join(sdkRoot, "build", "attempts", inputSha256.slice(0, 12) + "-"));
  await writeFile(path.join(attemptRoot, "emscripten-config.py"), measuredToolchain.configuration.bytes);
  const toolchainConfiguration = { path: measuredToolchain.configuration.path, sha256: measuredToolchain.configuration.sha256 };
  const workspaceRoot = path.join(attemptRoot, "sdk");
  await copyManifest(sdk.root, workspaceRoot, sdk.manifest);
  const descriptor = { schemaVersion: 1, identity, native, sdk, attemptRoot, workspaceRoot, toolchainConfiguration,
    wasmBuildRoot: path.join(attemptRoot, "wasm"), distRoot: path.join(workspaceRoot, "dist") };
  await writeJson(path.join(attemptRoot, "descriptor.json"), descriptor);
  return descriptor;
}

async function verifyWorkspace(descriptor) {
  if (sha256(await readFile(descriptor.toolchainConfiguration.path)) !== descriptor.toolchainConfiguration.sha256) {
    throw new Error("Emscripten configuration changed during the build.");
  }
  await verifyWorkspaceInputs(descriptor);
  await verifyGeneratedBindings(descriptor);
  await verifyContent(descriptor.native.root, descriptor.native.manifest);
  await verifyContent(descriptor.sdk.root, descriptor.sdk.manifest);
}

async function bundleNativeNotices(descriptor) {
  const notices = [];
  for (const file of descriptor.native.manifest) {
    if (/^(?:COPYING|LICENSE|AUTHORS|NOTICE)(?:[._-].*)?$/i.test(file.path)) notices.push(file);
  }
  if (!notices.length) throw new Error("No native license notices found in selected source.");
  for (const file of notices) {
    const destination = path.join(descriptor.distRoot, "licenses", "jsbsim", file.path);
    await mkdir(path.dirname(destination), { recursive: true });
    await copyFile(path.join(descriptor.native.root, file.path), destination);
  }
  await mkdir(path.join(descriptor.distRoot, "licenses"), { recursive: true });
  await copyFile(path.join(descriptor.workspaceRoot, "LICENSE"), path.join(descriptor.distRoot, "licenses", "sdk-LICENSE"));
  await writeFile(path.join(descriptor.distRoot, "licenses", "README.txt"),
    "The SDK wrapper retains its MIT notice. The bundled JSBSim runtime retains its native LGPL notices.\n" +
    "Native origin: " + descriptor.identity.native.origin + "\nNative commit: " + descriptor.identity.native.commit + "\n" +
    "Native content SHA-256: " + descriptor.identity.native.contentSha256 + "\n");
}

async function finalize(descriptor, commands) {
  await verifyWorkspace(descriptor);
  await bundleNativeNotices(descriptor);
  const files = Object.fromEntries((await contentManifest(descriptor.distRoot)).map(file => [file.path, file.sha256]));
  const bindings = JSON.parse(await readFile(path.join(descriptor.workspaceRoot, "generated/bindings-manifest.json"), "utf8"));
  const inputHash = name => descriptor.sdk.manifest.find(file => file.path === name)?.sha256;
  const sourceLock = JSON.parse(await readFile(path.join(descriptor.sdk.root, "jsbsim-source.lock.json"), "utf8"));
  const packageFiles = Object.fromEntries(await Promise.all(["package.json", "LICENSE", "README.md"].map(async name =>
    [name, sha256(await readFile(path.join(descriptor.workspaceRoot, name)))])));
  const metadata = { schemaVersion: 1, identity: descriptor.identity, files, packageFiles,
    provenance: {
      nativeArchive: descriptor.native.archiveSha256 ? { path: sourceLock.archive.path, sha256: descriptor.native.archiveSha256 } : null,
      nativeSourceLockSha256: inputHash("jsbsim-source.lock.json"),
      sdkDependencyLockSha256: inputHash("package-lock.json"),
      toolchainLockSha256: inputHash("build-toolchain.lock.json"),
      generatedBindings: bindings,
    },
    validation: { status: "passed", commands } };
  await writeJson(path.join(descriptor.distRoot, "build-metadata.json"), metadata);
  const artifactSha256 = sha256(JSON.stringify({ identity: descriptor.identity, files }));
  const artifactRoot = path.join(sdkRoot, "build", "artifacts", artifactSha256);
  const staging = await mkdtemp(path.join(sdkRoot, "build", ".artifact-"));
  await copyManifest(descriptor.distRoot, path.join(staging, "dist"), await contentManifest(descriptor.distRoot));
  for (const name of ["package.json", "LICENSE", "README.md"]) {
    await copyFile(path.join(descriptor.workspaceRoot, name), path.join(staging, name));
  }
  await mkdir(path.dirname(artifactRoot), { recursive: true });
  try { await rename(staging, artifactRoot); }
  catch (error) {
    if (!["EEXIST", "ENOTEMPTY"].includes(error.code)) throw error;
    const expected = await contentManifest(staging);
    await verifyContent(artifactRoot, expected);
    await (await import("node:fs/promises")).rm(staging, { recursive: true, force: true });
  }
  const result = { schemaVersion: 1, artifactSha256, artifactRoot,
    descriptor: path.join(descriptor.attemptRoot, "descriptor.json"), identity: descriptor.identity };
  // A pointer identifies a completed candidate; it never rewrites live dist.
  const pointer = path.join(sdkRoot, "build", "last-build.json");
  const pointerStaging = pointer + "." + process.pid;
  await writeJson(pointerStaging, result);
  await rename(pointerStaging, pointer);
  console.log(JSON.stringify(result, null, 2));
  return result;
}

async function build(options) {
  const descriptor = await createDescriptor(options);
  const descriptorFile = path.join(descriptor.attemptRoot, "descriptor.json");
  if (options.prepareOnly) { console.log(descriptorFile); return; }
  const env = { ...process.env, JSBSIM_BUILD_DESCRIPTOR: descriptorFile,
    JSBSIM_SOURCE_ROOT: descriptor.native.root, GITHUB_SHA: descriptor.identity.native.commit,
    TRAVIS_COMMIT: descriptor.identity.native.commit, APPVEYOR_REPO_COMMIT: descriptor.identity.native.commit };
  const cwd = descriptor.workspaceRoot;
  const commands = [];
  const run = (exe, args) => command(exe, args, { cwd, env });
  run("npm", ["ci", "--ignore-scripts", "--no-audit", "--fund=false", ...(options.allowNetwork ? [] : ["--offline"])]);
  await writeFile(path.join(cwd, "src/build-identity.ts"), generatedBuildIdentity(descriptor.identity));
  run(process.execPath, ["scripts/generate-fgfdmexec-bindings.mjs"]);
  run("bash", ["scripts/build-wasm.sh"]);
  run(path.join(cwd, "node_modules", ".bin", "tsup"), ["--config", "tsup.config.ts"]);
  run(path.join(cwd, "node_modules", ".bin", "tsc"), ["--noEmit"]);
  commands.push("tsc --noEmit (frozen SDK workspace)");
  run(process.execPath, ["--test", "test/"]);
  commands.push("node --test test/ (frozen SDK workspace and resolved native fixtures)");
  await finalize(descriptor, commands);
}

try { await build(argumentsFor(process.argv.slice(2))); }
catch (error) { console.error(error.stack ?? error); process.exitCode = 1; }
