import assert from "node:assert/strict";
import { after, describe, it } from "node:test";
import { chmod, mkdtemp, mkdir, readFile, rm, symlink, writeFile } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { buildInputSha256, captureSource, contentManifest, extractSourceArchive, generatedBuildIdentity, installImmutableFile, manifestSha256, readDescriptor, resolveNativeSource,
  safeRelative, SDK_INPUT_ROOTS, sdkCaptureOptions, sha256, validateSourceLock, verifyCmakeCache, verifyContent, verifyGeneratedBindings, verifyResolvedSourceLock, verifyWorkspaceInputs, writeJson } from "../scripts/build-system/source.mjs";

import { checkToolchain, rejectAmbientCompilerOverrides } from "../scripts/build-system/toolchain.mjs";

const temporary = [];
async function directory() {
  const result = await mkdtemp(path.join(os.tmpdir(), "jsbsim-source-contract-"));
  temporary.push(result);
  return result;
}
after(async () => { for (const root of temporary) await rm(root, { recursive: true, force: true }); });

function archiveEntry(name, content = "fixture", type = "0") {
  const bytes = Buffer.from(content), header = Buffer.alloc(512);
  header.write(name, 0, 100); header.write("0000644\0", 100);
  header.write(bytes.length.toString(8).padStart(11, "0") + "\0", 124);
  header.write(type, 156); header.write("ustar\0", 257);
  header.fill(32, 148, 156);
  const sum = header.reduce((total, value) => total + value, 0);
  header.write(sum.toString(8).padStart(6, "0") + "\0 ", 148);
  return Buffer.concat([header, bytes, Buffer.alloc((512 - bytes.length % 512) % 512), Buffer.alloc(1024)]);
}
async function lockedFixture() {
  const root = await directory(), archive = path.join(root, "native.tar");
  await writeFile(archive, archiveEntry("source.cpp", "int fixture = 1;\n"));
  const extracted = path.join(root, "expected");
  await extractSourceArchive(archive, extracted);
  const lock = { schemaVersion: 1, origin: "https://github.com/Felipegalind0/jsbsim", commit: "a".repeat(40),
    archive: { path: "native.tar", sha256: sha256(await readFile(archive)) },
    contentSha256: manifestSha256(await contentManifest(extracted)) };
  await writeJson(path.join(root, "jsbsim-source.lock.json"), lock);
  return { root, archive, lock, cacheRoot: path.join(root, "cache") };
}

describe("resolved native source contract", () => {
  it("preserves byte and executable-mode identity while ignoring time metadata", async () => {
    const root = await directory();
    await writeFile(path.join(root, "b.cpp"), "b");
    await writeFile(path.join(root, "a.sh"), "a");
    const before = await contentManifest(root);
    assert.deepEqual(before.map(file => file.path), ["a.sh", "b.cpp"]);
    await chmod(path.join(root, "a.sh"), 0o755);
    const executable = await contentManifest(root);
    assert.notEqual(manifestSha256(before), manifestSha256(executable));
    assert.equal(executable[0].mode, 0o755);
    await writeFile(path.join(root, "b.cpp"), "changed");
    await assert.rejects(verifyContent(root, executable), /changed/);
  });

  it("rejects traversal and external-link archive entries before writing their targets", async () => {
    const root = await directory();
    for (const [name, type] of [["../outside", "0"], ["/outside", "0"], ["linked", "2"], ["hardlink", "1"]]) {
      const archive = path.join(root, "bad.tar");
      await writeFile(archive, archiveEntry(name, "target", type));
      await assert.rejects(extractSourceArchive(archive, path.join(root, "out-" + type + name.replaceAll("/", "x"))), /Unsafe|Unsupported/);
    }
    assert.throws(() => safeRelative("a/../escape"), /Unsafe/);
  });

  it("refuses uncaptured filesystem symlinks", async () => {
    const root = await directory();
    await symlink(os.tmpdir(), path.join(root, "outside"));
    await assert.rejects(contentManifest(root), /symlinks/);
  });

  it("resolves the pinned bytes without a repository or vendor checkout", async () => {
    const fixture = await lockedFixture();
    const source = await resolveNativeSource(fixture.root, { mode: "pinned", cacheRoot: fixture.cacheRoot });
    assert.equal(source.commit, fixture.lock.commit);
    assert.equal(source.contentSha256, fixture.lock.contentSha256);
    assert.equal(source.dirty, false);
    assert.equal(await readFile(path.join(source.root, "source.cpp"), "utf8"), "int fixture = 1;\n");
    assert.equal((await resolveNativeSource(fixture.root, { mode: "pinned", cacheRoot: fixture.cacheRoot })).root, source.root);
  });

  it("rejects corrupted archives and contaminated content-addressed caches", async () => {
    const fixture = await lockedFixture();
    const source = await resolveNativeSource(fixture.root, { mode: "pinned", cacheRoot: fixture.cacheRoot });
    await writeFile(path.join(source.root, "source.cpp"), "contamination");
    await assert.rejects(resolveNativeSource(fixture.root, { mode: "pinned", cacheRoot: fixture.cacheRoot }), /changed/);
    await writeFile(fixture.archive, "different archive");
    await assert.rejects(resolveNativeSource(fixture.root, { mode: "pinned", cacheRoot: fixture.cacheRoot }), /checksum/);
  });

  it("rejects a valid archive with the wrong extracted-source digest", async () => {
    const fixture = await lockedFixture();
    await writeJson(path.join(fixture.root, "jsbsim-source.lock.json"), { ...fixture.lock, contentSha256: "b".repeat(64) });
    await assert.rejects(resolveNativeSource(fixture.root, { mode: "pinned", cacheRoot: fixture.cacheRoot }), /Extracted source/);
  });

  it("does not fall back when local selection is missing or conflicts with pinned mode", async () => {
    const fixture = await lockedFixture();
    await assert.rejects(resolveNativeSource(fixture.root, { mode: "local", cacheRoot: fixture.cacheRoot }), /requires/);
    await assert.rejects(resolveNativeSource(fixture.root, { mode: "pinned", localSource: "missing", cacheRoot: fixture.cacheRoot }), /Conflicting/);
    await assert.rejects(resolveNativeSource(fixture.root, { mode: "local", localSource: "missing", cacheRoot: fixture.cacheRoot }));
  });

  it("rejects placeholder locks rather than inferring current HEAD or latest", async () => {
    const { lock } = await lockedFixture();
    assert.throws(() => validateSourceLock({ ...lock, commit: "0".repeat(40) }), /placeholder/);
    assert.throws(() => validateSourceLock({ ...lock, contentSha256: "0".repeat(64) }), /placeholder/);
    assert.throws(() => validateSourceLock({ ...lock, archive: { ...lock.archive, path: "../native.tar" } }), /Unsafe/);
  });

  it("captures exact local inputs and refuses concurrent identity changes", async () => {
    const root = await directory(), sourceRoot = path.join(root, "source");
    await mkdir(sourceRoot);
    await writeFile(path.join(sourceRoot, "new-untracked.cpp"), "source");
    const identity = { commit: "c".repeat(40), dirty: true, trackedDiffSha256: "d".repeat(64) };
    const captured = await captureSource(sourceRoot, path.join(root, "cache"), { identityReader: async () => identity });
    assert.equal(captured.dirty, true);
    assert.equal(await readFile(path.join(captured.root, "new-untracked.cpp"), "utf8"), "source");
    let changed = 0;
    await assert.rejects(captureSource(sourceRoot, path.join(root, "unstable"), {
      identityReader: async () => ({ ...identity, commit: String(++changed).padStart(40, "0") }),
    }), /changed while being captured/);
  });
});

async function descriptorFixture() {
  const root = await directory();
  const manifest = [{ path: "source.cpp", mode: 0o644, sha256: sha256("fixture") }];
  const digest = manifestSha256(manifest);
  const native = { root: path.join(root, "native"), origin: "https://github.com/Felipegalind0/jsbsim",
    mode: "local", commit: "a".repeat(40), contentSha256: digest, dirty: false, manifest };
  const sdk = { root: path.join(root, "sdk"), commit: "b".repeat(40), contentSha256: digest, dirty: true, manifest };
  const identity = { schemaVersion: 1, package: { name: "@fixture/jsbsim-wasm", version: "1.0.0" },
    native: { origin: native.origin, commit: native.commit, contentSha256: digest, dirty: false },
    sdk: { commit: sdk.commit, contentSha256: digest, dirty: true },
    build: { mode: "local", toolchain: { node: "fixture", npm: "fixture", emscripten: "fixture", cmake: "fixture",
      clang: "fixture", platform: "fixture", arch: "fixture", emscriptenConfigSha256: "f".repeat(64) },
      options: { buildType: "Release", cxxStandard: 17, sdkTarget: "es2022" } } };
  identity.build.inputSha256 = buildInputSha256(identity);
  const descriptor = { schemaVersion: 1, workspaceRoot: path.join(root, "workspace"), native, sdk, identity };
  const file = path.join(root, "descriptor.json");
  await writeJson(file, descriptor);
  return { root, file, descriptor };
}

describe("build consumers reject inconsistent resolved identities", () => {
  it("accepts a coherent descriptor and rejects native/SDK identity or manifest substitution", async () => {
    const { file, descriptor } = await descriptorFixture();
    assert.deepEqual(await readDescriptor(file), descriptor);
    const changes = [
      value => { value.native.commit = "c".repeat(40); },
      value => { value.native.contentSha256 = "d".repeat(64); },
      value => { value.native.dirty = true; },
      value => { value.sdk.commit = "c".repeat(40); },
      value => { value.sdk.manifest[0].sha256 = "e".repeat(64); },
      value => { value.native.manifest[0].sha256 = "e".repeat(64); },
      value => { value.native.origin = "https://github.com/another/jsbsim"; },
    ];
    for (const change of changes) {
      const invalid = structuredClone(descriptor);
      change(invalid);
      await writeJson(file, invalid);
      await assert.rejects(readDescriptor(file), /identity mismatch|manifest digest mismatch|origin mismatch/);
    }
  });

  it("recomputes the recipe and refuses stale options, malformed manifests and dirty pinned inputs", async () => {
    const { file, descriptor } = await descriptorFixture();
    const staleRecipe = structuredClone(descriptor);
    staleRecipe.identity.build.toolchain.clang = "different compiler";
    await writeJson(file, staleRecipe);
    await assert.rejects(readDescriptor(file), /recipe identity mismatch/);
    const duplicate = structuredClone(descriptor);
    duplicate.native.manifest.push(duplicate.native.manifest[0]);
    await writeJson(file, duplicate);
    await assert.rejects(readDescriptor(file), /unsorted source manifest/);
    const pinned = structuredClone(descriptor);
    pinned.native.mode = pinned.identity.build.mode = "pinned";
    pinned.identity.build.inputSha256 = buildInputSha256(pinned.identity);
    await writeJson(file, pinned);
    await assert.rejects(readDescriptor(file), /dirty pinned input/);
  });

  it("refuses CMake caches for a different source root or build recipe", async () => {
    const { descriptor } = await descriptorFixture();
    const cache = (root, input) => `JSBSIM_SOURCE_DIR:UNINITIALIZED=${root}\nJSBSIM_BUILD_INPUT_SHA256:UNINITIALIZED=${input}\n`;
    assert.doesNotThrow(() => verifyCmakeCache(descriptor, cache(descriptor.native.root, descriptor.identity.build.inputSha256)));
    assert.throws(() => verifyCmakeCache(descriptor, cache(path.join(descriptor.native.root, "other"), descriptor.identity.build.inputSha256)), /different resolved source/);
    assert.throws(() => verifyCmakeCache(descriptor, cache(descriptor.native.root, "f".repeat(64))), /different resolved source/);
    assert.throws(() => verifyCmakeCache(descriptor, ""), /different resolved source/);
  });

  it("requires matching binding source, recipe and all generated C++/TypeScript bytes", async () => {
    const { descriptor } = await descriptorFixture();
    const files = Object.fromEntries(["generated/FGFDMExecBindings.cpp", "src/generated/fgfdmexec-api.ts", "src/generated/jsbsim-api.ts"]
      .map(file => [file, sha256(file)]));
    for (const file of Object.keys(files)) {
      await mkdir(path.dirname(path.join(descriptor.workspaceRoot, file)), { recursive: true });
      await writeFile(path.join(descriptor.workspaceRoot, file), file);
    }
    const stampFile = path.join(descriptor.workspaceRoot, "generated/bindings-manifest.json");
    const stamp = { schemaVersion: 1, inputSha256: descriptor.identity.build.inputSha256,
      nativeContentSha256: descriptor.native.contentSha256, nativeCommit: descriptor.native.commit, files };
    await writeJson(stampFile, stamp);
    assert.deepEqual(await verifyGeneratedBindings(descriptor), stamp);
    for (const key of ["inputSha256", "nativeContentSha256", "nativeCommit"]) {
      await writeJson(stampFile, { ...stamp, [key]: "different" });
      await assert.rejects(verifyGeneratedBindings(descriptor), /different source/);
    }
    const incomplete = structuredClone(stamp);
    delete incomplete.files["src/generated/jsbsim-api.ts"];
    await writeJson(stampFile, incomplete);
    await assert.rejects(verifyGeneratedBindings(descriptor), /Incomplete/);
    await writeJson(stampFile, stamp);
    await writeFile(path.join(descriptor.workspaceRoot, "src/generated/jsbsim-api.ts"), "stale wrapper");
    await assert.rejects(verifyGeneratedBindings(descriptor), /Generated bindings changed/);
  });
});

it("accepts only reviewed Emscripten distribution banners and preserves actual compiler identity", async () => {
  const lock = JSON.parse(await readFile(new URL("../build-toolchain.lock.json", import.meta.url), "utf8"));
  const actual = { node: lock.node, npm: lock.npm, cmake: lock.cmake, platform: "linux", emscripten: "" };
  for (const profile of lock.emscripten.profiles) {
    actual.emscripten = profile.banner;
    assert.doesNotThrow(() => checkToolchain(actual, lock));
    assert.equal(actual.emscripten, profile.banner);
  }
  assert.throws(() => checkToolchain({ ...actual, emscripten: actual.emscripten + "-unknown" }, lock), /Unapproved/);
  assert.throws(() => checkToolchain({ ...actual, emscripten: actual.emscripten.replace("6.0.9", "6.0.8") }, lock), /Unapproved/);
  assert.throws(() => checkToolchain({ ...actual, npm: "different" }, lock), /Toolchain mismatch/);
});

it("reuses identical immutable output bytes and refuses replacement", async () => {
  const root = await directory(), input = path.join(root, "input"), target = path.join(root, "output");
  await writeFile(input, "accepted bytes");
  await installImmutableFile(input, target);
  await installImmutableFile(input, target);
  // Use a new inode: editing a hardlinked staging file would also edit its target.
  const changed = path.join(root, "changed");
  await writeFile(changed, "different bytes");
  await assert.rejects(installImmutableFile(changed, target), /different bytes/);
  assert.equal(await readFile(target, "utf8"), "accepted bytes");
});

it("rejects mutated frozen test/source inputs and generated identity before rechecking", async () => {
  const { descriptor } = await descriptorFixture();
  const directories = new Set(["src", "bindings", "scripts", "cmake", "test", "bench", "docs", "demo", "patches", ".github"]);
  for (const name of SDK_INPUT_ROOTS) {
    const target = path.join(descriptor.workspaceRoot, name);
    if (directories.has(name)) await mkdir(target, { recursive: true });
    else await writeFile(target, "fixture");
  }
  const testFile = path.join(descriptor.workspaceRoot, "test/regression.mjs");
  await writeFile(testFile, "original assertions");
  const identityFile = path.join(descriptor.workspaceRoot, "src/build-identity.ts");
  await writeFile(identityFile, "authored placeholder");
  descriptor.sdk.manifest = await contentManifest(descriptor.workspaceRoot, sdkCaptureOptions);
  await writeFile(identityFile, generatedBuildIdentity(descriptor.identity));
  await writeFile(path.join(descriptor.workspaceRoot, "demo/tsconfig.node.tsbuildinfo"), "generated compiler cache");
  await assert.doesNotReject(verifyWorkspaceInputs(descriptor));
  await writeFile(testFile, "weakened assertions");
  await assert.rejects(verifyWorkspaceInputs(descriptor), /authored workspace inputs changed/);
  await writeFile(testFile, "original assertions");
  await writeFile(identityFile, "a different build identity");
  await assert.rejects(verifyWorkspaceInputs(descriptor), /build identity changed/);
});

it("rejects source-lock changes between native resolution and SDK input capture", async () => {
  const fixture = await lockedFixture();
  const source = await resolveNativeSource(fixture.root, { mode: "pinned", cacheRoot: fixture.cacheRoot });
  assert.doesNotThrow(() => verifyResolvedSourceLock(source, fixture.lock));
  const changedLocks = [
    { ...fixture.lock, commit: "f".repeat(40) },
    { ...fixture.lock, contentSha256: "f".repeat(64) },
    { ...fixture.lock, archive: { ...fixture.lock.archive, sha256: "f".repeat(64) } },
    { ...fixture.lock, archive: { ...fixture.lock.archive, path: "another.tar.gz" } },
    { ...fixture.lock, origin: "https://github.com/another/jsbsim" },
  ];
  for (const lock of changedLocks) assert.throws(() => verifyResolvedSourceLock(source, lock), /frozen SDK source lock/);
});

it("refuses ambient flags or include paths that would escape the recorded build recipe", () => {
  assert.doesNotThrow(() => rejectAmbientCompilerOverrides({ EMSDK: "/official/emsdk", EM_CONFIG: "/official/emsdk/.emscripten" }));
  assert.throws(() => rejectAmbientCompilerOverrides({ EMSDK: "/official/emsdk", EM_CONFIG: "/different/config" }), /EM_CONFIG/);
  assert.doesNotThrow(() => rejectAmbientCompilerOverrides({ CFLAGS: "", CPATH: "  ", EM_CACHE: "/cache", PATH: "/tools" }));
  for (const key of ["CFLAGS", "CXXFLAGS", "CPPFLAGS", "LDFLAGS", "EMCC_CFLAGS", "EMMAKEN_CFLAGS", "CPATH", "CPLUS_INCLUDE_PATH", "LIBRARY_PATH", "EM_CONFIG", "CMAKE_TOOLCHAIN_FILE"]) {
    assert.throws(() => rejectAmbientCompilerOverrides({ [key]: "override" }), /Compiler-affecting environment/);
  }
});
