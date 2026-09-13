# Canonical native source and identified SDK builds

Native edits belong in `Felipegalind0/jsbsim`; SDK bindings, ownership and build tooling belong here. `0sfs` consumes the SDK and FOSS Earth separately. Build snapshots contain no Git working directories and are not additional editing locations.

The SDK package is `@felipegalind0/jsbsim-wasm`, initially `1.2.4-fork.1`. This identifies our fork; it does not assert that its native engine is JSBSim 1.2.4. Read the native revision in `buildIdentity`. No npm publication is implied by this local package name.

## Source selection

`jsbsim-source.lock.json` identifies our native fork, a full integration commit, a repository-relative gzip Git archive under `sources/`, its SHA-256, and the extracted-content digest. Updating that lock is explicit; normal builds never fetch a branch, update a submodule, change package versions or apply patches to a checkout.

```sh
# Install exactly the SDK's locked build dependencies into the npm cache.
npm ci

# Default/CI: clean SDK revision plus the verified locked native archive.
npm run build

# Development: explicitly capture the editable canonical native checkout.
npm run build:local -- --jsbsim-source=../jsbsim
```

The tool versions in `build-toolchain.lock.json` must match. Emscripten has two explicit accepted distribution banners: Homebrew’s tagged 6.0.9 source retains `-git`, while the [official 6.0.9 release packager](https://chromium.googlesource.com/emscripten-releases/+/f04ea239d533260dd1db760dd2d668d5f9a88d6b/src/build.py) validates and removes that suffix. The lock records source evidence and hashes for each; arbitrary suffixes are rejected. This accepts two reviewed distributions, without claiming identical LLVM binaries. The generated metadata also records the LLVM compiler, platform, architecture and actual Emscripten configuration SHA-256. The standard `EM_CONFIG=$EMSDK/.emscripten` selection emitted by [setup-emsdk](https://github.com/mymindstorm/setup-emsdk/blob/v14/src/main.ts) is permitted; arbitrary override paths are rejected. The build records config bytes privately in its attempt, verifies their digest after execution, and includes the digest in its recipe. Each frozen workspace installs locked dependencies with `npm ci --offline --ignore-scripts`; run the initial `npm ci` first. An explicit `--allow-network` build option permits fetching missing package-cache entries. It never changes native source selection.

Pinned builds reject dirty SDK sources. Local mode permits preserved dirty work and marks it as development input. Local native capture includes tracked and nonignored untracked files; ignored virtual environments and build caches are not scanned. Add any newly required ignored source deliberately to the source-input contract before relying on it.

Legacy environment source overrides and compiler-affecting flags/include/library overrides are rejected at the top-level entry point. Frozen SDK lock identity is reconciled with the resolved native descriptor before generation, so a source-lock change during capture cannot misattribute the compiled source. `JSBSIM_BUILD_DESCRIPTOR` and `JSBSIM_SOURCE_ROOT` are internal child-process inputs emitted by the resolver. Missing or conflicting explicit inputs fail rather than switching to another checkout.

## Capture and build boundaries

The resolver captures native and SDK content into `build/sources/`. It compares before/after manifests and Git identity around each copy and verifies the copied bytes. Manifests use sorted POSIX paths, executable versus nonexecutable modes, and SHA-256 hashes; timestamps do not identify source content. Archive extraction accepts regular files/directories and rejects traversal, links and special entries. Existing cache contents are rechecked before use.

The pipeline copies captured SDK inputs into a unique `build/attempts/` workspace. Generated C++ and TypeScript stay in that workspace. The binding generator uses `em++` and the same captured native headers that CMake compiles. A generated-input manifest ties bindings to the source digest; the build refuses mismatched bindings or CMake cache identities. Native CI version attribution receives the native commit, while SDK provenance retains the separately captured SDK commit.

SDK authored inputs and captured native inputs are checked again after compilation and tests. The SDK distribution, declarations, maps, native notices and metadata are assembled together. A failed compile or test leaves an inspectable attempt and does not update the completed-candidate pointer.

## Checked candidates and packaging

Successful builds print an immutable `build/artifacts/<artifact-digest>/` package root and atomically update `build/last-build.json`. Canonical `dist/` is not overwritten or used as the source of a new package. Historical outputs remain recoverable until explicitly reviewed for cleanup.

```sh
npm run pack:build
# A clean pinned candidate only; this still performs no publication or Git action.
npm run pack:build -- --release
```

Packages and their external integrity records go under `build/packages/`; `build/last-package.json` identifies the produced tarball. Install that exact tarball for local application acceptance. Packaging checks all recorded dist files and package metadata before packing. Existing tarballs and integrity reports are reused only when their bytes match; they are never overwritten by another build or a different release flag. Direct canonical `npm pack` is blocked to avoid relabeling an old `dist` as a new fork package.

The `release` command means clean, pinned local packing. Publication, tags, pushes and PRs are separate authorized actions. Old flags that selected latest upstream tags, allowed dirty release state or automatically incremented versions have been removed.

## Build identity

```ts
import { buildIdentity } from "@felipegalind0/jsbsim-wasm";
```

The exported identity contains schema version, package identity, native origin/commit/content digest/dirty state, SDK commit/content digest/dirty state, build mode/input digest, toolchain and options. It contains no filesystem paths or final SDK-entry hash.

`@felipegalind0/jsbsim-wasm/build-metadata` exports JSON with that same identity, every relative dist-file SHA-256 except the manifest itself, package-file hashes, source/dependency/toolchain lock hashes, actual archive provenance when used, the generated-binding manifest and completed-check records. This separation avoids a self-referential SDK-entry hash. Final tarball integrity is recorded externally.

Source hashes establish identity, not recoverability or aircraft fidelity. Preserve actual uncommitted source content. Native/compiler timestamps can affect binary bytes; builds record exact distributed bytes and do not claim untested bit-for-bit reproducibility.

## Checks and fixture routing

```sh
# Pure source-contract checks; no native build needed.
npm run test:source

# Recheck the last candidate against its exact frozen inputs.
npm test
npm run typecheck
npm run build:metadata
```

The full build itself runs typechecking and all SDK tests. Real-WASM tests and benchmarks obtain native aircraft/engine/system fixtures from the resolved descriptor, never from vendor. The test wrapper rechecks authored inputs in the frozen workspace, exact generated identity and binding hashes, and rejects changed SDK inputs, local native HEAD/content, or artifact bytes instead of silently testing an older distribution.

Current preservation work includes batching/contacts, lifetime cleanup, bounded diagnostics, native turbine initialization, wheel dynamics and portability. Existing PRs remain separate from local acceptance. The first coherent migration build exposed real model-replacement/destruction failures. Those failed attempts are retained; a later artifact is accepted only after the full checks pass against its own recorded inputs. See the owning lifecycle and native migration records for current results.

## Native lock updates and portability

After preserving and testing an integration revision, create its Git archive and run:

```sh
npm run update:jsbsim -- \
  --origin=https://github.com/Felipegalind0/jsbsim \
  --commit=FULL_NATIVE_COMMIT \
  --archive=sources/jsbsim-FULL_NATIVE_COMMIT.tar.gz
```

This explicit command validates/extracts the archive and writes only the source lock. It does not choose a branch, edit native source, bump a package version or commit changes. Review and retain the archive and lock together.

The old compatibility patch is retained as provenance. Native `doc/centralized-source-migration.md` maps each hunk to its current implementation, including the logging behavior that moved from `FGJSBBase` to `FGLogConsole`. Applying vendor patches is retired. Vendor Git wiring is retired separately after preservation; physical vendor files remain until the final clean pinned build satisfies the retirement gate.

## CI, demo and cleanup

The former updater workflow now verifies the committed native archive and builds a checked package. It has read-only repository permissions and does not create dependency PRs or publish. Demo deployment is manual; its build uses the same locked pipeline. Demo synchronization verifies the selected artifact and stages its complete SDK JavaScript, declarations, WASM and metadata as one directory under `demo/public/sdk`. The Vite and TypeScript aliases use that directory, and Vite checks its hashes. Replaced demo distributions remain under `build/demo-history`; historical `demo/public/wasm` is unused. Demo authored inputs are included in the SDK snapshot, with generated synchronized output excluded.

Do not delete source captures or accepted artifacts needed by a live consumer, validation report or rollback record. There is no blanket clean command. Remove an individually reviewed failed attempt only after retaining any unique diagnostic evidence. Repository/worktree cleanup and upstream contribution follow the centralization and contribution policies recorded in `0sfs/docs/`.

The wrapper's MIT notice and native JSBSim LGPL notices remain separate and are included in the package. Existing authorship and patch provenance are retained.

The CI workflow has not been run during this local migration. The explicit emsdk profile resolves the known distribution-banner difference; actual hosted-CI execution remains a separate recorded result. Historical upstream release tarballs and metadata are retained under `release/`, with the old metadata explicitly under `release/history/`; they are never selected as current.

Canonical `generated/FGFDMExecBindings.cpp` and `src/generated/*` are retained historical editor previews. They are not authoritative for a current native revision. The supported pipeline regenerates all three files in its frozen workspace and verifies their source stamp; package output never consumes the canonical generated C++/TypeScript preview. Canonical `src/build-identity.ts` deliberately rejects runtime use outside this pipeline.
