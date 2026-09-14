# Building the in-tree JSBSim WASM package

Develop engine code in the enclosing repository's `src/` and reusable bindings,
SDK, diagnostics and build tooling here in `wasm/`. They share one Git branch and
revision. The app installs a checked package; it does not compile C++ at startup.

## Build and package

Use Node, npm, CMake and Emscripten from `build-toolchain.lock.json`. From the
JSBSim repository root:

```sh
npm --prefix wasm ci
npm --prefix wasm run test:source
npm --prefix wasm run build
npm --prefix wasm run pack:build -- --release
```

The default build requires a clean repository. For uncommitted development:

```sh
npm --prefix wasm run build:dev
```

Development candidates retain the common dirty flag and actual content hashes;
release packing and stable app adoption reject them. `--allow-network` permits
npm to fill a missing dependency cache; builds otherwise install dependencies
from the existing offline cache. Source is always the enclosing checkout.
Neither command selects, downloads or patches another native source tree.

A build captures tracked and nonignored files from the enclosing Git repository
once, checking Git identity and file content before and after copying. The SDK
manifest is the `wasm/` subset of that same snapshot. Captures contain no `.git`
working directory. An attempt gets a private SDK workspace for dependency
installation, regenerated bindings, TypeScript and test output. CMake configures
the frozen enclosing root with `BUILD_WASM_MODULE=ON`; `wasm/CMakeLists.txt` links
the existing `libJSBSim`. Normal native builds default this option to OFF and do
not require Node or Emscripten.

The full pipeline generates bindings from the captured engine headers, builds
real WASM and TypeScript, typechecks and runs all SDK tests against that attempt's
runtime and native model fixtures. Generated bindings, CMake cache identity,
authored test/source files, compiler configuration and output bytes are checked
before promotion. Failed attempts never replace an accepted artifact.

`wasm/build/last-build.json` points to an immutable completed artifact under
`build/artifacts/`. `pack:build` verifies its recorded files and produces a tarball
and integrity report under `build/packages/`; `build/last-package.json` identifies
that tarball. `--release` requires clean source. Packing does not publish, tag,
commit, push or update an installed app. Existing outputs cannot be overwritten
with different bytes. `npm pack` on the editing directory is intentionally
blocked; use the identified package command.

## Identity and rechecks

Identity schema 2 records `build.mode: "in-tree"`, the common engine/SDK commit
and dirty flag, `sdk.path: "wasm"`, the complete repository content digest and
the SDK subtree digest. It also records the package identity, actual toolchain,
compiler configuration and build recipe hash. Metadata records distribution
and package-file hashes, generated binding inputs and passed checks. Provenance
names the common repository; external native archive and source-lock fields are
explicitly null. These are source and artifact identity guarantees, not a claim
of bit-identical compiler output across different machines.

```sh
npm --prefix wasm test
npm --prefix wasm run typecheck
npm --prefix wasm run bench:property-batch
```

Rechecks reject repository changes since the accepted build and reverify frozen
sources, generated bindings and runtime bytes. Successful software checks do not
establish aircraft calibration. Existing batching performance review and native
wheel dynamics review remain separate upstream work.

## Preservation and attribution

The complete SDK history through `61ee9486651bca36f4a9b2f927953d716985e391` was
imported without squashing at `d8a6453c082c133219f47e75ffc18ade71749575`.
Its imported tree is byte-identical to SDK tree
`1a441ec3d271e302959c8ec7aa23ce8f352f9f45`. The in-tree migration retains the SDK's
MIT notice and 0x62's authorship, alongside native JSBSim and third-party notices.
The package scope remains `@felipegalind0/jsbsim`; version `1.2.4-fork.6`
identifies this wrapper package, while the native version/revision is separate.

The old engine archive/lock, generated editor previews, old release binaries and
nested GitHub workflow files are retained as historical references. They are not
selected as the engine or distributed runtime. External-source update commands
and the detached CMake wrapper fail explicitly. Builds always regenerate their
own bindings from the enclosing engine. Root GitHub CI owns the new optional
WASM workflow; local demo sources and build commands remain. The compatibility
patch in `patches/` is provenance only and must not be reapplied; native
`doc/centralized-source-migration.md` maps its disposition.

The previous separate SDK and accepted `1.2.4-fork.1` app package remain the
migration reference until this in-tree package passes adoption checks. See the
app's in-tree integration execution record for the actual current artifact and
verification results. Hosted CI and upstream adoption are separate from local
acceptance.
