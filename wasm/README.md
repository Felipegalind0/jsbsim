# JSBSim WASM Tooling + TypeScript SDK

This directory builds its enclosing [JSBSim repository](https://github.com/Felipegalind0/jsbsim) to WebAssembly for Node.js and browsers, and ships a TypeScript SDK for loading and interacting with `FGFDMExec`. Engine and SDK come from one captured repository revision. Native JSBSim originates with [JSBSim-Team/jsbsim](https://github.com/JSBSim-Team/jsbsim); this wrapper preserves the upstream SDK authorship.

> [!WARNING]  
> This toolkit is still in early development and may contain bugs or unexpected behavior.

**Upstream SDK demo: https://0x62.github.io/jsbsim-wasm/** (not this fork’s identified artifact)

## Highlights

- Engine, bindings and SDK use the same Git revision and immutable source snapshot. The old source lock/archive are historical references and cannot select a build.
- `FGFDMExec` bindings are generated automatically from `FGFDMExec.h`.
- No static data preloading is used.
- Runtime data lives in Emscripten MEMFS for speed.
- Optional persistence is available through IDBFS sync (browser).
- SDK package output is ESM-first.

## Installation and build

The local fork package is `@felipegalind0/jsbsim`, `1.2.4-fork.6`. It is not published by this workflow. Install the exact checked tarball recorded in `build/last-package.json`; the native revision is identified separately by `buildIdentity.native.commit`.

Use the Node, npm, CMake and Emscripten versions in `build-toolchain.lock.json`, with `emcmake` and `em++` in `PATH`:

```bash
cd wasm
npm ci
# Clean enclosing JSBSim repository:
npm run build
# Or explicitly capture uncommitted development changes:
npm run build:dev
npm run pack:build
```

The orchestrator captures stable source inputs, generates bindings, builds WASM and TypeScript, runs the SDK checks, and assembles an identified package under `build/artifacts/`. Failed builds do not replace accepted artifacts. Canonical `dist/` is not a build or packaging input. The legacy partial-build aliases invoke the same full pipeline.

Normal builds do not fetch native source, alter Git branches or versions, apply vendor patches, publish, or push. See [the source and build contract](docs/centralized-builds.md) for capture boundaries, clean/development builds, metadata, package integrity and rollback.

## SDK Usage

`JSBSimSdk` extends a generated `JSBSimApi` class:

- `JSBSimApi` is generated from `FGFDMExec.h` and exposes methods in `camelCase`.
- `JSBSimSdk` adds runtime/VFS helpers (`create`, `writeDataFile`, `syncToPersistence`, etc.).

### Basic lifecycle

```ts
import { JSBSimSdk } from "@felipegalind0/jsbsim";
import { wasmBinaryUrl, wasmModuleUrl } from "@felipegalind0/jsbsim/wasm";

const sdk = await JSBSimSdk.create({
  moduleUrl: wasmModuleUrl,
  wasmUrl: wasmBinaryUrl,
  persistence: { enabled: true },
  log: {
    console: true,
    stripAnsi: true
  }
});

sdk.on("stdout", (entry) => {
  console.log("[jsbsim]", entry.message);
});

sdk.on("stderr", (entry) => {
  console.error("[jsbsim]", entry.message);
});

// Write runtime files into MEMFS
sdk.writeDataFile("aircraft/c172/c172.xml", xmlText);
sdk.writeDataFile("scripts/c172-test.xml", scriptXml);

// Optional: override runtime search paths
sdk.configurePaths({
  rootDir: "/runtime",
  aircraftPath: "aircraft",
  enginePath: "engine",
  systemsPath: "systems"
});

sdk.loadModel("c172"); // addModelToPath defaults to true
sdk.loadScript("scripts/c172-test.xml"); // deltaT defaults to 0, initfile defaults to ""
sdk.runIc();

while (sdk.run()) {
  const altitudeFt = sdk.getPropertyValue("position/h-sl-ft");
  if (altitudeFt > 2000) break;
}

await sdk.syncToPersistence();
```

### WASM package export

Use the package `/wasm` export to reference the bundled runtime artifacts:

```ts
import { wasmBinaryUrl, wasmModuleUrl } from "@felipegalind0/jsbsim/wasm";
```

Raw artifact subpaths are also exported:

- `@felipegalind0/jsbsim/wasm/module`
- `@felipegalind0/jsbsim/wasm/binary`

> [!WARNING]  
> To use the binary/module URLs you must disable dependancy optimisation in your bundler. For example, in Vite, set `optimizeDeps.exclude: ["@felipegalind0/jsbsim"]`. Alternatively, upload the WASM binary/module file to `public/` and pass the URL directly.

### Enums and mode flags

```ts
import {
  JSBSimSdk,
  TrimMode,
  ResetToInitialConditionsMode
} from "@felipegalind0/jsbsim";

const sdk = await JSBSimSdk.create();

sdk.setTrimMode(TrimMode.tLongitudinal);
sdk.doTrim(TrimMode.tLongitudinal);

// Reset flags are bitmasks and can be OR-ed together.
sdk.resetToInitialConditions(
  ResetToInitialConditionsMode.START_NEW_OUTPUT |
  ResetToInitialConditionsMode.DONT_EXECUTE_RUN_IC
);

// Required when DONT_EXECUTE_RUN_IC is set.
sdk.runIc();
```

### Overloads and default parameters

```ts
// Generated overloads map directly to FGFDMExec overloads.
sdk.loadModel("c172");
sdk.loadModel("aircraft", "engine", "systems", "c172");

// JSBSimSdk helper for path overrides.
sdk.loadModelWithOptions("c172", {
  aircraftPath: "aircraft",
  enginePath: "engine",
  systemsPath: "systems",
  addModelToPath: true
});

// Default args mirrored from FGFDMExec.h.
sdk.loadPlanet("earth.xml"); // useAircraftPath defaults to true
sdk.forceOutput(); // idx defaults to 0
const catalog = sdk.queryPropertyCatalog("fcs/"); // end_of_line defaults to "\n"
```

### Raw exec access

If you need the underlying embind object, it is available on `sdk.exec`:

```ts
sdk.exec.RunIC();
sdk.exec.Run();
```

### Fast property access

`getPropertyValue()` converts the name and walks the property tree on every
call. When a simulation loop reads the same properties every step, resolve them
once with a batch:

```ts
const gear = sdk.createPropertyBatch([
  "gear/unit[0]/WOW",
  "gear/unit[0]/compression-ft",
  "velocities/u-fps",
]);

sdk.run();
const [wow, compressionFt, uFps] = gear.read(); // one call for every value

gear.write([1, 0.2, 90]); // or gear.set(index, value)
gear.dispose();
```

`read()` returns a `Float64Array` view of wasm memory: use it before the next
SDK call, or pass your own array (`gear.read(target)`) to copy. Paths that do
not exist read as `NaN` and are listed in `gear.missing`; pass
`{ create: true }` to create them. Create batches after `loadModel()`.

On an Apple M5 (Node 26, C172, `npm run bench:property-batch`), reading 15
properties took 3.58 µs with `getPropertyValue()` and 0.14 µs with a batch.

### Gear contact state

`createGearContactReader()` returns a read-only snapshot of every landing-gear
and contact unit after the last `run()`: weight on wheels, strut compression,
compression velocity and force, contact location relative to the CG, wheel
roll/side velocity, slip and steering angles, and the roll/side and body-axis
reaction forces JSBSim applied.

```ts
import { GEAR_CONTACT_FIELDS } from "@felipegalind0/jsbsim";

const contacts = sdk.createGearContactReader();
sdk.run();
const values = contacts.read(); // contacts.count * contacts.stride numbers
const noseGear = contacts.readUnit(0); // { wow, strutForceLbs, wheelRollForceLbs, ... }
```

Units and signs are JSBSim's (ft, ft/s, lbf, deg); a supporting strut reports a
negative force, and the body forces of all units sum to
`forces/fb{x,y,z}-gear-lbs`. Reading never changes the simulation. Readers are
detached automatically by `sdk.destroy()`.

## Testing

```bash
npm run test:source
# The full build runs typechecking and all tests before promotion.
# These commands recheck the accepted artifact and its captured inputs:
npm test
npm run typecheck
```

Real-WASM tests and benchmarks use the C172 model and runtime files from the same resolved native snapshot that was compiled. Checks reject changed SDK inputs, enclosing repository inputs or artifact bytes rather than silently testing an older build. Results establish software behavior; they do not establish aircraft calibration.

## Updating native source and packaging

Native updates are explicit: preserve and test the canonical native integration commit, create its Git archive under `sources/`, then update its lock:

```bash
npm run update:jsbsim -- \
  --origin=https://github.com/Felipegalind0/jsbsim \
  --commit=FULL_NATIVE_COMMIT \
  --archive=sources/jsbsim-FULL_NATIVE_COMMIT.tar.gz
```

The updater validates and hashes the archive and writes only the lock. It does not select latest upstream, modify a checkout, increment package versions or commit. Keep the reviewed source archive and lock together.

`npm run pack:build` packs only a checked immutable artifact; `npm run release` additionally requires clean inputs from one repository revision. These commands write a tarball and external integrity record under `build/packages/`; neither publishes nor performs Git operations. Direct canonical `npm pack` is blocked because canonical `dist/` may contain a historical build.

The former automatic updater workflow now verifies the committed lock and builds a checked package with read-only repository permissions. It does not create dependency PRs. Demo deployment is manual. Publication and upstream contributions require separate review and authorization.

## Build identity

```ts
import { buildIdentity } from "@felipegalind0/jsbsim";
```

The identity records native and SDK commits, content digests, dirty state, build mode, toolchain and options. The `@felipegalind0/jsbsim/build-metadata` JSON export adds every distributed file hash, source/dependency lock provenance, generated-binding identity and completed checks. Tarball integrity is recorded externally to avoid self-referential hashes.

## Demo SPA

The `demo/` folder contains a simple React + Vite UI that:

- Loads the WASM module
- Preloads a hobby rocket model and launch script
- Provides launch/pause-resume/reload controls
- Streams altitude, vertical velocity, and vertical acceleration telemetry live

From repo root:

```bash
npm run demo:install
npm run build
npm run demo:sync-assets
npm run demo:dev
```

## Notes on API Exposure

The binding generator parses `FGFDMExec.h` and emits:

- `src/generated/fgfdmexec-api.ts` (TypeScript types/enums for embind surface)
- `src/generated/jsbsim-api.ts` (camelCase wrapper class with JSDoc/defaults)

Most public `FGFDMExec` methods are exposed automatically; a small ignore list is used for methods that are not useful in this SDK context (for example output file-name overrides). For complex native types that are not JS-safe, opaque numeric handles are used.

Hand-written bindings that go beyond `FGFDMExec` live in `bindings/*.cpp` and are compiled into the same module (`PropertyBatch`, `GearContacts`). They are not regenerated.

## License

This SDK bundles a WebAssembly build of JSBSim, which is licensed under the GNU Lesser General Public License v2.1 (LGPL-2.1). The WebAssembly bundle, and associated compatibility patches (`/patches`) are distributed under the terms of the LGPL-2.1.

This SDK itself, which provides a Javascript/Typescript wrapper around the JSBSim WASM module, is licensed under the MIT License.
