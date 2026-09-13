# Canonical native source migration — 2026-09-13

The native `integration` branch consolidates the existing optional wheel-spin
feature, Emscripten portability changes, turbine trim-state correction, and the
plain-text Emscripten logger default. The WASM SDK should build this canonical
source at an explicit commit. It must not recreate a patched native source tree
inside its vendor directory.

## Preserved source identities

The native starting commit is
`24e085bf81b5ef8bab8500ab9d416571753b93cc`, originally on
`feature/wheel-spin-dof`. Its optional wheel rotational degree of freedom and
`tests/TestWheelSpin.py` remain intact. Contacts without both positive
`wheel_radius` and `wheel_inertia` continue to use the legacy path.

The earlier Emscripten portability change is
`d47fd2e38feba9cc40340ac9740356f6f7afc37d` (`fix/emscripten-portability`,
existing PR 1504). Its socket and `strerror_r` changes were already present as
working-tree changes in the canonical source and are retained without a second
implementation or a duplicate upstream PR.

The existing working-tree change to `FGTurbine::Trim()` updates the observable
member `N1` and `N2` at zero simulation time. It replaces the shadowing local
`N2`; it does not add an application property reset. Its existing
`TestTurbineTrimSpool.py` and CTest registration are retained.

The audited SDK baseline was `35d610095d71ea40c8f90a0f1e5a14e11006ee1c`,
with vendor JSBSim at `1a2e114d79af2430db02a4f7a4a85328cdc5d403`.
The vendor working-tree diff matched
`patches/jsbsim-emscripten-compat.patch`, whose SHA-256 was
`58cbf2c1b2484a06eebdc132e5c1ff6f86fa76b4c56b917e3cd2a0c80ef669a7`.
Before consolidation, complete repository preservation copies were made at
`/Users/felg/gh/.preservation/jsbsim-centralization-20260913T204345Z`.
The existing PR branches were preserved; the new integration branch is the
place for the combined source used by the SDK.

## Disposition of every old SDK patch hunk

Hunk coordinates below identify the old patch, not current source line numbers.

| Old file and hunk | Disposition in canonical native source |
| --- | --- |
| `src/FGJSBBase.cpp`, `@@ -46,6 +46,19 @@` | Preserve behavior at its current owner. Upstream `0b688c801c52d800f75d5c41e5434ce5d7618e88` moved escape codes into `FGLogConsole` in July 2026. The old static `FGJSBBase` strings no longer exist here. `src/input_output/FGLog.h` now defaults `highlighting` to false only for `__EMSCRIPTEN__`, so formatted browser/worker messages contain no ANSI escapes. The native default remains true. |
| `src/FGJSBBase.cpp`, `@@ -57,6 +70,7 @@` | The closing `#endif` belongs to the preceding obsolete static-string implementation. Its conditional behavior is represented by the new logger default; do not transplant the old block. |
| `src/FGJSBBase.cpp`, `@@ -91,4 +105,3 @@` | Drop the trailing blank-line deletion. It is cosmetic and has no compatibility effect. |
| `src/input_output/FGfdmSocket.cpp`, `@@ -40,13 +40,22 @@` | Retain the existing PR 1504 subset: select the explicit POSIX include branch for Emscripten as well as OpenBSD, and add `netinet/in.h`, `sys/select.h`, and `sys/time.h` there. Do not carry the additional six includes inserted into the generic `#else` by the old SDK patch: that branch is not used by Emscripten, and no native defect requiring this separate expansion was established. This is a scope decision, not a claim that all those declarations already appear directly in `FGfdmSocket.h`. The canonical native source builds with the narrower change. |
| `src/simgear/misc/strutils.cxx`, `@@ -682,9 +682,9 @@` | Retain PR 1504 exactly: Emscripten must not select the GNU pointer-returning `strerror_r` branch; include Emscripten in the POSIX integer-returning branch. |
| `src/simgear/misc/strutils.cxx`, `@@ -692,7 +692,7 @@` | Retain PR 1504 exactly: the POSIX return-code handling must also run on Emscripten even when `_GNU_SOURCE` is defined. |
| `src/simgear/misc/strutils.cxx`, `@@ -710,7 +710,7 @@` | Retain the matching PR 1504 closing-condition comment so it describes the actual Emscripten error-handling guard. |

The logger regression keeps the existing six format assertions, with native
ANSI expectations and Emscripten plain-text expectations selected at compile
time. An additional assertion checks that explicitly disabling formatting
preserves the message. This exercises the public logger behavior without
reintroducing removed `FGJSBBase` implementation details.

## Focused native verification

Executed on 2026-09-13 in the canonical native repository. The durable local
Python environment is `.venv/`, and the build is `build/native/`; both are
covered by the existing `.gitignore`. Neither source identity nor reproduction
instructions depend on an old temporary build directory.

Toolchain: CMake 4.4.3, AppleClang 21.0.0.21000101, Homebrew Python 3.14.7,
Cython 3.3.0, and Homebrew CxxTest (`/opt/homebrew/bin/cxxtestgen`). The focused
Python dependencies were installed from the local cache: NumPy 2.5.3, pandas
3.0.5, SciPy 1.18.1, python-dateutil 2.9.0.post0, and six 1.17.0.

Commands executed from this repository:

```sh
uv venv --python /opt/homebrew/opt/python@3.14/bin/python3.14 .venv
uv pip install --python .venv/bin/python --offline \
  numpy==2.5.3 cython==3.3.0 pandas==3.0.5 scipy==1.18.1

env -u GITHUB_RUN_NUMBER -u GITHUB_SHA cmake -S . -B build/native \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_DOCS=OFF -DBUILD_PYTHON_MODULE=ON \
  -DBUILD_JULIA_PACKAGE=OFF -DBUILD_MATLAB_SFUNCTION=OFF \
  -DPython3_EXECUTABLE=/Users/felg/gh/Felipegalind0/jsbsim/.venv/bin/python \
  -DCYTHON_EXECUTABLE=/Users/felg/gh/Felipegalind0/jsbsim/.venv/bin/cython
cmake --build build/native --target _jsbsim FGLogTest1 --parallel 6
ctest --test-dir build/native --output-on-failure \
  -R '^(TestTurbineTrimSpool|TestTurbine|TestEngineIndexedProps|CheckTrim|TestWheelSpin|TestGndReactions|TestHoldDown|FGLogTest1)$'
```

Configure and compilation passed. Missing optional PkgConfig and LSB_Release
produced nonfatal configuration warnings. CTest passed **8/8 targets** in
36.59 seconds:

| Target | Coverage exercised |
| --- | --- |
| `TestTurbineTrimSpool` | Zero-time N1/N2 state at throttle 0, 0.35, 1, then 0; unchanged clock during `run_ic`; first integrated idle sample continuity. |
| `TestTurbine` | Existing turbine spool-up regression. |
| `TestEngineIndexedProps` | Existing indexed engine property/function/table behavior. |
| `CheckTrim` | Existing ground and airborne trim, actuator delay, and rocket ignition behavior. |
| `TestWheelSpin` | Five existing tests for opt-in/invalid configuration, touchdown spin-up, rolling/braking equivalence, ground start, and airborne spin-down. |
| `TestGndReactions` | Existing centered/eccentric ground-reaction cases. |
| `TestHoldDown` | Existing static/dynamic hold-down and ground-reaction cases. |
| `FGLogTest1` | 43 CxxTest cases, including native default ANSI formatting and explicitly disabled formatting. |

The executed native binaries had these SHA-256 identities:

| Artifact | SHA-256 |
| --- | --- |
| `build/native/src/libJSBSim.a` | `160ac7ac1e56491c18f282407ccbf6d8cca44cd5a68ccd2b15036c4a6efd1ff9` |
| `build/native/tests/jsbsim/_jsbsim.so` | `38c4e5dccc2b0c5b316f4629952e14e05b8f6f6de3d1653abce2b1bc1c4290ed` |
| `build/native/tests/unit_tests/FGLogTest1` | `204b7e0ff18930f0466fdca4b7b70e12a1b888fd08566890820513f9e8f4d247` |

The optional existing `TestInputSocket` regression was not run: an offline
attempt to install `telnetlib3` found no cached package. Native socket source
compilation passed, but this run does not establish live socket behavior.
Emscripten compilation and actual emitted WASM logs require the SDK build and
checks; the native run only executes the native branch of the conditional
format expectations. These software regressions do not establish SF50
calibration or aircraft fidelity.

## Build identity contract

`src/CMakeLists.txt` embeds `GITHUB_SHA` when `GITHUB_RUN_NUMBER` is set. A SDK
workflow must not pass its own repository SHA as the native version identity.
The build orchestrator must use the resolved canonical native source commit
from its source descriptor for that environment, and record the native and SDK
identities separately. This local native configuration cleared the two GitHub
environment variables because it exercised the integration working tree before
the consolidation commit. The artifact hashes above identify that actual run;
a subsequent SDK/release build must pin the resulting native commit and its own
produced artifacts. No native version-string refactor is required for this
migration.


## Model replacement lifetime correction — 2026-09-13

The first coherent SDK build exposed a native model replacement defect at
`aeb43b705596883c8cbeb6cac80464af7cc1d76b`. A standalone native executive
loading `c172p`, then a missing aircraft, and then being destroyed reproduced
the failure without WASM, cached property batches, gear readers, or `RunIC`.
Repeated successful loads and loaded/missing/reload sequences also crashed.
The initial single successful load and initial missing-file controls did not.

AddressSanitizer identified a heap-use-after-free in
`FGLocation::GetSeaLevelRadius()`: executive destruction called
`FGPropertyManager::Unbind()`, which untied a property by reading its last
value from an already destroyed inertial model. The earlier `DeAllocate()`
had cleared the models without releasing their property bindings. Replacement
also produced hundreds of failed-property-tie warnings before the crash.

`DeAllocate()` now unbinds properties while all model and initial-condition
objects are still alive. The existing executive-owned binding block is a
shared `Bind()` helper, preserving its `Constructing` guard and restoring
executive properties after replacement allocation. A separate load-attempt
flag ensures that the next load starts with fresh models after a partially
processed XML failure as well as after a successful load. A failed load still
returns false. Clearing the catalog vector prevents duplicate catalog entries
from accumulating across repeated successful loads.

`TestModelReload.py` exercises six cases: repeated success, failure followed by
destruction, success/failure/recovery, two missing files followed by recovery,
initial failure followed by destruction, and repeated partial XML failures
followed by recovery. It explicitly triggers executive destruction. Successful
replacement cases also exercise live model state and executive `dt`, random
seed, reset, pause, clock, and termination properties; a successful load return
alone is insufficient.

The same durable native environment and build described above were used.
After the correction, this focused command passed **11/11 CTest targets** in
38.23 seconds, including the six new reload cases and all eight earlier
preservation targets:

```sh
cmake --build build/native --target _jsbsim FGLogTest1 --parallel 6
ctest --test-dir build/native --output-on-failure \
  -R '^(TestModelReload|TestModelLoading|CheckSimTimeReset|TestTurbineTrimSpool|TestTurbine|TestEngineIndexedProps|CheckTrim|TestWheelSpin|TestGndReactions|TestHoldDown|FGLogTest1)$'
```

The native extension used for this run has SHA-256
`e5499254f1d127c4db5bc4c3ea2df7976dbf6053cf3effab69fe03c2505e4070`.
The preceding artifact hashes document the earlier preservation run and do not
identify this corrected binary.

A separate ignored build at `build/reload-asan` used AppleClang with
`-fsanitize=address -fno-omit-frame-pointer`, Debug configuration, and the
standalone native reproducer linked against its `libJSBSim.a`. After rebuilding
with the correction, all six original sequence controls/reproductions exited
successfully with no sanitizer errors or failed-property-tie warnings. The
standalone executable SHA-256 was
`2e5b32a28ed591aa122bf3534e74bba90e4143b142bb70594012a6ee13c54abf`.
Local diagnostic scripts, sequence logs, pre-fix sanitizer stack, and the
post-fix summary are retained in
`build/native/model-reload-diagnostic/`; the tracked regression and this source
change are the durable reproduction contract. SDK rebuilding and WASM
verification must use a newly pinned native commit containing this fix before
claiming that the WASM failure is resolved.
