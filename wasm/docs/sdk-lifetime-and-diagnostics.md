# Native ownership and model-load diagnostics

The early local run passed 17 tests; it predates source centralization and is
retained as historical evidence. The coherent local build using native
`61b3132947dde7bc46827fd9fe3eef31193336d2` passed typechecking and 40 tests,
including real-WASM executive destruction, failed/repeated model loads and
plain-text logging. Final clean pinned artifacts run the complete current suite
and carry their own validation metadata. See [the build contract](centralized-builds.md).
These checks do not measure browser process memory reclamation. No new upstream
PR has been opened for this work.

## Ownership

An SDK owns its executive, property batches, and gear readers. Call
`sdk.destroy()` once finished; repeated calls are harmless. Do not separately
delete `sdk.exec`. SDK destruction disposes its model-bound views before
calling the Embind executive's `delete()`. Older runtime factories with only
`module.destroy(exec)` retain a compatibility path. A missing destructor
is an error rather than a silently successful cleanup.

Explicit child disposal unregisters the child from the SDK. Loading another
model, even an unsuccessful load attempt, invalidates existing property
batches and gear readers. Recreate them for the next model. Direct calls to
the publicly exposed native executive bypass these ownership protections.

Initialization errors after native allocation also dispose the executive.
Cleanup attempts continue after child failures, which are reported through
an `AggregateError`. This frees SDK-owned objects; it does not promise that
an Emscripten module's entire memory allocation is returned to the OS.

This follows [Embind's ownership model](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html#memory-management).

## Model loading

```ts
try {
  sdk.loadModelOrThrow("sf50");
} catch (error) {
  if (error instanceof JSBSimModelLoadError) {
    console.error(error.model, error.paths, error.logs, error.cause);
  }
  throw error;
}
```

The error distinguishes a false native return from a thrown value. Numeric
Wasm exceptions remain available as `cause`; they are not falsely decoded
as meaningful error messages. Each attempt retains at most 128 log entries,
with each message/raw field limited to its last 2048 characters. The message
includes the last eight entries. Earlier runtime output is excluded.
Existing boolean-returning APIs remain available.

## Checks to run

```sh
npm run build:sdk
node --test test/sdk-lifecycle.test.mjs
npm test
npm run typecheck
```

The focused tests use injected native factories to test failure paths without
requiring a Wasm rebuild. The existing extension tests additionally exercise
a real built runtime. Repeated browser create/dispose measurements are still
needed before making browser-memory claims.
