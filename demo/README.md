# JSBSim WASM Demo SPA

This demo is a lightweight React + Vite app that boots a preloaded hobby-rocket scenario.

## Run

From repo root:

```bash
npm run demo:install
npm run build
npm run demo:sync-assets
npm run demo:dev
```

Then open the Vite URL. The app preloads a rocket model and launch script. Press `Launch` to ignite and track the full up/down flight profile.

## What it shows

- WASM module load (`jsbsim_wasm.mjs` + `jsbsim_wasm.wasm`)
- Preloaded hobby rocket model + script
- Launch / pause-resume / reload controls
- Flight stage widget (launch, burnout, coast, apogee, descent, landing)
- Live telemetry cards and `recharts` streams (altitude, vertical velocity, vertical acceleration)

`demo:sync-assets` stages the accepted SDK JavaScript, declarations, WASM and metadata together under `public/sdk`. Both the Vite alias and TypeScript alias use that distribution; the demo does not import canonical SDK source or root `dist`. Vite verifies all synchronized file hashes before serving or building. Replaced generated distributions are retained under `build/demo-history`; old `public/wasm` files are historical and unused.

For explicit canonical native development, replace the build command with `npm run build:local -- --jsbsim-source=../jsbsim`. The demo source is captured in the SDK input manifest; a demo UI change requires a new identified SDK build before the full acceptance sequence.
