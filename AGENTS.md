# Agent instructions

This checkout is Felipegalind0's fork of JSBSim. Its `master` carries the fork's
own work, the WASM SDK under `wasm/` among it, and pull requests to JSBSim-Team
are cut from review branches that hold only what each one changes. This file is
the fork's and never goes into one. How the fork is built, packaged and used by
0SFS: `../../0sfs/docs/jsbsim.md`; how upstream contributions are made:
`../../0sfs/docs/jsbsim-upstream-contribution-policy.md`.

## Compute

- Compute something once; compute it twice only when that is the cheapest way.
  A value the model reads every step and that changes rarely is computed when it
  changes, not every step; a check that runs repeatedly stops when it has
  nothing left to do.
- Anything here that draws, such as a demo page under `wasm/`, draws a frame only
  when what it shows changes, as FOSS Earth's
  [render on demand](../../foss-earth/docs/render-on-demand.md) describes.

## UI

- A dropdown is as wide as its own text, its longest option, and never stretched
  to its row, column or panel, as in FOSS Earth's
  [UI layout](../../foss-earth/docs/ui-layout.md#dropdowns-fit-their-own-text).
