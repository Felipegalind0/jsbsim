# Historical upstream release artifacts

The `dist-1.2.4-beta.*.tar.gz` archives and `history/publish-metadata-1.2.4-beta.4.json` are preserved historical upstream outputs. They do not identify this fork’s current SDK or native build. Their existing bytes have not been rewritten.

Current checked packages and integrity records are written under `build/packages/`, selected by `build/last-package.json`. `npm run release:metadata` reads that selected package; it never presents this historical metadata as current.
