# E2E test data

Run E2E tests with:

```
TRAFFIC_SIM_E2E_OSM=path/to/extract.osm.pbf go test -tags e2e ./internal/e2e/
```

## Choosing a fixture

The e2e test asserts loose, order-of-magnitude properties (≥ 100 nodes
after build, snap-fallback warnings stay below 10× the alive vehicle
count, spawn count > 0). Those thresholds tolerate normal variation
across small extracts, but the test will be **most useful** if everyone
runs it against the same fixture so a regression on one developer's
machine reproduces for everyone.

### Recommended fixture (~1–5 MB)

A small neighborhood of dense, signalized streets is ideal — large
enough to exercise lane changes, signals, and AWSC arbitration, but
small enough to keep the per-run wall-clock under a few seconds.

Two reproducible ways to get one:

1. **Pinned bbox via osmium-tool** (deterministic):
   ```
   # Manhattan Financial District, ~2 MB
   osmium extract \
     --bbox -74.020,40.700,-74.000,40.715 \
     --output testdata/cell.osm.pbf \
     planet.osm.pbf
   ```

2. **Geofabrik tile + bbox** (good enough for casual use):
   download a US-state extract from https://download.geofabrik.de/ and
   trim it with the `osmium extract` command above.

Avoid the bbbike.org "select your own bbox" UI for CI use — it doesn't
pin a date, so re-downloading later will pick up arbitrary OSM edits
and the test thresholds may drift.

### Committing the fixture

The repo intentionally does NOT ship a binary OSM fixture (the test
runs whatever path `TRAFFIC_SIM_E2E_OSM` points at), so the e2e tests
are opt-in for developers who care to set the env var. If you want
deterministic CI, commit a small fixture under this directory (a few
MB of `.osm.pbf` is fine for git) and set the env var in your CI config.
