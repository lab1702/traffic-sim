# E2E test data

Run E2E tests with:

```
TRAFFIC_SIM_E2E_OSM=path/to/extract.osm.pbf go test -tags e2e ./internal/e2e/
```

Recommended small extract: a single neighborhood from
https://extract.bbbike.org (5-20 MB .osm.pbf).
