# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased] - 2026-09-20

### Added

- `Graph::coverage`, the area the loaded network answers for, and
  `Graph::snap_within_coverage`, the nearest-node lookup that respects it. A PBF
  import reads the `HeaderBBox` out of the OSMHeader blob and an XML import
  reads the `<bounds>` element, both landing in `Graph::declared_bounds`, so
  graph.bin carries the bounds the file itself declares. A file with no bounds,
  which is what osmium writes without `--set-bounds`, falls back to the extent
  of the graph's own nodes grown by the longest edge in the graph, computed once
  on first use. A point within one edge of the outermost node can still sit on a
  road of this network, which is what coverage is asking.

### Fixed

- Route, isochrone, od-matrix and closest-facility refuse a point outside the
  coverage instead of snapping it to whatever node is nearest. Against a Monaco
  extract, `GET /isochrone?lat=43.647&lon=-79.41&max_seconds=600` returned the
  Monaco isochrone, 1292 nodes around 43.72, 7.41. It now answers 400 with
  "origin 43.647, -79.410 is outside the loaded road network", followed by the
  coverage as "(lon <min> to <max>, lat <min> to <max>)". A point inside the
  coverage snaps to the nearest node as before, at any distance. `/nearest` is
  unchanged and still answers for any point, reporting how far the node is.
- `itinera route` and `itinera isochrone` refuse an out-of-coverage point with
  that same message, where they used to snap to the nearest node anywhere in the
  graph.

### Changed

- graph.bin gained the declared bounds field, so a file written by an earlier
  build fails to load with "io error: unexpected end of file". Rebuild it with
  `itinera import`.

## [Unreleased] - 2026-09-16

### Fixed

- Public docs audited against the code. The README drops "Blazing fast" and the
  claim that itinera brings the performance of C++ routing engines, which no
  benchmark here measures, and the Contraction Hierarchies bullet now names the
  576-node grid the 170 us came from instead of continental-scale networks. On
  docs/index.html the unit test stat goes from 105 to 102, matching the
  attribute count and the badge, and the line count from ~9K to ~8K.

## [Unreleased] - 2026-09-02

### Removed

- `itinera_server::api_keys`, the in-memory API key store with its permissions,
  token bucket and daily counter. Nothing called it, and authentication is the
  bearer JWT behind `ITINERA_JWT_SECRET`. The `hex` module it was the only user
  of goes with it, along with the `sha2` and `chrono` dependencies.

## [Unreleased] - 2026-08-31

### Added

- A criterion bench suite (`performance_targets` in itinera-core) covering
  graph build, OSM XML import, CH preprocessing and query, isochrones and the
  binary round trip, on generated street grids sized in each bench id. The
  README performance table now quotes numbers measured with it on an idle
  24-core machine instead of unmeasured design targets.

### Fixed

- `ContractionHierarchy::query` can traverse shortcuts. A shortcut edge is stored with road
  class 0, which every speed profile maps to 0 km/h, so `edge_weight` reported it as
  unreachable and both directions of the search skipped it. On any graph whose contraction
  adds shortcuts, that left the query returning no route at all, or one slower than Dijkstra
  finds on the same graph. A shortcut now costs the travel time recorded when it was built.
  Only grids of identical streets escaped it, because contraction adds no shortcut there,
  and those were the only graphs the tests covered.

## [Unreleased] - 2026-08-30

### Added

- `POST /match` snaps a GPS trace to the loaded routing graph. `itinera-match` was a
  library with no way to reach it and no road network to match against, so the server
  now builds one `RoadNetwork` from the graph at startup, one segment per road with the
  way's name, class and profile speed. A trace holds 1 to 1000 points, the profile is
  `driving`, `walking` or `cycling`, and `search_radius_m` defaults to 50 and is capped
  at 1000.
- `RoadNetwork` indexes its segments in an R-tree. `candidates()` scanned every segment
  for every trace point, which a graph-sized network cannot afford on a request path.

## [Unreleased] - 2026-08-12

### Changed

- README drops the WASM-capable claim (no wasm crate) and the 74-test badge
  (84). Docs match: truck is a speed table, not weight tags.
- sha2 on 0.11. API key digests are hex encoded by a local module instead of
  `{:x}`, which digest 0.11 no longer implements, and a golden test pins the
  string so a stored hash still matches.

## [Unreleased] - 2026-08-09

### Changed

- The isochrone boundary is a concave hull instead of a convex one. A convex hull
  spans every bay and dead end in the street network, so a `GET /isochrone` over an
  L or U shaped network claimed reach over ground no road touches. `isochrone()`
  takes a `concavity` argument and the endpoint takes an optional `concavity` query
  parameter, both defaulting to 2.0. Lower values hug the network more closely,
  infinity reproduces the old convex boundary.

### Fixed

- `itinera isochrone` emits a valid GeoJSON geometry. It fed the boundary ring
  straight into a `Polygon`, but the ring is open and a GeoJSON linear ring has to
  be closed and hold at least four positions, so every isochrone the CLI printed
  was rejected by strict readers. The ring is now closed, and a boundary of one or
  two points comes out as a `Point` or `LineString` rather than a broken polygon.

## [Unreleased] - 2026-08-02

### Added

- HTTP endpoints for the network analysis already in `itinera-core`: `POST /network/components`,
  `POST /network/od-matrix`, `POST /network/closest-facility`, `POST /network/betweenness`.

## [0.1.0] - 2026-05-30

### Added

- Initial release.
