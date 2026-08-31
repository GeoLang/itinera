# Changelog

All notable changes to this project will be documented in this file.

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
