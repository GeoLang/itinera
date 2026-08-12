# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased] - 2026-08-12

### Changed

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
