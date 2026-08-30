# 🛣️ Itinera

**Pure-Rust routing engine** — a modern alternative to OSRM, Valhalla, and GraphHopper.

Routing core with no C dependencies. Single binary. Blazing fast.

![License](https://img.shields.io/badge/license-AGPL--3.0-blue)
![Rust](https://img.shields.io/badge/Rust-2024-orange)
![Tests](https://img.shields.io/badge/tests-100_passing-brightgreen)
![CI](https://github.com/GeoLang/itinera/actions/workflows/ci.yml/badge.svg)

[Documentation](https://geolang.github.io/itinera/) · [GitHub](https://github.com/GeoLang/itinera)

---

## Why Itinera?

| | OSRM | Valhalla | GraphHopper | **Itinera** |
|--|------|----------|-------------|-------------|
| Language | C++ | C++ | Java | **Rust** |
| Memory safety | ❌ | ❌ | ✅ (GC) | ✅ (compile-time) |
| C dependencies | Many | Many | JVM | **None in the routing core** (the server's TLS/JWT stack pulls `ring` and `aws-lc-sys`) |
| WASM support | ❌ | ❌ | ❌ | ❌ |
| Single binary | ❌ | ❌ | ❌ | ✅ |
| License | BSD-2 | MIT | Apache-2 | AGPL-3.0 |
| Contraction Hierarchies | ✅ | ❌ | ✅ | ✅ |
| Turn-by-turn | ✅ | ✅ | ✅ | ✅ |
| Isochrones | ❌ (plugin) | ✅ | ✅ | ✅ |

**Itinera** brings the performance of C++ routing engines with Rust's safety guarantees.
No garbage collector pauses. No segfaults. No dependency hell.

---

## Features

- **Dijkstra & A\*** — Classic shortest-path algorithms with haversine heuristic
- **Contraction Hierarchies** — Fast queries on continental-scale networks (sub-millisecond is a target, see Performance Targets)
- **Isochrones** — Reachability polygons for travel-time analysis
- **OSM Import** — Parse OpenStreetMap XML and PBF into a compact routing graph
- **Turn-by-turn** — Navigation instructions with maneuver detection (bearing-based)
- **Multi-modal** — Car, bicycle, pedestrian, truck routing profiles
- **HTTP API** — REST interface with CORS support
- **CSR Graph** — Cache-friendly Compressed Sparse Row with reverse index
- **R-tree spatial index** — Fast nearest-node queries
- **Binary serialization** — Compact bincode format for instant graph loading
- **Turn restrictions** — No-turn / only-turn parsed from OSM relations and enforced by Dijkstra, A*, and Contraction Hierarchies
- **Network analysis** — Connected components, OD matrix, closest facility, betweenness centrality
- **Map matching**: HMM snapping of GPS traces onto the loaded routing graph

---

## Architecture

```
itinera/
├── crates/
│   ├── itinera-graph/    # CSR graph, nodes, edges, profiles, R-tree
│   ├── itinera-core/     # Dijkstra, A*, CH, isochrones, maneuvers
│   ├── itinera-osm/      # OSM XML + PBF import, tag parsing
│   ├── itinera-match/    # HMM map matching of GPS traces, R-tree indexed
│   ├── itinera-server/   # Axum HTTP API (route, nearest, isochrone, match, network analysis)
│   └── itinera-cli/      # CLI binary (import, preprocess, serve, route)
└── docs/                 # GitHub Pages documentation
```

---

## Quick Start

```bash
# Build from source
git clone https://github.com/GeoLang/itinera.git
cd itinera && cargo build --release

# Import OSM data (supports .osm and .osm.pbf)
itinera import --input region.osm.pbf --output graph.bin

# Pre-build Contraction Hierarchies
itinera preprocess --graph graph.bin --output ch.bin --profile car

# Start the routing server
itinera serve --bind 0.0.0.0:5000 --graph graph.bin --ch ch.bin

# Query a route
curl "http://localhost:5000/route?from=48.8566,2.3522&to=48.8738,2.2950&profile=car"

# Use CH for sub-millisecond queries
curl "http://localhost:5000/route?from=48.8566,2.3522&to=48.8738,2.2950&algorithm=ch"

# Compute isochrone (10-minute reachability)
curl "http://localhost:5000/isochrone?lat=48.8566&lon=2.3522&max_seconds=600"

# Find nearest road node
curl "http://localhost:5000/nearest?lat=48.8566&lon=2.3522"
```

---

## API Endpoints

| Endpoint | Description |
|----------|-------------|
| `GET /route?from=lat,lon&to=lat,lon` | Compute shortest route |
| `GET /nearest?lat=...&lon=...` | Find nearest graph node |
| `GET /isochrone?lat=...&lon=...&max_seconds=...` | Reachability polygon |
| `POST /match` | Snap a GPS trace to the road network |
| `POST /delivery/optimize` | Stop ordering over great-circle distances at a fixed 30 km/h, not road distances |
| `POST /network/components` | Connected components of the graph |
| `POST /network/od-matrix` | Origin-destination travel time matrix |
| `POST /network/closest-facility` | Nearest facility for each demand point |
| `POST /network/betweenness` | Approximate betweenness centrality |
| `GET /health` | Health check |
| `GET /healthz`, `GET /readyz` | Liveness and readiness probes |
| `GET /metrics` | Prometheus metrics |

**Query parameters:**
- `profile` — `car` (default), `bicycle`, `pedestrian`, `truck`
- `algorithm` — `astar` (default), `dijkstra`, `ch`

**Network analysis:** points are given as `{"lat": ..., "lon": ...}` and snap to the nearest
graph node. `/network/od-matrix` takes `origins` and `destinations`, `/network/closest-facility`
takes `demand_points` and `facilities`, both capped at 100 points per list and 2500 pairs per
request. `/network/betweenness` takes `sample_size` (1–1000, default 64). `/network/components`
and `/network/betweenness` return the `top_k` largest results (default 20). Costs are travel
times in seconds under the requested `profile`.

```bash
curl -X POST http://localhost:5000/network/od-matrix \
  -H 'Content-Type: application/json' \
  -d '{"origins":[{"lat":48.8566,"lon":2.3522}],"destinations":[{"lat":48.8738,"lon":2.2950}]}'
```

**Map matching:** `POST /match` takes a `trace` of `{"lat": ..., "lon": ...}` points, each
optionally carrying `timestamp`, `accuracy_m`, `speed_mps` and `bearing_deg`, which are
accepted but do not yet steer the match. The optional
`profile` is `driving` (default), `walking` or `cycling`, and `search_radius_m` (default 50,
max 1000) sets how far from a point the matcher looks for roads. A trace holds 1 to 1000
points. The trace is snapped to the graph the server loaded, which is indexed as one segment
per road, so a two-way road counts once. The response gives the snapped points with their
road names, the matched route, a confidence in 0 to 1, the total distance, and the roads the
trace ran along with their travel times under the requested profile.

```bash
curl -X POST http://localhost:5000/match \
  -H 'Content-Type: application/json' \
  -d '{"trace":[{"lat":48.8566,"lon":2.3522},{"lat":48.8570,"lon":2.3530}],"profile":"driving"}'
```

**Response (route):**
```json
{
  "distance_m": 4700.0,
  "duration_s": 282.0,
  "geometry": [[48.8566, 2.3522], [48.8606, 2.3376], [48.8738, 2.2950]],
  "steps": [
    {"distance_m": 1200, "duration_s": 72, "name": "Rue de Rivoli", "maneuver": "Depart"},
    {"distance_m": 3500, "duration_s": 210, "name": "Champs-Élysées", "maneuver": "TurnRight"}
  ]
}
```

---

## Performance Targets

| Metric | Target |
|--------|--------|
| Graph build (Germany, 20M edges) | < 60s |
| CH preprocessing (Germany) | < 5 min |
| Point-to-point query (CH) | < 1ms |
| Isochrone (10 min budget) | < 50ms |
| Memory (Germany graph) | < 2 GB |
| Binary graph load time | < 2s |

---

## Routing Profiles

| Profile | Motorway | Trunk | Primary | Secondary | Tertiary | Unclassified | Residential |
|---------|----------|-------|---------|-----------|----------|--------------|-------------|
| Car | 130 | 100 | 80 | 60 | 50 | 40 | 30 |
| Truck | 90 | 80 | 60 | 50 | 40 | 30 | 20 |
| Bicycle | — | 25 | 22 | 20 | 18 | 16 | 15 |
| Pedestrian | — | 5 | 5 | 5 | 5 | 5 | 5 |

Speeds in km/h. "—" means road class is inaccessible for that mode.

---

## Maneuver Detection

Itinera detects turn-by-turn maneuvers using bearing-difference analysis:

| Angle | Maneuver |
|-------|----------|
| < 10° | Continue |
| 10–45° | Slight turn |
| 45–135° | Turn |
| 135–170° | Sharp turn |
| > 170° | U-turn |

---

## Development

```bash
# Run all tests
cargo test --all

# Format & lint (required before commit)
cargo fmt --all
cargo clippy --all-targets --all-features -- -D warnings

# Build release binary
cargo build --release
```

---

## License

AGPL-3.0-or-later, see [LICENSE](LICENSE).

Copyright (C) 2026 Grok Image Compression Inc.
