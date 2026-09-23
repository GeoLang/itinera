# Itinera

A routing engine in Rust for OpenStreetMap road networks: shortest paths, contraction hierarchies, isochrones, map matching and network analysis over HTTP.

The routing crates have no C dependencies. The server's JWT stack pulls in `ring` and `aws-lc-sys`. Everything ships as one `itinera` binary.

![License](https://img.shields.io/badge/license-AGPL--3.0-blue)
![Rust](https://img.shields.io/badge/Rust-2024-orange)
![CI](https://github.com/GeoLang/itinera/actions/workflows/ci.yml/badge.svg)

[Documentation](https://geolang.github.io/itinera/) · [GitHub](https://github.com/GeoLang/itinera)

## Features

- **Dijkstra and A\***: A\* uses a haversine heuristic.
- **Contraction hierarchies**: bidirectional query over a hierarchy prebuilt with `itinera preprocess`, unpacked to a full path with turn-by-turn steps.
- **Turn restrictions**: no-turn and only-turn relations from OSM, enforced by Dijkstra, A\* and the hierarchy.
- **Isochrones**: concave hull around the nodes reachable within a time budget.
- **OSM import**: XML and PBF into a compressed sparse row graph with a reverse index, saved with bincode.
- **Profiles**: car, bicycle, pedestrian and truck speed tables, see [Routing Profiles](#routing-profiles).
- **Turn-by-turn**: maneuvers from the bearing change at each node.
- **Map matching**: HMM snapping of GPS traces onto the loaded graph.
- **Network analysis**: connected components, OD matrix, closest facility, approximate betweenness centrality.
- **Delivery stop ordering**: nearest neighbour plus 2-opt over great-circle distances.
- **Coverage check**: a route, isochrone or network analysis point outside the loaded network is refused with a message naming the network bounds. The bounds are the ones declared in the OSM file, otherwise the node extent grown by the longest edge.

## Architecture

```
itinera/
├── crates/
│   ├── itinera-graph/    # CSR graph, nodes, edges, profiles, R-tree
│   ├── itinera-core/     # Dijkstra, A*, CH, isochrones, maneuvers, network analysis, stop ordering
│   ├── itinera-osm/      # OSM XML + PBF import, tag parsing
│   ├── itinera-match/    # HMM map matching of GPS traces
│   ├── itinera-server/   # Axum HTTP API
│   └── itinera-cli/      # the itinera binary: import, preprocess, serve, route, isochrone
└── docs/                 # GitHub Pages site and OpenAPI spec
```

## Quick Start

```bash
cargo install --path crates/itinera-cli

# .pbf is read as PBF, anything else as OSM XML
itinera import --input region.osm.pbf --output graph.bin

# optional, needed for algorithm=ch
itinera preprocess --graph graph.bin --output ch.bin --profile car

itinera serve --bind 0.0.0.0:5000 --graph graph.bin --ch ch.bin

curl "http://localhost:5000/route?from=48.8566,2.3522&to=48.8738,2.2950&profile=car"
curl "http://localhost:5000/route?from=48.8566,2.3522&to=48.8738,2.2950&algorithm=ch"
curl "http://localhost:5000/isochrone?lat=48.8566&lon=2.3522&max_seconds=600"
curl "http://localhost:5000/nearest?lat=48.8566&lon=2.3522"
```

`serve` defaults to `--bind 0.0.0.0:5000 --graph graph.bin --profile car`. A hierarchy holds travel times for the profile it was built with, so `algorithm=ch` with a different `profile` gives wrong durations.

The CLI also answers single queries without a server:

```bash
itinera route --from 48.8566,2.3522 --to 48.8738,2.2950 --graph graph.bin
itinera isochrone --center 48.8566,2.3522 --max-seconds 600 --graph graph.bin
```

`route` takes `--algorithm astar|dijkstra`. `isochrone` prints a GeoJSON Feature.

### Docker

Tagged releases publish `ghcr.io/geolang/itinera` and prebuilt binaries for Linux and macOS on x86_64 and aarch64. The image serves `/data/graph.bin` on port 3000. If the graph is missing and `/data/region.osm.pbf` exists, it imports that first, so `/data` must be writable.

```bash
mkdir -p data
cp /path/to/region.osm.pbf data/region.osm.pbf
docker run -p 3000:3000 -v "$PWD/data:/data" ghcr.io/geolang/itinera:latest
```

`docker compose up -d` builds the image, mounts `./data` read-only and adds Prometheus on port 9090. With the read-only mount the import cannot run, so put `graph.bin` in `./data` first.

## API Endpoints

| Endpoint | Description |
|----------|-------------|
| `GET /route?from=lat,lon&to=lat,lon` | Shortest route with turn-by-turn steps |
| `GET /nearest?lat=...&lon=...` | Nearest graph node |
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

The full request and response shapes are in [docs/openapi.yaml](docs/openapi.yaml).

**Query parameters:**
- `profile`: `car`, `bicycle`, `pedestrian`, `truck`. Defaults to the server's `--profile`.
- `algorithm`, `/route` only: `astar` (default), `dijkstra`, `ch`.
- `concavity`, `/isochrone` only: `2.0` (default), zero or greater. Lower values follow the road network more closely, infinity gives a convex boundary.

**Authentication:** with `ITINERA_JWT_SECRET` set, every endpoint except `/health`, `/healthz`, `/readyz` and `/metrics` needs an `Authorization: Bearer` header holding an HS256 JWT signed with that secret, carrying `sub`, `exp` and `role` claims. With the variable unset the server answers every request. `docker-compose.yml` sets it to `change-me-in-production`.

**Network analysis:** points are given as `{"lat": ..., "lon": ...}` and snap to the nearest graph node. `/network/od-matrix` takes `origins` and `destinations`, `/network/closest-facility` takes `demand_points` and `facilities`, both capped at 100 points per list and 2500 pairs per request. `/network/betweenness` takes `sample_size` (1 to 1000, default 64). `/network/components` and `/network/betweenness` return the `top_k` largest results (default 20). Costs are travel times in seconds under the requested `profile`.

```bash
curl -X POST http://localhost:5000/network/od-matrix \
  -H 'Content-Type: application/json' \
  -d '{"origins":[{"lat":48.8566,"lon":2.3522}],"destinations":[{"lat":48.8738,"lon":2.2950}]}'
```

**Map matching:** `POST /match` takes a `trace` of `{"lat": ..., "lon": ...}` points, each optionally carrying `timestamp`, `accuracy_m`, `speed_mps` and `bearing_deg`, which are accepted but do not steer the match. The optional `profile` is `driving` (default), `walking` or `cycling`, and `search_radius_m` (default 50, max 1000) sets how far from a point the matcher looks for roads. A trace holds 1 to 1000 points. The graph is indexed as one segment per road, so a two-way road counts once. The response gives the snapped points with their road names, the matched route, a confidence from 0 to 1, the total distance, and the roads the trace ran along with their travel times under the requested profile.

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

## Performance

Measured with `cargo bench -p itinera-core --bench performance_targets` on a 24-core Threadripper with 128 GB RAM. The fixtures are generated street grids, since the repo ships no road network, so each row names the size actually measured. The last column keeps the original design targets at Germany scale (20M edges), which nothing here has measured.

| Operation | Measured | On | Target at Germany scale |
|--------|--------|--------|--------|
| Graph build (CSR + R-tree) | 192 ms | 262k nodes, 1.05M edges | < 60 s |
| OSM XML import | 116 ms | 4.8 MiB XML, 261k edges | none set |
| CH preprocessing | 3.6 s | 576 nodes, 1746 shortcuts | < 5 min |
| Point-to-point query (CH) | 170 us | 576 nodes, corner to corner | < 1 ms |
| Isochrone (10 min budget) | 7.0 ms | 11k of 262k nodes reached | < 50 ms |
| Binary graph save and load | 56 ms and 90 ms | 56 MiB | < 2 s load |
| Memory | not measured | serialized size is 56 MiB at 1.05M edges | < 2 GB |

CH preprocessing time grows steeply with node count because the node ordering rescans every remaining node at each level, so the 576-node figure does not extrapolate to large graphs.

## Routing Profiles

| Profile | Motorway | Trunk | Primary | Secondary | Tertiary | Unclassified | Residential |
|---------|----------|-------|---------|-----------|----------|--------------|-------------|
| Car | 130 | 100 | 80 | 60 | 50 | 40 | 30 |
| Truck | 90 | 80 | 60 | 50 | 40 | 30 | 20 |
| Bicycle | no access | 25 | 22 | 20 | 18 | 16 | 15 |
| Pedestrian | no access | 5 | 5 | 5 | 5 | 5 | 5 |

Speeds in km/h. `*_link` roads take their parent's class, `road` counts as unclassified, and `living_street` and `service` as residential. Import skips every other `highway` value, including `footway`, `path`, `cycleway` and `track`, so the bicycle and pedestrian profiles route on roads only. The truck table has no weight or height limits. `bike`, `foot`, `walk` and `hgv` are accepted as profile aliases.

## Maneuver Detection

| Bearing change | Maneuver |
|-------|----------|
| < 10° | Continue |
| 10° to 45° | Slight turn |
| 45° to 135° | Turn |
| 135° to 170° | Sharp turn |
| > 170° | U-turn |

## Development

```bash
cargo test --all
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
```

## License

AGPL-3.0-or-later, see [LICENSE](LICENSE).

Copyright (C) 2026 Grok Image Compression Inc.
