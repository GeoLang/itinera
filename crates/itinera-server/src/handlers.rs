use axum::{
    Router,
    extract::{Query, State},
    http::StatusCode,
    middleware,
    response::Json,
    routing::get,
};
use serde::{Deserialize, Serialize};
use tower_http::cors::CorsLayer;
use tower_http::trace::TraceLayer;

use itinera_core::{Route, astar, dijkstra, isochrone, vrp};
use itinera_graph::{Coord, SpeedProfile};

use crate::state::AppState;
use crate::{auth, metrics};

/// Build the HTTP router.
pub fn router(state: AppState) -> Router {
    // Install Prometheus metrics
    metrics::install();

    Router::new()
        .route("/route", get(route_handler))
        .route("/nearest", get(nearest_handler))
        .route("/isochrone", get(isochrone_handler))
        .route("/delivery/optimize", axum::routing::post(delivery_optimize))
        .route("/health", get(health_handler))
        .route("/healthz", get(liveness_handler))
        .route("/readyz", get(readiness_handler))
        .route("/metrics", get(metrics::metrics_handler))
        .layer(middleware::from_fn(auth::auth_middleware))
        .layer(TraceLayer::new_for_http())
        .layer(CorsLayer::permissive())
        .with_state(state)
}

// === Request/Response types ===

#[derive(Debug, Deserialize)]
struct RouteQuery {
    /// Source coordinate: "lat,lon"
    from: String,
    /// Target coordinate: "lat,lon"
    to: String,
    /// Algorithm: "dijkstra", "astar", or "ch" (default: "astar")
    algorithm: Option<String>,
    /// Profile: "car", "bicycle", "pedestrian", "truck" (default: "car")
    profile: Option<String>,
}

#[derive(Debug, Deserialize)]
struct NearestQuery {
    lat: f64,
    lon: f64,
}

#[derive(Debug, Deserialize)]
struct IsochroneQuery {
    lat: f64,
    lon: f64,
    /// Max travel time in seconds.
    max_seconds: f64,
    /// Profile: "car", "bicycle", "pedestrian", "truck" (default: "car")
    profile: Option<String>,
}

#[derive(Debug, Serialize)]
struct RouteResponse {
    distance_m: f64,
    duration_s: f64,
    geometry: Vec<[f64; 2]>,
    steps: Vec<StepResponse>,
}

#[derive(Debug, Serialize)]
struct StepResponse {
    distance_m: f64,
    duration_s: f64,
    name: Option<String>,
    maneuver: String,
}

#[derive(Debug, Serialize)]
struct NearestResponse {
    node_id: u32,
    lat: f64,
    lon: f64,
    distance_m: f64,
}

#[derive(Debug, Serialize)]
struct IsochroneResponse {
    reachable_nodes: usize,
    boundary: Vec<[f64; 2]>,
}

#[derive(Debug, Serialize)]
struct ErrorResponse {
    error: String,
}

// === Handlers ===

async fn health_handler() -> &'static str {
    "ok"
}

/// Liveness probe — always returns 200 if the process is running.
async fn liveness_handler() -> &'static str {
    "ok"
}

/// Readiness probe — checks that a graph is loaded.
async fn readiness_handler(State(state): State<AppState>) -> (StatusCode, &'static str) {
    if state.graph.num_nodes() > 0 {
        (StatusCode::OK, "ready")
    } else {
        (StatusCode::SERVICE_UNAVAILABLE, "not ready")
    }
}

async fn route_handler(
    State(state): State<AppState>,
    Query(params): Query<RouteQuery>,
) -> Result<Json<RouteResponse>, (StatusCode, Json<ErrorResponse>)> {
    ::metrics::counter!("itinera_route_requests").increment(1);
    let from = parse_coord(&params.from).map_err(bad_request)?;
    let to = parse_coord(&params.to).map_err(bad_request)?;

    let profile = resolve_profile(params.profile.as_deref(), &state.profile)?;

    let source = state
        .graph
        .nearest_node(from)
        .ok_or_else(|| bad_request("no node found near source".to_string()))?;
    let target = state
        .graph
        .nearest_node(to)
        .ok_or_else(|| bad_request("no node found near target".to_string()))?;

    let algo = params.algorithm.as_deref().unwrap_or("astar");

    match algo {
        "ch" => {
            let ch = state.ch.as_ref().ok_or_else(|| {
                bad_request(
                    "contraction hierarchy not available; use 'astar' or 'dijkstra'".to_string(),
                )
            })?;
            let (cost, path) = ch.query(source, target, &profile).ok_or_else(|| {
                (
                    StatusCode::NOT_FOUND,
                    Json(ErrorResponse {
                        error: "no route found".to_string(),
                    }),
                )
            })?;

            let geometry: Vec<[f64; 2]> = path
                .iter()
                .filter_map(|nid| ch.graph.node_coord(*nid))
                .map(|c| [c.lat, c.lon])
                .collect();

            Ok(Json(RouteResponse {
                distance_m: cost * 50.0 / 3.6, // approximate from travel time
                duration_s: cost,
                geometry,
                steps: Vec::new(), // CH doesn't produce detailed steps
            }))
        }
        _ => {
            let route: Route = match algo {
                "dijkstra" => dijkstra(&state.graph, source, target, &profile),
                _ => astar(&state.graph, source, target, &profile),
            }
            .map_err(|e| {
                (
                    StatusCode::NOT_FOUND,
                    Json(ErrorResponse {
                        error: e.to_string(),
                    }),
                )
            })?;

            Ok(Json(RouteResponse {
                distance_m: route.distance_m,
                duration_s: route.duration_s,
                geometry: route.geometry.iter().map(|c| [c.lat, c.lon]).collect(),
                steps: route
                    .steps
                    .iter()
                    .map(|s| StepResponse {
                        distance_m: s.distance_m,
                        duration_s: s.duration_s,
                        name: s.name.clone(),
                        maneuver: format!("{:?}", s.maneuver),
                    })
                    .collect(),
            }))
        }
    }
}

async fn nearest_handler(
    State(state): State<AppState>,
    Query(params): Query<NearestQuery>,
) -> Result<Json<NearestResponse>, (StatusCode, Json<ErrorResponse>)> {
    ::metrics::counter!("itinera_nearest_requests").increment(1);
    let coord = Coord::new(params.lat, params.lon);
    let node_id = state
        .graph
        .nearest_node(coord)
        .ok_or_else(|| bad_request("graph is empty".to_string()))?;

    let node_coord = state.graph.node_coord(node_id).unwrap();
    let distance = coord.distance_to(node_coord);

    Ok(Json(NearestResponse {
        node_id: node_id.0,
        lat: node_coord.lat,
        lon: node_coord.lon,
        distance_m: distance,
    }))
}

async fn isochrone_handler(
    State(state): State<AppState>,
    Query(params): Query<IsochroneQuery>,
) -> Result<Json<IsochroneResponse>, (StatusCode, Json<ErrorResponse>)> {
    ::metrics::counter!("itinera_isochrone_requests").increment(1);
    let coord = Coord::new(params.lat, params.lon);
    let profile = resolve_profile(params.profile.as_deref(), &state.profile)?;

    let source = state
        .graph
        .nearest_node(coord)
        .ok_or_else(|| bad_request("graph is empty".to_string()))?;

    let result = isochrone(&state.graph, source, params.max_seconds, &profile);

    Ok(Json(IsochroneResponse {
        reachable_nodes: result.nodes.len(),
        boundary: result.boundary.iter().map(|c| [c.lat, c.lon]).collect(),
    }))
}

// === Helpers ===

fn parse_coord(s: &str) -> Result<Coord, String> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        return Err(format!(
            "invalid coordinate format: '{s}', expected 'lat,lon'"
        ));
    }
    let lat = parts[0]
        .trim()
        .parse::<f64>()
        .map_err(|e| format!("invalid latitude: {e}"))?;
    let lon = parts[1]
        .trim()
        .parse::<f64>()
        .map_err(|e| format!("invalid longitude: {e}"))?;
    Ok(Coord::new(lat, lon))
}

fn resolve_profile(
    name: Option<&str>,
    default: &SpeedProfile,
) -> Result<SpeedProfile, (StatusCode, Json<ErrorResponse>)> {
    match name {
        Some(name) => SpeedProfile::from_name(name).ok_or_else(|| {
            bad_request(format!(
                "unknown profile '{name}'; valid options: car, bicycle, pedestrian, truck"
            ))
        }),
        None => Ok(default.clone()),
    }
}

fn bad_request(msg: String) -> (StatusCode, Json<ErrorResponse>) {
    (StatusCode::BAD_REQUEST, Json(ErrorResponse { error: msg }))
}

// === Delivery Optimization ===

#[derive(Debug, Deserialize)]
struct DeliveryOptimizeRequest {
    depot: LatLng,
    stops: Vec<DeliveryStop>,
    #[serde(default = "default_true")]
    return_to_depot: bool,
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Deserialize)]
struct DeliveryStop {
    id: String,
    lat: f64,
    lng: f64,
}

#[derive(Debug, Deserialize)]
struct LatLng {
    lat: f64,
    lng: f64,
}

#[derive(Debug, Serialize)]
struct DeliveryOptimizeResponse {
    ordered_stops: Vec<OrderedStop>,
    total_distance_m: f64,
    estimated_duration_s: f64,
}

#[derive(Debug, Serialize)]
struct OrderedStop {
    id: String,
    lat: f64,
    lng: f64,
    sequence: usize,
}

async fn delivery_optimize(
    Json(req): Json<DeliveryOptimizeRequest>,
) -> Result<Json<DeliveryOptimizeResponse>, (StatusCode, Json<ErrorResponse>)> {
    ::metrics::counter!("itinera_delivery_requests").increment(1);
    if req.stops.is_empty() {
        return Err(bad_request("at least one stop required".into()));
    }
    if req.stops.len() > 500 {
        return Err(bad_request("max 500 stops supported".into()));
    }

    let depot = vrp::Stop {
        id: "depot".into(),
        lat: req.depot.lat,
        lng: req.depot.lng,
    };
    let stops: Vec<vrp::Stop> = req
        .stops
        .iter()
        .map(|s| vrp::Stop {
            id: s.id.clone(),
            lat: s.lat,
            lng: s.lng,
        })
        .collect();

    let result = vrp::optimize_route(&depot, &stops, req.return_to_depot);

    let ordered_stops: Vec<OrderedStop> = result
        .order
        .iter()
        .enumerate()
        .map(|(seq, &idx)| OrderedStop {
            id: stops[idx].id.clone(),
            lat: stops[idx].lat,
            lng: stops[idx].lng,
            sequence: seq + 1,
        })
        .collect();

    // Rough duration estimate: assume 30 km/h average for urban delivery
    let duration_s = result.total_distance / (30_000.0 / 3600.0);

    Ok(Json(DeliveryOptimizeResponse {
        ordered_stops,
        total_distance_m: result.total_distance,
        estimated_duration_s: duration_s,
    }))
}
