use std::collections::HashMap;

use axum::{
    Router,
    extract::{Query, State},
    http::StatusCode,
    middleware,
    response::Json,
    routing::{get, post},
};
use serde::{Deserialize, Serialize};
use tower_http::cors::CorsLayer;
use tower_http::trace::TraceLayer;

use itinera_core::{Route, astar, dijkstra, isochrone, network_analysis, vrp};
use itinera_graph::{Coord, Graph, NodeId, SpeedProfile};

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
        .route("/delivery/optimize", post(delivery_optimize))
        .route("/network/components", post(network_components))
        .route("/network/od-matrix", post(network_od_matrix))
        .route("/network/closest-facility", post(network_closest_facility))
        .route("/network/betweenness", post(network_betweenness))
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

// === Network Analysis ===

// od matrix and closest facility run one dijkstra per pair, and betweenness one
// per sampled source, so every request is capped up front
const MAX_NETWORK_POINTS: usize = 100;
const MAX_NETWORK_PAIRS: usize = 2500;
const MAX_BETWEENNESS_SAMPLE: usize = 1000;
const MAX_TOP_K: usize = 1000;
const DEFAULT_TOP_K: usize = 20;
const DEFAULT_BETWEENNESS_SAMPLE: usize = 64;

#[derive(Debug, Clone, Copy, Deserialize)]
struct Point {
    lat: f64,
    lon: f64,
}

#[derive(Debug, Default, Deserialize)]
struct ComponentsRequest {
    /// Number of components to return, largest first (default 20).
    top_k: Option<usize>,
}

#[derive(Debug, Deserialize)]
struct OdMatrixRequest {
    origins: Vec<Point>,
    destinations: Vec<Point>,
    profile: Option<String>,
}

#[derive(Debug, Deserialize)]
struct ClosestFacilityRequest {
    demand_points: Vec<Point>,
    facilities: Vec<Point>,
    profile: Option<String>,
}

#[derive(Debug, Deserialize)]
struct BetweennessRequest {
    /// Number of source nodes to sample (default 64).
    sample_size: Option<usize>,
    /// Number of nodes to return, highest score first (default 20).
    top_k: Option<usize>,
    profile: Option<String>,
}

#[derive(Debug, Serialize)]
struct ComponentsResponse {
    num_nodes: usize,
    num_components: u32,
    largest_component_size: usize,
    components: Vec<ComponentSummary>,
}

#[derive(Debug, Serialize)]
struct ComponentSummary {
    component_id: u32,
    size: usize,
}

#[derive(Debug, Serialize)]
struct OdMatrixResponse {
    entries: Vec<OdEntryResponse>,
}

#[derive(Debug, Serialize)]
struct OdEntryResponse {
    origin_index: usize,
    destination_index: usize,
    origin_node: u32,
    destination_node: u32,
    duration_s: f64,
}

#[derive(Debug, Serialize)]
struct ClosestFacilityResponse {
    assignments: Vec<FacilityAssignment>,
    /// Demand points with no reachable facility.
    unreachable: usize,
}

#[derive(Debug, Serialize)]
struct FacilityAssignment {
    demand_index: usize,
    facility_index: usize,
    demand_node: u32,
    facility_node: u32,
    duration_s: f64,
}

#[derive(Debug, Serialize)]
struct BetweennessResponse {
    sampled_sources: usize,
    nodes: Vec<CentralityEntry>,
}

#[derive(Debug, Serialize)]
struct CentralityEntry {
    node_id: u32,
    lat: f64,
    lon: f64,
    score: f64,
}

async fn network_components(
    State(state): State<AppState>,
    body: Option<Json<ComponentsRequest>>,
) -> Result<Json<ComponentsResponse>, (StatusCode, Json<ErrorResponse>)> {
    ::metrics::counter!("itinera_network_components_requests").increment(1);
    let req = body.map(|Json(req)| req).unwrap_or_default();
    let top_k = resolve_top_k(req.top_k)?;

    let result = network_analysis::connected_components(&state.graph);

    let mut components: Vec<ComponentSummary> = result
        .component_sizes
        .iter()
        .map(|(&component_id, &size)| ComponentSummary { component_id, size })
        .collect();
    components.sort_by(|a, b| {
        b.size
            .cmp(&a.size)
            .then_with(|| a.component_id.cmp(&b.component_id))
    });
    let largest_component_size = components.first().map_or(0, |c| c.size);
    components.truncate(top_k);

    Ok(Json(ComponentsResponse {
        num_nodes: state.graph.num_nodes(),
        num_components: result.num_components,
        largest_component_size,
        components,
    }))
}

async fn network_od_matrix(
    State(state): State<AppState>,
    Json(req): Json<OdMatrixRequest>,
) -> Result<Json<OdMatrixResponse>, (StatusCode, Json<ErrorResponse>)> {
    ::metrics::counter!("itinera_network_od_matrix_requests").increment(1);
    let profile = resolve_profile(req.profile.as_deref(), &state.profile)?;
    check_points("origins", &req.origins)?;
    check_points("destinations", &req.destinations)?;
    check_pairs(req.origins.len() * req.destinations.len())?;

    let origins = snap_points(&state.graph, &req.origins)?;
    let destinations = snap_points(&state.graph, &req.destinations)?;
    let origin_index = index_by_node(&origins);
    let destination_index = index_by_node(&destinations);

    let mut entries: Vec<OdEntryResponse> =
        network_analysis::od_matrix(&state.graph, &origins, &destinations, &profile)
            .into_iter()
            .map(|entry| OdEntryResponse {
                origin_index: origin_index[&entry.origin],
                destination_index: destination_index[&entry.destination],
                origin_node: entry.origin.0,
                destination_node: entry.destination.0,
                duration_s: entry.cost,
            })
            .collect();
    // od_matrix iterates destinations as a set, so impose an order here
    entries.sort_by_key(|e| (e.origin_index, e.destination_index));

    Ok(Json(OdMatrixResponse { entries }))
}

async fn network_closest_facility(
    State(state): State<AppState>,
    Json(req): Json<ClosestFacilityRequest>,
) -> Result<Json<ClosestFacilityResponse>, (StatusCode, Json<ErrorResponse>)> {
    ::metrics::counter!("itinera_network_closest_facility_requests").increment(1);
    let profile = resolve_profile(req.profile.as_deref(), &state.profile)?;
    check_points("demand_points", &req.demand_points)?;
    check_points("facilities", &req.facilities)?;
    check_pairs(req.demand_points.len() * req.facilities.len())?;

    let demand = snap_points(&state.graph, &req.demand_points)?;
    let facilities = snap_points(&state.graph, &req.facilities)?;
    let demand_index = index_by_node(&demand);
    let facility_index = index_by_node(&facilities);

    let results = network_analysis::closest_facility(&state.graph, &demand, &facilities, &profile);
    let unreachable = demand.len() - results.len();

    let assignments: Vec<FacilityAssignment> = results
        .into_iter()
        .map(|r| FacilityAssignment {
            demand_index: demand_index[&r.demand_node],
            facility_index: facility_index[&r.facility_node],
            demand_node: r.demand_node.0,
            facility_node: r.facility_node.0,
            duration_s: r.cost,
        })
        .collect();

    Ok(Json(ClosestFacilityResponse {
        assignments,
        unreachable,
    }))
}

async fn network_betweenness(
    State(state): State<AppState>,
    Json(req): Json<BetweennessRequest>,
) -> Result<Json<BetweennessResponse>, (StatusCode, Json<ErrorResponse>)> {
    ::metrics::counter!("itinera_network_betweenness_requests").increment(1);
    let profile = resolve_profile(req.profile.as_deref(), &state.profile)?;
    let top_k = resolve_top_k(req.top_k)?;

    let sample_size = req.sample_size.unwrap_or(DEFAULT_BETWEENNESS_SAMPLE);
    if sample_size == 0 || sample_size > MAX_BETWEENNESS_SAMPLE {
        return Err(bad_request(format!(
            "sample_size must be between 1 and {MAX_BETWEENNESS_SAMPLE}"
        )));
    }

    let scores = network_analysis::betweenness_centrality(&state.graph, &profile, sample_size);

    let mut nodes: Vec<CentralityEntry> = scores
        .into_iter()
        .filter_map(|(node, score)| {
            let coord = state.graph.node_coord(node)?;
            Some(CentralityEntry {
                node_id: node.0,
                lat: coord.lat,
                lon: coord.lon,
                score,
            })
        })
        .collect();
    nodes.sort_by(|a, b| {
        b.score
            .total_cmp(&a.score)
            .then_with(|| a.node_id.cmp(&b.node_id))
    });
    nodes.truncate(top_k);

    Ok(Json(BetweennessResponse {
        sampled_sources: sample_size.min(state.graph.num_nodes()),
        nodes,
    }))
}

fn check_points(name: &str, points: &[Point]) -> Result<(), (StatusCode, Json<ErrorResponse>)> {
    if points.is_empty() {
        return Err(bad_request(format!("{name} must not be empty")));
    }
    if points.len() > MAX_NETWORK_POINTS {
        return Err(bad_request(format!(
            "{name} has {} points, max {MAX_NETWORK_POINTS}",
            points.len()
        )));
    }
    Ok(())
}

fn check_pairs(pairs: usize) -> Result<(), (StatusCode, Json<ErrorResponse>)> {
    if pairs > MAX_NETWORK_PAIRS {
        return Err(bad_request(format!(
            "{pairs} pairs requested, max {MAX_NETWORK_PAIRS}"
        )));
    }
    Ok(())
}

fn resolve_top_k(top_k: Option<usize>) -> Result<usize, (StatusCode, Json<ErrorResponse>)> {
    let top_k = top_k.unwrap_or(DEFAULT_TOP_K);
    if top_k == 0 || top_k > MAX_TOP_K {
        return Err(bad_request(format!(
            "top_k must be between 1 and {MAX_TOP_K}"
        )));
    }
    Ok(top_k)
}

fn snap_points(
    graph: &Graph,
    points: &[Point],
) -> Result<Vec<NodeId>, (StatusCode, Json<ErrorResponse>)> {
    points
        .iter()
        .map(|p| {
            graph
                .nearest_node(Coord::new(p.lat, p.lon))
                .ok_or_else(|| bad_request(format!("no node found near {},{}", p.lat, p.lon)))
        })
        .collect()
}

/// Map each snapped node back to the first request point that produced it.
fn index_by_node(nodes: &[NodeId]) -> HashMap<NodeId, usize> {
    let mut map = HashMap::new();
    for (index, &node) in nodes.iter().enumerate() {
        map.entry(node).or_insert(index);
    }
    map
}
