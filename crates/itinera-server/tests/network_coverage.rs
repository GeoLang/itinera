use axum::body::Body;
use axum::http::{Request, StatusCode};
use http_body_util::BodyExt;
use itinera_graph::{BoundingBox, Coord, Edge, Graph, Node, NodeId, SpeedProfile};
use itinera_server::{AppState, router};
use serde_json::{Value, json};
use tower::ServiceExt;

// Toronto, far outside either coverage the tests build
const FAR_LAT: f64 = 43.647;
const FAR_LON: f64 = -79.41;
// the node extent grown by the longest edge, 1 km
const PADDED_NODE_BOUNDS: &str = "(lon 1.99 to 2.02, lat 47.99 to 48.02)";
const DECLARED_BOUNDS: &str = "(lon 1.90 to 2.10, lat 47.90 to 48.10)";

fn node(id: u32, lat: f64, lon: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: Coord::new(lat, lon),
        osm_id: i64::from(id) + 100,
        ch_level: 0,
    }
}

fn bidi(from: u32, to: u32, distance_m: f64, way_id: i64) -> Vec<Edge> {
    let edge = |from: u32, to: u32| Edge {
        from: NodeId(from),
        to: NodeId(to),
        distance_m,
        duration_s: 0.0,
        way_id,
        road_class: 5,
        oneway: false,
        name: None,
        geometry: Vec::new(),
    };
    vec![edge(from, to), edge(to, from)]
}

// 1 km per side, nodes only at the corners, no header bbox
fn square() -> Graph {
    let nodes = vec![
        node(0, 48.00, 2.00),
        node(1, 48.01, 2.00),
        node(2, 48.00, 2.01),
        node(3, 48.01, 2.01),
    ];
    let mut edges = bidi(0, 1, 1000.0, 1);
    edges.extend(bidi(1, 3, 1000.0, 2));
    edges.extend(bidi(0, 2, 1000.0, 3));
    edges.extend(bidi(2, 3, 1000.0, 4));
    Graph::build(nodes, edges)
}

// the same roads, imported from an extract whose header declares a much wider area
fn square_with_declared_bounds() -> Graph {
    let mut graph = square();
    graph.declared_bounds = Some(BoundingBox::from_corners(47.90, 1.90, 48.10, 2.10));
    graph
}

async fn send(graph: Graph, request: Request<Body>) -> (StatusCode, Value) {
    let app = router(AppState::new(graph, SpeedProfile::car()));
    let response = app.oneshot(request).await.unwrap();
    let status = response.status();
    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    let body = if bytes.is_empty() {
        Value::Null
    } else {
        serde_json::from_slice(&bytes).unwrap()
    };
    (status, body)
}

async fn get(graph: Graph, uri: &str) -> (StatusCode, Value) {
    let request = Request::builder().uri(uri).body(Body::empty()).unwrap();
    send(graph, request).await
}

async fn post_json(graph: Graph, path: &str, body: Value) -> (StatusCode, Value) {
    let request = Request::builder()
        .method("POST")
        .uri(path)
        .header("content-type", "application/json")
        .body(Body::from(body.to_string()))
        .unwrap();
    send(graph, request).await
}

fn point(lat: f64, lon: f64) -> Value {
    json!({ "lat": lat, "lon": lon })
}

#[tokio::test]
async fn route_refuses_an_origin_outside_the_coverage() {
    let (status, body) = get(
        square(),
        &format!("/route?from={FAR_LAT},{FAR_LON}&to=48.01,2.00"),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(
        body["error"],
        format!("origin 43.647, -79.410 is outside the loaded road network {PADDED_NODE_BOUNDS}")
    );
}

#[tokio::test]
async fn route_refuses_a_destination_outside_the_coverage() {
    let (status, body) = get(
        square(),
        &format!("/route?from=48.00,2.00&to={FAR_LAT},{FAR_LON}"),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(
        body["error"],
        format!(
            "destination 43.647, -79.410 is outside the loaded road network {PADDED_NODE_BOUNDS}"
        )
    );
}

#[tokio::test]
async fn route_accepts_points_inside_the_coverage() {
    let (status, body) = get(square(), "/route?from=48.0002,2.0001&to=48.0098,2.0001").await;

    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["distance_m"].as_f64().unwrap(), 1000.0);
}

#[tokio::test]
async fn isochrone_refuses_an_origin_outside_the_coverage() {
    let (status, body) = get(
        square(),
        &format!("/isochrone?lat={FAR_LAT}&lon={FAR_LON}&max_seconds=600"),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(
        body["error"],
        format!("origin 43.647, -79.410 is outside the loaded road network {PADDED_NODE_BOUNDS}")
    );
}

#[tokio::test]
async fn isochrone_refuses_an_origin_past_the_padded_node_extent() {
    let (status, body) = get(square(), "/isochrone?lat=48.10&lon=2.00&max_seconds=600").await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(
        body["error"],
        format!("origin 48.100, 2.000 is outside the loaded road network {PADDED_NODE_BOUNDS}")
    );
}

#[tokio::test]
async fn route_accepts_a_point_past_the_nodes_but_within_one_edge_of_them() {
    let (status, body) = get(square(), "/route?from=48.015,2.002&to=48.00,2.00").await;

    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["distance_m"].as_f64().unwrap(), 1000.0);
}

#[tokio::test]
async fn isochrone_accepts_an_origin_inside_the_coverage() {
    let (status, body) = get(
        square(),
        "/isochrone?lat=48.0002&lon=2.0001&max_seconds=600",
    )
    .await;

    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["reachable_nodes"], 4);
}

#[tokio::test]
async fn declared_bounds_widen_the_coverage_past_the_nodes() {
    let (status, body) = get(
        square_with_declared_bounds(),
        "/isochrone?lat=48.05&lon=2.05&max_seconds=600",
    )
    .await;

    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["reachable_nodes"], 4);
}

#[tokio::test]
async fn a_refusal_names_the_declared_bounds() {
    let (status, body) = get(
        square_with_declared_bounds(),
        "/isochrone?lat=48.20&lon=2.00&max_seconds=600",
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(
        body["error"],
        format!("origin 48.200, 2.000 is outside the loaded road network {DECLARED_BOUNDS}")
    );
}

#[tokio::test]
async fn od_matrix_refuses_an_origin_outside_the_coverage() {
    let (status, body) = post_json(
        square(),
        "/network/od-matrix",
        json!({
            "origins": [point(FAR_LAT, FAR_LON)],
            "destinations": [point(48.01, 2.00)],
        }),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(
        body["error"],
        format!("origin 43.647, -79.410 is outside the loaded road network {PADDED_NODE_BOUNDS}")
    );
}

#[tokio::test]
async fn od_matrix_refuses_a_destination_outside_the_coverage() {
    let (status, body) = post_json(
        square(),
        "/network/od-matrix",
        json!({
            "origins": [point(48.00, 2.00)],
            "destinations": [point(FAR_LAT, FAR_LON)],
        }),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(
        body["error"],
        format!(
            "destination 43.647, -79.410 is outside the loaded road network {PADDED_NODE_BOUNDS}"
        )
    );
}

#[tokio::test]
async fn od_matrix_accepts_points_inside_the_coverage() {
    let (status, body) = post_json(
        square(),
        "/network/od-matrix",
        json!({
            "origins": [point(48.0002, 2.0001)],
            "destinations": [point(48.0098, 2.0001)],
        }),
    )
    .await;

    assert_eq!(status, StatusCode::OK);
    let entries = body["entries"].as_array().unwrap();
    assert_eq!(entries.len(), 1);
    assert_eq!(entries[0]["origin_node"], 0);
    assert_eq!(entries[0]["destination_node"], 1);
}

#[tokio::test]
async fn nearest_still_answers_for_a_point_outside_the_coverage() {
    let (status, body) = get(square(), &format!("/nearest?lat={FAR_LAT}&lon={FAR_LON}")).await;

    assert_eq!(status, StatusCode::OK);
    assert!(body["distance_m"].as_f64().unwrap() > 6_000_000.0);
}
