//! HTTP tests for the network analysis endpoints.

use axum::body::Body;
use axum::http::{Request, StatusCode};
use http_body_util::BodyExt;
use itinera_graph::{Coord, Edge, Graph, Node, NodeId, SpeedProfile};
use itinera_server::{AppState, router};
use serde_json::{Value, json};
use tower::ServiceExt;

fn node(id: u32, lat: f64, lon: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: Coord::new(lat, lon),
        osm_id: i64::from(id) + 100,
        ch_level: 0,
    }
}

/// Bidirectional edge pair. road_class 5 is 50 km/h for the car profile,
/// so the travel time in seconds is `distance_m * 3.6 / 50`.
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

/// Diamond graph. The 0-1-3 side costs 72 s per hop, the 0-2-3 side 216 s,
/// so every shortest path between 0 and 3 runs through node 1.
///
/// ```text
///   1 --- 3
///   |     |
///   0 --- 2
/// ```
fn diamond() -> Graph {
    let nodes = vec![
        node(0, 48.00, 2.00),
        node(1, 48.01, 2.00),
        node(2, 48.00, 2.01),
        node(3, 48.01, 2.01),
    ];
    let mut edges = bidi(0, 1, 1000.0, 1);
    edges.extend(bidi(1, 3, 1000.0, 2));
    edges.extend(bidi(0, 2, 3000.0, 3));
    edges.extend(bidi(2, 3, 3000.0, 4));
    Graph::build(nodes, edges)
}

/// Two disjoint pairs: {0, 1} near 48.0 and {2, 3} near 49.0.
fn two_components() -> Graph {
    let nodes = vec![
        node(0, 48.00, 2.00),
        node(1, 48.01, 2.00),
        node(2, 49.00, 3.00),
        node(3, 49.01, 3.00),
    ];
    let mut edges = bidi(0, 1, 1000.0, 1);
    edges.extend(bidi(2, 3, 1000.0, 2));
    Graph::build(nodes, edges)
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

async fn post_json(graph: Graph, path: &str, body: Value) -> (StatusCode, Value) {
    let request = Request::builder()
        .method("POST")
        .uri(path)
        .header("content-type", "application/json")
        .body(Body::from(body.to_string()))
        .unwrap();
    send(graph, request).await
}

async fn post_empty(graph: Graph, path: &str) -> (StatusCode, Value) {
    let request = Request::builder()
        .method("POST")
        .uri(path)
        .body(Body::empty())
        .unwrap();
    send(graph, request).await
}

fn point(lat: f64, lon: f64) -> Value {
    json!({ "lat": lat, "lon": lon })
}

#[tokio::test]
async fn components_defaults_when_no_body_is_sent() {
    let (status, body) = post_empty(diamond(), "/network/components").await;

    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["num_nodes"], 4);
    assert_eq!(body["num_components"], 1);
    assert_eq!(body["largest_component_size"], 4);
    assert_eq!(body["components"].as_array().unwrap().len(), 1);
    assert_eq!(body["components"][0]["size"], 4);
}

#[tokio::test]
async fn components_reports_each_disconnected_part() {
    let (status, body) = post_json(two_components(), "/network/components", json!({})).await;

    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["num_components"], 2);
    assert_eq!(body["largest_component_size"], 2);
    let sizes: Vec<u64> = body["components"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| c["size"].as_u64().unwrap())
        .collect();
    assert_eq!(sizes, vec![2, 2]);
}

#[tokio::test]
async fn components_rejects_zero_top_k() {
    let (status, body) = post_json(diamond(), "/network/components", json!({ "top_k": 0 })).await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "top_k must be between 1 and 1000");
}

#[tokio::test]
async fn od_matrix_returns_travel_time_per_pair() {
    let (status, body) = post_json(
        diamond(),
        "/network/od-matrix",
        json!({
            "origins": [point(48.00, 2.00)],
            "destinations": [point(48.01, 2.00), point(48.01, 2.01)],
        }),
    )
    .await;

    assert_eq!(status, StatusCode::OK);
    let entries = body["entries"].as_array().unwrap();
    assert_eq!(entries.len(), 2);

    assert_eq!(entries[0]["origin_index"], 0);
    assert_eq!(entries[0]["destination_index"], 0);
    assert_eq!(entries[0]["origin_node"], 0);
    assert_eq!(entries[0]["destination_node"], 1);
    assert_eq!(entries[0]["duration_s"].as_f64().unwrap(), 72.0);

    // 0 -> 3 goes through node 1, two hops of 72 s
    assert_eq!(entries[1]["destination_index"], 1);
    assert_eq!(entries[1]["destination_node"], 3);
    assert_eq!(entries[1]["duration_s"].as_f64().unwrap(), 144.0);
}

#[tokio::test]
async fn od_matrix_rejects_too_many_pairs() {
    let origins = vec![point(48.00, 2.00); 60];
    let destinations = vec![point(48.01, 2.01); 60];
    let (status, body) = post_json(
        diamond(),
        "/network/od-matrix",
        json!({ "origins": origins, "destinations": destinations }),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "3600 pairs requested, max 2500");
}

#[tokio::test]
async fn od_matrix_rejects_empty_origins() {
    let (status, body) = post_json(
        diamond(),
        "/network/od-matrix",
        json!({ "origins": [], "destinations": [point(48.01, 2.00)] }),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "origins must not be empty");
}

#[tokio::test]
async fn od_matrix_rejects_unknown_profile() {
    let (status, body) = post_json(
        diamond(),
        "/network/od-matrix",
        json!({
            "origins": [point(48.00, 2.00)],
            "destinations": [point(48.01, 2.00)],
            "profile": "hovercraft",
        }),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert!(
        body["error"]
            .as_str()
            .unwrap()
            .starts_with("unknown profile 'hovercraft'")
    );
}

#[tokio::test]
async fn closest_facility_picks_the_cheapest_facility() {
    let (status, body) = post_json(
        diamond(),
        "/network/closest-facility",
        json!({
            "demand_points": [point(48.00, 2.00)],
            "facilities": [point(48.01, 2.01), point(48.01, 2.00)],
        }),
    )
    .await;

    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["unreachable"], 0);
    let assignments = body["assignments"].as_array().unwrap();
    assert_eq!(assignments.len(), 1);
    assert_eq!(assignments[0]["demand_index"], 0);
    assert_eq!(assignments[0]["demand_node"], 0);
    // node 1 at 72 s beats node 3 at 144 s
    assert_eq!(assignments[0]["facility_index"], 1);
    assert_eq!(assignments[0]["facility_node"], 1);
    assert_eq!(assignments[0]["duration_s"].as_f64().unwrap(), 72.0);
}

#[tokio::test]
async fn closest_facility_counts_unreachable_demand() {
    let (status, body) = post_json(
        two_components(),
        "/network/closest-facility",
        json!({
            "demand_points": [point(49.00, 3.00)],
            "facilities": [point(48.00, 2.00)],
        }),
    )
    .await;

    assert_eq!(status, StatusCode::OK);
    assert!(body["assignments"].as_array().unwrap().is_empty());
    assert_eq!(body["unreachable"], 1);
}

#[tokio::test]
async fn betweenness_ranks_the_shared_intermediate_node() {
    let (status, body) = post_json(
        diamond(),
        "/network/betweenness",
        json!({ "sample_size": 4, "top_k": 4 }),
    )
    .await;

    assert_eq!(status, StatusCode::OK);
    assert_eq!(body["sampled_sources"], 4);
    let nodes = body["nodes"].as_array().unwrap();
    assert_eq!(nodes.len(), 4);

    // node 1 carries both 0 -> 3 and 3 -> 0
    assert_eq!(nodes[0]["node_id"], 1);
    assert_eq!(nodes[0]["score"].as_f64().unwrap(), 2.0);
    assert_eq!(nodes[0]["lat"].as_f64().unwrap(), 48.01);
    assert_eq!(nodes[0]["lon"].as_f64().unwrap(), 2.00);

    // node 2 is on the expensive side and carries nothing
    let node_2 = nodes.iter().find(|n| n["node_id"] == 2).unwrap();
    assert_eq!(node_2["score"].as_f64().unwrap(), 0.0);
}

#[tokio::test]
async fn betweenness_honours_top_k() {
    let (status, body) = post_json(
        diamond(),
        "/network/betweenness",
        json!({ "sample_size": 4, "top_k": 1 }),
    )
    .await;

    assert_eq!(status, StatusCode::OK);
    let nodes = body["nodes"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["node_id"], 1);
}

#[tokio::test]
async fn betweenness_rejects_zero_sample_size() {
    let (status, body) = post_json(
        diamond(),
        "/network/betweenness",
        json!({ "sample_size": 0 }),
    )
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "sample_size must be between 1 and 1000");
}
