//! HTTP tests for the map matching endpoint.

use axum::body::Body;
use axum::http::{Request, StatusCode};
use http_body_util::BodyExt;
use itinera_graph::{Coord, Edge, Graph, Node, NodeId, SpeedProfile};
use itinera_server::{AppState, router};
use serde_json::{Value, json};
use tower::ServiceExt;

const WEST_SIDE: &str = "West Side";
const NORTH_SIDE: &str = "North Side";
const EAST_SIDE: &str = "East Side";
const SOUTH_SIDE: &str = "South Side";

fn node(id: u32, lat: f64, lon: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: Coord::new(lat, lon),
        osm_id: i64::from(id) + 100,
        ch_level: 0,
    }
}

/// Bidirectional edge pair, both directions carrying the same name and way id.
fn bidi(from: u32, to: u32, distance_m: f64, way_id: i64, name: &str) -> Vec<Edge> {
    let edge = |from: u32, to: u32| Edge {
        from: NodeId(from),
        to: NodeId(to),
        distance_m,
        duration_s: 0.0,
        way_id,
        road_class: 5,
        oneway: false,
        name: Some(name.to_string()),
        geometry: Vec::new(),
    };
    vec![edge(from, to), edge(to, from)]
}

/// Diamond graph with a named road on each side.
///
/// ```text
///   1 -- North -- 3
///   |             |
///  West          East
///   |             |
///   0 -- South -- 2
/// ```
fn diamond() -> Graph {
    let nodes = vec![
        node(0, 48.00, 2.00),
        node(1, 48.01, 2.00),
        node(2, 48.00, 2.01),
        node(3, 48.01, 2.01),
    ];
    let mut edges = bidi(0, 1, 1000.0, 1, WEST_SIDE);
    edges.extend(bidi(1, 3, 1000.0, 2, NORTH_SIDE));
    edges.extend(bidi(0, 2, 3000.0, 3, SOUTH_SIDE));
    edges.extend(bidi(2, 3, 3000.0, 4, EAST_SIDE));
    Graph::build(nodes, edges)
}

async fn post_match(body: Value) -> (StatusCode, Value) {
    let request = Request::builder()
        .method("POST")
        .uri("/match")
        .header("content-type", "application/json")
        .body(Body::from(body.to_string()))
        .unwrap();
    let app = router(AppState::new(diamond(), SpeedProfile::car()));
    let response = app.oneshot(request).await.unwrap();
    let status = response.status();
    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    (status, serde_json::from_slice(&bytes).unwrap())
}

fn trace_point(lat: f64, lon: f64) -> Value {
    json!({ "lat": lat, "lon": lon })
}

#[tokio::test]
async fn trace_along_one_side_matches_that_road() {
    let (status, body) = post_match(json!({
        "trace": [
            trace_point(48.0020, 2.00001),
            trace_point(48.0050, 2.00000),
            trace_point(48.0080, 2.00002),
        ],
    }))
    .await;

    assert_eq!(status, StatusCode::OK);
    assert!(body["confidence"].as_f64().unwrap() > 0.0);
    assert!(body["matched_route"].as_array().unwrap().len() >= 2);

    let names: Vec<&str> = body["matched_points"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| p["road_name"].as_str().unwrap())
        .collect();
    assert_eq!(names, vec![WEST_SIDE, WEST_SIDE, WEST_SIDE]);

    let roads = body["road_segments"].as_array().unwrap();
    assert_eq!(roads.len(), 1);
    assert_eq!(roads[0]["road_name"], WEST_SIDE);
    assert_eq!(roads[0]["road_class"], "tertiary");
    assert_eq!(roads[0]["speed_kmh"].as_f64().unwrap(), 50.0);
    assert!(body["total_distance_m"].as_f64().unwrap() > 0.0);
}

#[tokio::test]
async fn match_rejects_empty_trace() {
    let (status, body) = post_match(json!({ "trace": [] })).await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "trace must not be empty");
}

#[tokio::test]
async fn match_rejects_overlong_trace() {
    let trace = vec![trace_point(48.0050, 2.00000); 1001];
    let (status, body) = post_match(json!({ "trace": trace })).await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "trace has 1001 points, max 1000");
}

#[tokio::test]
async fn match_rejects_unknown_profile() {
    let (status, body) = post_match(json!({
        "trace": [trace_point(48.0050, 2.00000)],
        "profile": "hovercraft",
    }))
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
async fn match_rejects_a_search_radius_outside_the_allowed_range() {
    let (status, body) = post_match(json!({
        "trace": [trace_point(48.0050, 2.00000)],
        "search_radius_m": 5000.0,
    }))
    .await;

    assert_eq!(status, StatusCode::BAD_REQUEST);
    assert_eq!(
        body["error"],
        "search_radius_m must be greater than 0 and at most 1000"
    );
}
