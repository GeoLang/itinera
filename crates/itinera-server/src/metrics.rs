//! Prometheus metrics endpoint.

use axum::response::IntoResponse;
use metrics_exporter_prometheus::{PrometheusBuilder, PrometheusHandle};
use std::sync::OnceLock;

static PROMETHEUS_HANDLE: OnceLock<PrometheusHandle> = OnceLock::new();

/// Install the Prometheus metrics recorder. Call once at startup.
pub fn install() {
    // get_or_init keeps this a no-op if a router was already built
    PROMETHEUS_HANDLE.get_or_init(|| {
        PrometheusBuilder::new()
            .install_recorder()
            .expect("failed to install Prometheus recorder")
    });

    metrics::describe_counter!("itinera_requests_total", "Total HTTP requests");
    metrics::describe_counter!("itinera_route_requests", "Route calculation requests");
    metrics::describe_counter!("itinera_nearest_requests", "Nearest-node requests");
    metrics::describe_counter!("itinera_isochrone_requests", "Isochrone requests");
    metrics::describe_counter!(
        "itinera_delivery_requests",
        "Delivery optimization requests"
    );
    metrics::describe_counter!(
        "itinera_network_components_requests",
        "Connected component requests"
    );
    metrics::describe_counter!("itinera_network_od_matrix_requests", "OD matrix requests");
    metrics::describe_counter!(
        "itinera_network_closest_facility_requests",
        "Closest facility requests"
    );
    metrics::describe_counter!(
        "itinera_network_betweenness_requests",
        "Betweenness centrality requests"
    );
    metrics::describe_histogram!(
        "itinera_route_duration_seconds",
        "Route calculation duration in seconds"
    );
    metrics::describe_gauge!("itinera_graph_nodes", "Number of nodes in the graph");
    metrics::describe_gauge!("itinera_graph_edges", "Number of edges in the graph");
}

/// Handler for GET /metrics — serves Prometheus text format.
pub async fn metrics_handler() -> impl IntoResponse {
    let output = match PROMETHEUS_HANDLE.get() {
        Some(handle) => handle.render(),
        None => "# HELP itinera_up Server is running\nitinera_up 1\n".to_string(),
    };
    (
        [(
            axum::http::header::CONTENT_TYPE,
            "text/plain; version=0.0.4; charset=utf-8",
        )],
        output,
    )
}
