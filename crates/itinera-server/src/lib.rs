//! # itinera-server
//!
//! HTTP API server for Itinera routing engine.
//! Provides OSRM-compatible route, isochrone, and nearest endpoints.
//!
//! Enterprise features: Prometheus metrics, JWT auth, health/readiness probes.

pub mod auth;
mod handlers;
pub mod metrics;
mod state;

pub use handlers::router;
pub use state::AppState;

/// Initialise the tracing subscriber (call once at startup).
pub fn init_tracing() {
    use tracing_subscriber::{EnvFilter, fmt};
    fmt()
        .with_env_filter(
            EnvFilter::from_default_env().add_directive("itinera=info".parse().unwrap()),
        )
        .init();
    tracing::info!("tracing initialised");
}
