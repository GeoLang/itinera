//! # itinera-core
//!
//! Core routing algorithms: Dijkstra, A*, contraction hierarchies, isochrones.

mod astar;
mod ch;
mod dijkstra;
mod error;
mod isochrone;
mod maneuver;
pub mod network_analysis;
mod route;
pub mod vrp;

pub use astar::astar;
pub use ch::ContractionHierarchy;
pub use dijkstra::dijkstra;
pub use error::RoutingError;
pub use isochrone::isochrone;
pub use maneuver::{annotate_maneuvers, detect_maneuver};
pub use route::{Route, RouteStep, StepManeuver};
