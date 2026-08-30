//! # itinera-graph
//!
//! Compact, cache-friendly graph data structure for road networks.
//! Uses a compressed sparse row (CSR) representation for fast traversal.

mod coord;
mod edge;
mod graph;
mod node;
mod profile;
pub mod turn;

pub use coord::Coord;
pub use edge::{Edge, EdgeId, ROAD_CLASS_NAMES, road_class_name};
pub use graph::Graph;
pub use node::{Node, NodeId};
pub use profile::{SpeedProfile, TravelMode};
pub use turn::{TurnCost, TurnRestriction};
