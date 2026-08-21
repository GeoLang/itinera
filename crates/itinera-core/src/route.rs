use serde::{Deserialize, Serialize};

use itinera_graph::{Coord, Graph, NodeId, SpeedProfile};

use crate::maneuver::annotate_maneuvers;

/// A computed route with geometry and turn-by-turn instructions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Route {
    /// Total distance in meters.
    pub distance_m: f64,
    /// Total duration in seconds.
    pub duration_s: f64,
    /// Ordered node IDs along the path.
    pub node_ids: Vec<u32>,
    /// Route geometry (all coordinates along the path).
    pub geometry: Vec<Coord>,
    /// Turn-by-turn steps.
    pub steps: Vec<RouteStep>,
}

/// A single step in turn-by-turn navigation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteStep {
    /// Distance of this step in meters.
    pub distance_m: f64,
    /// Duration of this step in seconds.
    pub duration_s: f64,
    /// Road name (if available).
    pub name: Option<String>,
    /// Maneuver at the start of this step.
    pub maneuver: StepManeuver,
}

/// Maneuver type for navigation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum StepManeuver {
    Depart,
    Arrive,
    TurnLeft,
    TurnRight,
    TurnSlightLeft,
    TurnSlightRight,
    TurnSharpLeft,
    TurnSharpRight,
    Continue,
    UTurn,
    Roundabout { exit_number: u8 },
    Merge,
    Fork { direction: ForkDirection },
}

/// Direction of a fork.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ForkDirection {
    Left,
    Right,
}

/// Build a `Route` from an unpacked node path.
///
/// Distance and steps come from original graph edges. `duration_s` is the
/// caller-supplied travel time (search cost).
pub fn route_from_path(
    graph: &Graph,
    path: &[u32],
    profile: &SpeedProfile,
    duration_s: f64,
) -> Route {
    let geometry: Vec<_> = path
        .iter()
        .filter_map(|&nid| graph.node_coord(NodeId(nid)))
        .collect();

    let maneuvers = annotate_maneuvers(graph, path);
    let mut steps = Vec::new();
    let mut total_distance = 0.0;

    for (idx, window) in path.windows(2).enumerate() {
        let from = NodeId(window[0]);
        let to = NodeId(window[1]);

        if let Some(edge) = edge_between(graph, from, to) {
            total_distance += edge.distance_m;
            let maneuver = maneuvers[idx].clone();
            steps.push(RouteStep {
                distance_m: edge.distance_m,
                duration_s: graph.edge_weight(edge, profile),
                name: edge.name.clone(),
                maneuver,
            });
        }
    }

    if let Some(last) = steps.last_mut() {
        last.maneuver = StepManeuver::Arrive;
    }

    Route {
        distance_m: total_distance,
        duration_s,
        node_ids: path.to_vec(),
        geometry,
        steps,
    }
}

fn edge_between(graph: &Graph, from: NodeId, to: NodeId) -> Option<&itinera_graph::Edge> {
    let edges = graph.outgoing_edges(from);
    edges
        .iter()
        .find(|e| e.to == to && e.way_id >= 0)
        .or_else(|| edges.iter().find(|e| e.to == to))
}
