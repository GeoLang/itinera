use std::collections::HashSet;
use std::sync::Arc;

use itinera_core::ContractionHierarchy;
use itinera_graph::{Graph, SpeedProfile};
use itinera_match::{RoadNetwork, RoadSegment};
use uuid::Uuid;

/// Names for `Edge::road_class`, index 0 being the class the OSM import gives a
/// way it does not recognise.
const ROAD_CLASS_NAMES: [&str; 8] = [
    "unknown",
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
];

const UNNAMED_ROAD: &str = "unnamed";

/// Shared application state for the HTTP server.
#[derive(Clone)]
pub struct AppState {
    pub graph: Arc<Graph>,
    pub profile: SpeedProfile,
    /// Optional pre-built contraction hierarchy for fast queries.
    pub ch: Option<Arc<ContractionHierarchy>>,
    /// The graph rebuilt as matcher segments, for `POST /match`.
    pub road_network: Arc<RoadNetwork>,
}

impl AppState {
    #[must_use]
    pub fn new(graph: Graph, profile: SpeedProfile) -> Self {
        let road_network = Arc::new(road_network_from_graph(&graph, &profile));
        Self {
            graph: Arc::new(graph),
            profile,
            ch: None,
            road_network,
        }
    }

    #[must_use]
    pub fn with_ch(mut self, ch: ContractionHierarchy) -> Self {
        self.ch = Some(Arc::new(ch));
        self
    }
}

/// One matcher segment per road, geometry in [lon, lat] order. A two-way road is
/// a pair of opposing edges, so the second one to come up is dropped.
fn road_network_from_graph(graph: &Graph, profile: &SpeedProfile) -> RoadNetwork {
    let mut converted: HashSet<(u32, u32, i64)> = HashSet::new();
    let mut segments = Vec::new();

    for edge in &graph.edges {
        let undirected = (
            edge.from.0.min(edge.to.0),
            edge.from.0.max(edge.to.0),
            edge.way_id,
        );
        if !edge.oneway && !converted.insert(undirected) {
            continue;
        }
        let (Some(from), Some(to)) = (graph.node_coord(edge.from), graph.node_coord(edge.to))
        else {
            continue;
        };

        let mut geometry = Vec::with_capacity(edge.geometry.len() + 2);
        geometry.push([from.lon, from.lat]);
        geometry.extend(edge.geometry.iter().map(|coord| [coord.lon, coord.lat]));
        geometry.push([to.lon, to.lat]);

        segments.push(RoadSegment {
            id: Uuid::new_v4(),
            name: edge
                .name
                .clone()
                .unwrap_or_else(|| UNNAMED_ROAD.to_string()),
            road_class: road_class_name(edge.road_class).to_string(),
            geometry,
            speed_limit_kmh: profile.speed_for_class(edge.road_class),
            oneway: edge.oneway,
        });
    }

    RoadNetwork::new(segments)
}

fn road_class_name(road_class: u8) -> &'static str {
    ROAD_CLASS_NAMES
        .get(road_class as usize)
        .copied()
        .unwrap_or(ROAD_CLASS_NAMES[0])
}
