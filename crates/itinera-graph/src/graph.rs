use std::sync::OnceLock;

use rstar::{RTree, primitives::GeomWithData};
use serde::{Deserialize, Serialize};

use crate::turn::RestrictionType;
use crate::{Coord, Edge, EdgeId, Node, NodeId, SpeedProfile, TurnRestriction};

type IndexedPoint = GeomWithData<[f64; 2], u32>;

/// Compressed Sparse Row (CSR) graph for fast adjacency traversal.
///
/// Edges are stored sorted by source node. `offsets[i]` gives the starting index
/// in the `edges` array for node `i`. Node `i`'s outgoing edges are
/// `edges[offsets[i]..offsets[i+1]]`.
///
/// Also maintains a reverse CSR for efficient incoming-edge traversal
/// (used by bidirectional CH queries).
#[derive(Debug, Serialize, Deserialize)]
pub struct Graph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
    /// CSR offset array: `offsets[i]` is the start index of node i's outgoing edges.
    /// Length = nodes.len() + 1.
    pub offsets: Vec<u32>,
    /// Reverse edge indices (sorted by target node).
    pub rev_edge_indices: Vec<u32>,
    /// Reverse CSR offset array: `rev_offsets[i]` is the start index of node i's
    /// incoming edges in `rev_edge_indices`. Length = nodes.len() + 1.
    pub rev_offsets: Vec<u32>,
    /// Turn restrictions indexed by via_node.
    pub restrictions: Vec<TurnRestriction>,
    /// Spatial index for `nearest_node`. Built once; skipped in serde.
    #[serde(skip)]
    rtree: OnceLock<RTree<IndexedPoint>>,
}

impl Clone for Graph {
    fn clone(&self) -> Self {
        Self {
            nodes: self.nodes.clone(),
            edges: self.edges.clone(),
            offsets: self.offsets.clone(),
            rev_edge_indices: self.rev_edge_indices.clone(),
            rev_offsets: self.rev_offsets.clone(),
            restrictions: self.restrictions.clone(),
            rtree: OnceLock::new(),
        }
    }
}

impl Graph {
    /// Build a graph from a set of nodes and edges.
    /// Edges must be provided with correct from/to NodeIds.
    #[must_use]
    pub fn build(mut nodes: Vec<Node>, mut edges: Vec<Edge>) -> Self {
        let n = nodes.len();
        // Sort nodes by id
        nodes.sort_by_key(|node| node.id);

        // Sort edges by source node
        edges.sort_by_key(|e| e.from);

        // Build CSR offsets
        let mut offsets = vec![0u32; n + 1];
        for edge in &edges {
            let src = edge.from.0 as usize;
            if src < n {
                offsets[src + 1] += 1;
            }
        }
        // Prefix sum
        for i in 1..=n {
            offsets[i] += offsets[i - 1];
        }

        // Build reverse CSR
        let mut rev_offsets = vec![0u32; n + 1];
        for edge in &edges {
            let tgt = edge.to.0 as usize;
            if tgt < n {
                rev_offsets[tgt + 1] += 1;
            }
        }
        for i in 1..=n {
            rev_offsets[i] += rev_offsets[i - 1];
        }

        // Build reverse edge index array (indices into `edges` sorted by target)
        let mut rev_edge_indices = vec![0u32; edges.len()];
        let mut rev_pos = rev_offsets.clone();
        for (idx, edge) in edges.iter().enumerate() {
            let tgt = edge.to.0 as usize;
            if tgt < n {
                let pos = rev_pos[tgt] as usize;
                rev_edge_indices[pos] = idx as u32;
                rev_pos[tgt] += 1;
            }
        }

        let graph = Self {
            nodes,
            edges,
            offsets,
            rev_edge_indices,
            rev_offsets,
            restrictions: Vec::new(),
            rtree: OnceLock::new(),
        };
        let _ = graph.spatial_index();
        graph
    }

    /// Rebuild the reverse CSR index. Call after adding shortcuts.
    pub fn rebuild_reverse_index(&mut self) {
        let n = self.nodes.len();
        let mut rev_offsets = vec![0u32; n + 1];
        for edge in &self.edges {
            let tgt = edge.to.0 as usize;
            if tgt < n {
                rev_offsets[tgt + 1] += 1;
            }
        }
        for i in 1..=n {
            rev_offsets[i] += rev_offsets[i - 1];
        }

        let mut rev_edge_indices = vec![0u32; self.edges.len()];
        let mut rev_pos = rev_offsets.clone();
        for (idx, edge) in self.edges.iter().enumerate() {
            let tgt = edge.to.0 as usize;
            if tgt < n {
                let pos = rev_pos[tgt] as usize;
                rev_edge_indices[pos] = idx as u32;
                rev_pos[tgt] += 1;
            }
        }

        self.rev_offsets = rev_offsets;
        self.rev_edge_indices = rev_edge_indices;
    }

    /// Number of nodes.
    #[must_use]
    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// Number of edges.
    #[must_use]
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    /// Get outgoing edges for a node.
    #[must_use]
    pub fn outgoing_edges(&self, node: NodeId) -> &[Edge] {
        let idx = node.0 as usize;
        if idx >= self.nodes.len() {
            return &[];
        }
        let start = self.offsets[idx] as usize;
        let end = self.offsets[idx + 1] as usize;
        &self.edges[start..end]
    }

    /// Get incoming edges for a node (via reverse CSR).
    pub fn incoming_edges(&self, node: NodeId) -> Vec<&Edge> {
        let idx = node.0 as usize;
        if idx >= self.nodes.len() {
            return Vec::new();
        }
        let start = self.rev_offsets[idx] as usize;
        let end = self.rev_offsets[idx + 1] as usize;
        self.rev_edge_indices[start..end]
            .iter()
            .map(|&ei| &self.edges[ei as usize])
            .collect()
    }

    /// Get the coordinate of a node.
    #[must_use]
    pub fn node_coord(&self, node: NodeId) -> Option<Coord> {
        let idx = node.0 as usize;
        self.nodes.get(idx).map(|n| n.coord)
    }

    /// Find nearest node to a coordinate using R-tree spatial index.
    #[must_use]
    pub fn nearest_node(&self, coord: Coord) -> Option<NodeId> {
        self.spatial_index()
            .nearest_neighbor(&[coord.lat, coord.lon])
            .map(|p| NodeId(p.data))
    }

    fn spatial_index(&self) -> &RTree<IndexedPoint> {
        self.rtree.get_or_init(|| {
            RTree::bulk_load(
                self.nodes
                    .iter()
                    .map(|n| GeomWithData::new([n.coord.lat, n.coord.lon], n.id.0))
                    .collect(),
            )
        })
    }

    /// Check if a turn from `from_way` to `to_way` at `via_node` is restricted.
    #[must_use]
    pub fn is_turn_restricted(&self, via_node: NodeId, from_way: i64, to_way: i64) -> bool {
        self.restrictions.iter().any(|r| {
            r.via_node == via_node
                && r.from_way == from_way
                && r.to_way == to_way
                && r.restriction_type == RestrictionType::No
        })
    }

    /// Whether the turn `from_way` → `to_way` at `via` is forbidden.
    ///
    /// Incoming way `0` means no previous way (start of search): never banned.
    /// `Only` restrictions at `(via, from_way)` ban every `to_way` except the named one.
    #[must_use]
    pub fn turn_is_banned(&self, via: NodeId, from_way: i64, to_way: i64) -> bool {
        if from_way == 0 || to_way == 0 {
            return false;
        }

        let mut has_only = false;
        let mut only_allows = false;
        for r in &self.restrictions {
            if r.via_node != via || r.from_way != from_way {
                continue;
            }
            match r.restriction_type {
                RestrictionType::No if r.to_way == to_way => return true,
                RestrictionType::Only => {
                    has_only = true;
                    if r.to_way == to_way {
                        only_allows = true;
                    }
                }
                RestrictionType::No => {}
            }
        }
        has_only && !only_allows
    }

    /// Calculate edge weight (travel time in seconds) given a speed profile.
    #[must_use]
    pub fn edge_weight(&self, edge: &Edge, profile: &SpeedProfile) -> f64 {
        let speed = profile.speed_for_class(edge.road_class);
        if speed <= 0.0 {
            return f64::INFINITY;
        }
        // distance_m / (speed_kmh * 1000 / 3600) = distance_m * 3.6 / speed_kmh
        edge.distance_m * 3.6 / speed
    }

    /// Add an edge to the graph (for building incrementally, e.g. CH shortcuts).
    /// Note: call `rebuild_reverse_index()` after all shortcuts are added.
    pub fn add_shortcut(
        &mut self,
        from: NodeId,
        to: NodeId,
        distance_m: f64,
        duration_s: f64,
    ) -> EdgeId {
        let id = EdgeId(self.edges.len() as u32);
        self.edges.push(Edge {
            from,
            to,
            distance_m,
            duration_s,
            way_id: -1,
            road_class: 0,
            oneway: true,
            name: None,
            geometry: Vec::new(),
        });
        id
    }

    /// Serialize graph to compact binary format using bincode.
    pub fn to_bytes(&self) -> Vec<u8> {
        bincode::serialize(self).expect("graph serialization should not fail")
    }

    /// Deserialize graph from compact binary format.
    pub fn from_bytes(data: &[u8]) -> Result<Self, bincode::Error> {
        bincode::deserialize(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_graph() -> Graph {
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: Coord::new(48.8566, 2.3522), // Paris
                osm_id: 1,
                ch_level: 0,
            },
            Node {
                id: NodeId(1),
                coord: Coord::new(48.8606, 2.3376), // Louvre
                osm_id: 2,
                ch_level: 0,
            },
            Node {
                id: NodeId(2),
                coord: Coord::new(48.8738, 2.2950), // Arc de Triomphe
                osm_id: 3,
                ch_level: 0,
            },
        ];

        let edges = vec![
            Edge {
                from: NodeId(0),
                to: NodeId(1),
                distance_m: 1200.0,
                duration_s: 72.0,
                way_id: 100,
                road_class: 5,
                oneway: false,
                name: Some("Rue de Rivoli".to_string()),
                geometry: Vec::new(),
            },
            Edge {
                from: NodeId(1),
                to: NodeId(2),
                distance_m: 3500.0,
                duration_s: 210.0,
                way_id: 101,
                road_class: 3,
                oneway: false,
                name: Some("Avenue des Champs-Élysées".to_string()),
                geometry: Vec::new(),
            },
            Edge {
                from: NodeId(0),
                to: NodeId(2),
                distance_m: 5000.0,
                duration_s: 300.0,
                way_id: 102,
                road_class: 4,
                oneway: true,
                name: None,
                geometry: Vec::new(),
            },
        ];

        Graph::build(nodes, edges)
    }

    #[test]
    fn test_graph_structure() {
        let g = sample_graph();
        assert_eq!(g.num_nodes(), 3);
        assert_eq!(g.num_edges(), 3);
    }

    #[test]
    fn test_outgoing_edges() {
        let g = sample_graph();
        let edges = g.outgoing_edges(NodeId(0));
        assert_eq!(edges.len(), 2);
        assert_eq!(edges[0].to, NodeId(1));
        assert_eq!(edges[1].to, NodeId(2));
    }

    #[test]
    fn test_nearest_node() {
        let g = sample_graph();
        // Closest to Paris center
        let nearest = g.nearest_node(Coord::new(48.857, 2.352)).unwrap();
        assert_eq!(nearest, NodeId(0));
    }

    #[test]
    fn test_nearest_node_builds_rtree_once() {
        let g = sample_graph();
        assert!(g.rtree.get().is_some());
        let first = g.nearest_node(Coord::new(48.857, 2.352)).unwrap();
        let second = g.nearest_node(Coord::new(48.861, 2.338)).unwrap();
        assert_eq!(first, NodeId(0));
        assert_eq!(second, NodeId(1));
        assert!(g.rtree.get().is_some());

        let restored = Graph::from_bytes(&g.to_bytes()).unwrap();
        assert!(restored.rtree.get().is_none());
        assert_eq!(
            restored.nearest_node(Coord::new(48.857, 2.352)).unwrap(),
            NodeId(0)
        );
        assert!(restored.rtree.get().is_some());

        let cloned = g.clone();
        assert!(cloned.rtree.get().is_none());
        assert_eq!(
            cloned.nearest_node(Coord::new(48.857, 2.352)).unwrap(),
            NodeId(0)
        );
        assert!(cloned.rtree.get().is_some());
    }

    #[test]
    fn test_turn_is_banned_no_and_only() {
        let mut g = sample_graph();
        g.restrictions.push(crate::TurnRestriction {
            via_node: NodeId(1),
            from_way: 100,
            to_way: 101,
            restriction_type: RestrictionType::No,
        });
        g.restrictions.push(crate::TurnRestriction {
            via_node: NodeId(0),
            from_way: 200,
            to_way: 201,
            restriction_type: RestrictionType::Only,
        });

        assert!(g.turn_is_banned(NodeId(1), 100, 101));
        assert!(!g.turn_is_banned(NodeId(1), 100, 102));
        assert!(!g.turn_is_banned(NodeId(1), 0, 101));
        assert!(!g.turn_is_banned(NodeId(1), 100, 0));

        assert!(!g.turn_is_banned(NodeId(0), 200, 201));
        assert!(g.turn_is_banned(NodeId(0), 200, 202));
        assert!(!g.turn_is_banned(NodeId(0), 0, 202));
    }

    #[test]
    fn test_haversine() {
        let paris = Coord::new(48.8566, 2.3522);
        let louvre = Coord::new(48.8606, 2.3376);
        let dist = paris.distance_to(louvre);
        // Approximately 1.1 km
        assert!(dist > 1000.0 && dist < 1200.0);
    }

    #[test]
    fn test_edge_weight() {
        let g = sample_graph();
        let profile = SpeedProfile::car();
        let edge = &g.outgoing_edges(NodeId(0))[0];
        let weight = g.edge_weight(edge, &profile);
        // 1200m at 50 km/h = 1200 * 3.6 / 50 = 86.4 seconds
        assert!((weight - 86.4).abs() < 0.1);
    }
}
