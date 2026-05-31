//! Network analysis tools beyond basic routing.
//!
//! Provides graph-level analytics like:
//! - Connected components (strongly/weakly connected)
//! - Origin-Destination (OD) cost matrix
//! - Closest facility analysis
//! - Network betweenness centrality (approximate)

use std::collections::{HashMap, HashSet, VecDeque};

use itinera_graph::{Graph, NodeId, SpeedProfile};

use crate::dijkstra::dijkstra;
use crate::error::RoutingError;

/// Result of connected component analysis.
#[derive(Debug, Clone)]
pub struct ComponentResult {
    /// Component ID for each node (indexed by node internal index).
    pub node_component: Vec<u32>,
    /// Number of distinct components.
    pub num_components: u32,
    /// Size of each component (component_id → node count).
    pub component_sizes: HashMap<u32, usize>,
}

/// A single OD matrix entry.
#[derive(Debug, Clone)]
pub struct OdEntry {
    pub origin: NodeId,
    pub destination: NodeId,
    /// Travel cost (time in seconds or distance in meters depending on profile).
    pub cost: f64,
}

/// Closest facility result.
#[derive(Debug, Clone)]
pub struct ClosestFacility {
    pub demand_node: NodeId,
    pub facility_node: NodeId,
    pub cost: f64,
}

/// Compute weakly connected components of the graph.
///
/// Treats all edges as undirected for connectivity analysis.
pub fn connected_components(graph: &Graph) -> ComponentResult {
    let n = graph.num_nodes();
    let mut component_id = vec![u32::MAX; n];
    let mut current_component = 0u32;
    let mut component_sizes: HashMap<u32, usize> = HashMap::new();

    for start in 0..n {
        if component_id[start] != u32::MAX {
            continue;
        }

        // BFS from this node
        let mut queue = VecDeque::new();
        queue.push_back(start);
        component_id[start] = current_component;
        let mut size = 0usize;

        while let Some(node_idx) = queue.pop_front() {
            size += 1;
            let node = NodeId(node_idx as u32);

            // Forward edges
            for edge in graph.outgoing_edges(node) {
                let neighbor = edge.to.0 as usize;
                if component_id[neighbor] == u32::MAX {
                    component_id[neighbor] = current_component;
                    queue.push_back(neighbor);
                }
            }

            // Backward edges (treat as undirected)
            for edge in graph.incoming_edges(node) {
                let neighbor = edge.from.0 as usize;
                if component_id[neighbor] == u32::MAX {
                    component_id[neighbor] = current_component;
                    queue.push_back(neighbor);
                }
            }
        }

        component_sizes.insert(current_component, size);
        current_component += 1;
    }

    ComponentResult {
        node_component: component_id,
        num_components: current_component,
        component_sizes,
    }
}

/// Compute the Origin-Destination cost matrix for given origin/destination sets.
///
/// Returns travel costs between all origin-destination pairs that are reachable.
pub fn od_matrix(
    graph: &Graph,
    origins: &[NodeId],
    destinations: &[NodeId],
    profile: &SpeedProfile,
) -> Vec<OdEntry> {
    let dest_set: HashSet<NodeId> = destinations.iter().copied().collect();
    let mut results = Vec::new();

    for &origin in origins {
        // Run Dijkstra from origin to all destinations
        for &dest in &dest_set {
            if let Ok(route) = dijkstra(graph, origin, dest, profile) {
                results.push(OdEntry {
                    origin,
                    destination: dest,
                    cost: route.duration_s,
                });
            }
        }
    }

    results
}

/// Find the closest facility for each demand point.
///
/// For each demand node, finds the nearest facility (by travel cost).
pub fn closest_facility(
    graph: &Graph,
    demand_nodes: &[NodeId],
    facilities: &[NodeId],
    profile: &SpeedProfile,
) -> Vec<ClosestFacility> {
    let mut results = Vec::with_capacity(demand_nodes.len());

    for &demand in demand_nodes {
        let mut best: Option<ClosestFacility> = None;

        for &facility in facilities {
            if let Ok(route) = dijkstra(graph, demand, facility, profile) {
                let cost = route.duration_s;
                if best.as_ref().is_none_or(|b| cost < b.cost) {
                    best = Some(ClosestFacility {
                        demand_node: demand,
                        facility_node: facility,
                        cost,
                    });
                }
            }
        }

        if let Some(b) = best {
            results.push(b);
        }
    }

    results
}

/// Approximate betweenness centrality using a sample of source nodes.
///
/// Returns a map of node → centrality score. Higher scores indicate
/// nodes that appear on many shortest paths (important intersections).
///
/// `sample_size`: number of random source nodes to use (0 = all nodes).
pub fn betweenness_centrality(
    graph: &Graph,
    profile: &SpeedProfile,
    sample_size: usize,
) -> HashMap<NodeId, f64> {
    let n = graph.num_nodes();
    let mut centrality: HashMap<NodeId, f64> = HashMap::new();

    let sources: Vec<usize> = if sample_size == 0 || sample_size >= n {
        (0..n).collect()
    } else {
        // Evenly spaced sample for reproducibility
        (0..sample_size).map(|i| i * n / sample_size).collect()
    };

    for &src_idx in &sources {
        let source = NodeId(src_idx as u32);

        // BFS/Dijkstra-based shortest path DAG
        let (dist, predecessors) = single_source_shortest_paths(graph, source, profile);

        // Accumulate betweenness via dependency propagation (Brandes algorithm)
        let mut dependency: Vec<f64> = vec![0.0; n];
        let mut sigma: Vec<f64> = vec![0.0; n];
        sigma[src_idx] = 1.0;

        // Count shortest paths
        let mut order: Vec<usize> = (0..n).filter(|&i| dist[i] < f64::INFINITY).collect();
        order.sort_by(|&a, &b| {
            dist[a]
                .partial_cmp(&dist[b])
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for &v in &order {
            for &pred in &predecessors[v] {
                sigma[v] += sigma[pred];
            }
        }

        // Back-propagation
        for &w in order.iter().rev() {
            if w == src_idx {
                continue;
            }
            for &v in &predecessors[w] {
                if sigma[w] > 0.0 {
                    dependency[v] += (sigma[v] / sigma[w]) * (1.0 + dependency[w]);
                }
            }
            *centrality.entry(NodeId(w as u32)).or_insert(0.0) += dependency[w];
        }
    }

    // Normalize
    let norm = if sources.len() < n {
        (n as f64) / (sources.len() as f64)
    } else {
        1.0
    };
    for val in centrality.values_mut() {
        *val *= norm;
    }

    centrality
}

/// Single-source shortest paths (Dijkstra) returning distances and predecessor lists.
fn single_source_shortest_paths(
    graph: &Graph,
    source: NodeId,
    profile: &SpeedProfile,
) -> (Vec<f64>, Vec<Vec<usize>>) {
    use std::cmp::Ordering;
    use std::collections::BinaryHeap;

    let n = graph.num_nodes();
    let mut dist = vec![f64::INFINITY; n];
    let mut predecessors: Vec<Vec<usize>> = vec![Vec::new(); n];

    #[derive(Debug, Clone)]
    struct State {
        cost: f64,
        node: usize,
    }

    impl PartialEq for State {
        fn eq(&self, other: &Self) -> bool {
            self.cost == other.cost
        }
    }
    impl Eq for State {}
    impl PartialOrd for State {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }
    impl Ord for State {
        fn cmp(&self, other: &Self) -> Ordering {
            other
                .cost
                .partial_cmp(&self.cost)
                .unwrap_or(Ordering::Equal)
        }
    }

    let src = source.0 as usize;
    dist[src] = 0.0;

    let mut heap = BinaryHeap::new();
    heap.push(State {
        cost: 0.0,
        node: src,
    });

    while let Some(State { cost, node }) = heap.pop() {
        if cost > dist[node] {
            continue;
        }

        let node_id = NodeId(node as u32);
        for edge in graph.outgoing_edges(node_id) {
            let weight = graph.edge_weight(edge, profile);
            let next = edge.to.0 as usize;
            let new_cost = cost + weight;

            if new_cost < dist[next] - 1e-10 {
                dist[next] = new_cost;
                predecessors[next] = vec![node];
                heap.push(State {
                    cost: new_cost,
                    node: next,
                });
            } else if (new_cost - dist[next]).abs() < 1e-10 {
                predecessors[next].push(node);
            }
        }
    }

    (dist, predecessors)
}

/// Validate that a node exists in the graph.
pub fn validate_node(graph: &Graph, node: NodeId) -> Result<(), RoutingError> {
    if (node.0 as usize) >= graph.num_nodes() {
        return Err(RoutingError::NodeNotFound(node.0));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use itinera_graph::{Coord, Edge, Graph, Node, NodeId, SpeedProfile};

    fn test_graph() -> Graph {
        // Simple diamond graph:
        //   0 -> 1 -> 3
        //   0 -> 2 -> 3
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: Coord { lat: 0.0, lon: 0.0 },
                osm_id: 100,
                ch_level: 0,
            },
            Node {
                id: NodeId(1),
                coord: Coord { lat: 1.0, lon: 0.0 },
                osm_id: 101,
                ch_level: 0,
            },
            Node {
                id: NodeId(2),
                coord: Coord { lat: 0.0, lon: 1.0 },
                osm_id: 102,
                ch_level: 0,
            },
            Node {
                id: NodeId(3),
                coord: Coord { lat: 1.0, lon: 1.0 },
                osm_id: 103,
                ch_level: 0,
            },
        ];
        let edges = vec![
            Edge {
                from: NodeId(0),
                to: NodeId(1),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 1,
                road_class: 5,
                oneway: false,
                name: Some("Road A".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(1),
                to: NodeId(0),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 1,
                road_class: 5,
                oneway: false,
                name: Some("Road A".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(0),
                to: NodeId(2),
                distance_m: 150.0,
                duration_s: 15.0,
                way_id: 2,
                road_class: 5,
                oneway: false,
                name: Some("Road B".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(2),
                to: NodeId(0),
                distance_m: 150.0,
                duration_s: 15.0,
                way_id: 2,
                road_class: 5,
                oneway: false,
                name: Some("Road B".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(1),
                to: NodeId(3),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 3,
                road_class: 5,
                oneway: false,
                name: Some("Road C".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(3),
                to: NodeId(1),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 3,
                road_class: 5,
                oneway: false,
                name: Some("Road C".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(2),
                to: NodeId(3),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 4,
                road_class: 5,
                oneway: false,
                name: Some("Road D".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(3),
                to: NodeId(2),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 4,
                road_class: 5,
                oneway: false,
                name: Some("Road D".into()),
                geometry: vec![],
            },
        ];
        Graph::build(nodes, edges)
    }

    fn disconnected_graph() -> Graph {
        // Two separate components: {0,1} and {2,3}
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: Coord { lat: 0.0, lon: 0.0 },
                osm_id: 100,
                ch_level: 0,
            },
            Node {
                id: NodeId(1),
                coord: Coord { lat: 1.0, lon: 0.0 },
                osm_id: 101,
                ch_level: 0,
            },
            Node {
                id: NodeId(2),
                coord: Coord {
                    lat: 10.0,
                    lon: 10.0,
                },
                osm_id: 102,
                ch_level: 0,
            },
            Node {
                id: NodeId(3),
                coord: Coord {
                    lat: 11.0,
                    lon: 10.0,
                },
                osm_id: 103,
                ch_level: 0,
            },
        ];
        let edges = vec![
            Edge {
                from: NodeId(0),
                to: NodeId(1),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 1,
                road_class: 5,
                oneway: false,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(1),
                to: NodeId(0),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 1,
                road_class: 5,
                oneway: false,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(2),
                to: NodeId(3),
                distance_m: 50.0,
                duration_s: 5.0,
                way_id: 2,
                road_class: 5,
                oneway: false,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(3),
                to: NodeId(2),
                distance_m: 50.0,
                duration_s: 5.0,
                way_id: 2,
                road_class: 5,
                oneway: false,
                name: None,
                geometry: vec![],
            },
        ];
        Graph::build(nodes, edges)
    }

    fn profile() -> SpeedProfile {
        SpeedProfile::car()
    }

    #[test]
    fn test_connected_components_single() {
        let graph = test_graph();
        let result = connected_components(&graph);
        assert_eq!(result.num_components, 1);
        assert_eq!(result.component_sizes[&0], 4);
    }

    #[test]
    fn test_connected_components_disconnected() {
        let graph = disconnected_graph();
        let result = connected_components(&graph);
        assert_eq!(result.num_components, 2);
        // Each component has 2 nodes
        let sizes: Vec<usize> = result.component_sizes.values().copied().collect();
        assert!(sizes.contains(&2));
    }

    #[test]
    fn test_od_matrix() {
        let graph = test_graph();
        let p = profile();
        let origins = vec![NodeId(0)];
        let destinations = vec![NodeId(1), NodeId(3)];

        let matrix = od_matrix(&graph, &origins, &destinations, &p);
        assert!(!matrix.is_empty());

        // Should find route from 0 to 1 (direct) and 0 to 3 (via 1 or 2)
        let to_1 = matrix.iter().find(|e| e.destination == NodeId(1));
        assert!(to_1.is_some());
        assert!(to_1.unwrap().cost > 0.0);

        let to_3 = matrix.iter().find(|e| e.destination == NodeId(3));
        assert!(to_3.is_some());
    }

    #[test]
    fn test_closest_facility() {
        let graph = test_graph();
        let p = profile();
        let demand = vec![NodeId(0)];
        let facilities = vec![NodeId(1), NodeId(3)];

        let results = closest_facility(&graph, &demand, &facilities, &p);
        assert_eq!(results.len(), 1);
        // Node 1 is closer to node 0 than node 3
        assert_eq!(results[0].facility_node, NodeId(1));
    }

    #[test]
    fn test_betweenness_centrality() {
        let graph = test_graph();
        let p = profile();
        let centrality = betweenness_centrality(&graph, &p, 0);

        // In a diamond graph, nodes 1 and 2 should have some centrality
        // as shortest paths from 0→3 go through them
        assert!(!centrality.is_empty());
    }

    #[test]
    fn test_validate_node_valid() {
        let graph = test_graph();
        assert!(validate_node(&graph, NodeId(0)).is_ok());
        assert!(validate_node(&graph, NodeId(3)).is_ok());
    }

    #[test]
    fn test_validate_node_invalid() {
        let graph = test_graph();
        assert!(validate_node(&graph, NodeId(99)).is_err());
    }
}
