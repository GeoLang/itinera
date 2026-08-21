use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use itinera_graph::{Coord, Graph, NodeId, SpeedProfile};

use crate::error::RoutingError;
use crate::route::{Route, route_from_path};

/// State with f-score for A*. `incoming_way` 0 means no previous way.
#[derive(Debug, Clone)]
struct AStarState {
    f_score: f64,
    g_score: f64,
    node: NodeId,
    incoming_way: i64,
}

impl PartialEq for AStarState {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for AStarState {}

impl PartialOrd for AStarState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AStarState {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

/// Heuristic: estimated travel time from coord to target using haversine distance
/// at a generous max speed (150 km/h) to ensure admissibility.
fn heuristic(from: Coord, to: Coord) -> f64 {
    let dist = from.distance_to(to);
    // time = distance / speed; 150 km/h = 41.67 m/s
    dist / 41.67
}

/// A* shortest path algorithm with haversine heuristic.
///
/// Typically 2-5x faster than Dijkstra for long-distance routes
/// due to directed search toward the target.
pub fn astar(
    graph: &Graph,
    source: NodeId,
    target: NodeId,
    profile: &SpeedProfile,
) -> Result<Route, RoutingError> {
    let n = graph.num_nodes();
    if n == 0 {
        return Err(RoutingError::EmptyGraph);
    }

    let src_idx = source.0 as usize;
    let tgt_idx = target.0 as usize;

    if src_idx >= n {
        return Err(RoutingError::NodeNotFound(source.0));
    }
    if tgt_idx >= n {
        return Err(RoutingError::NodeNotFound(target.0));
    }

    let target_coord = graph
        .node_coord(target)
        .ok_or(RoutingError::NodeNotFound(target.0))?;

    let mut g_scores: HashMap<(u32, i64), f64> = HashMap::new();
    let mut prev: HashMap<(u32, i64), (u32, i64)> = HashMap::new();
    let mut settled: HashSet<(u32, i64)> = HashSet::new();

    g_scores.insert((source.0, 0), 0.0);

    let source_coord = graph
        .node_coord(source)
        .ok_or(RoutingError::NodeNotFound(source.0))?;
    let initial_h = heuristic(source_coord, target_coord);

    let mut heap = BinaryHeap::new();
    heap.push(AStarState {
        f_score: initial_h,
        g_score: 0.0,
        node: source,
        incoming_way: 0,
    });

    let mut arrival: Option<(i64, f64)> = None;

    while let Some(AStarState {
        g_score,
        node,
        incoming_way,
        ..
    }) = heap.pop()
    {
        let key = (node.0, incoming_way);
        if !settled.insert(key) {
            continue;
        }

        if g_score > g_scores.get(&key).copied().unwrap_or(f64::INFINITY) {
            continue;
        }

        if node == target {
            arrival = Some((incoming_way, g_score));
            break;
        }

        for edge in graph.outgoing_edges(node) {
            if graph.turn_is_banned(node, incoming_way, edge.way_id) {
                continue;
            }
            let weight = graph.edge_weight(edge, profile);
            if weight == f64::INFINITY {
                continue;
            }

            let next_key = (edge.to.0, edge.way_id);
            let new_g = g_score + weight;

            if new_g < g_scores.get(&next_key).copied().unwrap_or(f64::INFINITY) {
                g_scores.insert(next_key, new_g);
                prev.insert(next_key, key);

                let next_coord = graph.node_coord(edge.to).unwrap_or(target_coord);
                let h = heuristic(next_coord, target_coord);

                heap.push(AStarState {
                    f_score: new_g + h,
                    g_score: new_g,
                    node: edge.to,
                    incoming_way: edge.way_id,
                });
            }
        }
    }

    let Some((arrival_way, duration_s)) = arrival else {
        return Err(RoutingError::NoRoute {
            from: format!("{source:?}"),
            to: format!("{target:?}"),
        });
    };

    let mut path = Vec::new();
    let mut current = (target.0, arrival_way);
    loop {
        path.push(current.0);
        if current.0 == source.0 {
            break;
        }
        current = prev.get(&current).copied().ok_or(RoutingError::NoRoute {
            from: format!("{source:?}"),
            to: format!("{target:?}"),
        })?;
    }
    path.reverse();

    Ok(route_from_path(graph, &path, profile, duration_s))
}

#[cfg(test)]
mod tests {
    use super::*;
    use itinera_graph::{Edge, Node};

    fn test_graph() -> Graph {
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: Coord::new(0.0, 0.0),
                osm_id: 1,
                ch_level: 0,
            },
            Node {
                id: NodeId(1),
                coord: Coord::new(0.0, 1.0),
                osm_id: 2,
                ch_level: 0,
            },
            Node {
                id: NodeId(2),
                coord: Coord::new(1.0, 0.0),
                osm_id: 3,
                ch_level: 0,
            },
            Node {
                id: NodeId(3),
                coord: Coord::new(1.0, 1.0),
                osm_id: 4,
                ch_level: 0,
            },
        ];

        let edges = vec![
            Edge {
                from: NodeId(0),
                to: NodeId(1),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 1,
                road_class: 5,
                oneway: true,
                name: Some("A".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(1),
                to: NodeId(3),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 2,
                road_class: 5,
                oneway: true,
                name: Some("B".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(0),
                to: NodeId(2),
                distance_m: 1250.0,
                duration_s: 25.0,
                way_id: 3,
                road_class: 5,
                oneway: true,
                name: Some("C".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(2),
                to: NodeId(3),
                distance_m: 250.0,
                duration_s: 5.0,
                way_id: 4,
                road_class: 5,
                oneway: true,
                name: Some("D".into()),
                geometry: vec![],
            },
        ];

        Graph::build(nodes, edges)
    }

    #[test]
    fn test_astar_finds_same_path_as_dijkstra() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let route = astar(&g, NodeId(0), NodeId(3), &profile).unwrap();
        assert_eq!(route.node_ids, vec![0, 1, 3]);
    }

    #[test]
    fn test_astar_no_route() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let result = astar(&g, NodeId(3), NodeId(0), &profile);
        assert!(result.is_err());
    }

    fn banned_left_graph() -> Graph {
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: Coord::new(0.0, 0.0),
                osm_id: 1,
                ch_level: 0,
            },
            Node {
                id: NodeId(1),
                coord: Coord::new(0.0, 1.0),
                osm_id: 2,
                ch_level: 0,
            },
            Node {
                id: NodeId(2),
                coord: Coord::new(0.0, 2.0),
                osm_id: 3,
                ch_level: 0,
            },
            Node {
                id: NodeId(3),
                coord: Coord::new(1.0, 1.0),
                osm_id: 4,
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
                oneway: true,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(1),
                to: NodeId(2),
                distance_m: 100.0,
                duration_s: 10.0,
                way_id: 2,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(0),
                to: NodeId(3),
                distance_m: 800.0,
                duration_s: 80.0,
                way_id: 3,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(3),
                to: NodeId(2),
                distance_m: 800.0,
                duration_s: 80.0,
                way_id: 4,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            },
        ];
        let mut g = Graph::build(nodes, edges);
        g.restrictions.push(itinera_graph::TurnRestriction {
            via_node: NodeId(1),
            from_way: 1,
            to_way: 2,
            restriction_type: itinera_graph::turn::RestrictionType::No,
        });
        g
    }

    #[test]
    fn test_astar_avoids_banned_left_turn() {
        let g = banned_left_graph();
        let profile = SpeedProfile::car();
        let route = astar(&g, NodeId(0), NodeId(2), &profile).unwrap();
        assert_eq!(route.node_ids, vec![0, 3, 2]);
        assert!(!route.node_ids.windows(2).any(|hop| hop == [1, 2]));
        assert!((route.distance_m - 1600.0).abs() < 1e-6);
    }
}
