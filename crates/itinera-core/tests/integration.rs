//! Integration tests for the Itinera routing engine.

use itinera_core::{ContractionHierarchy, DEFAULT_CONCAVITY, astar, dijkstra, isochrone};
use itinera_graph::{Coord, Edge, Graph, Node, NodeId, SpeedProfile};

/// Build a grid graph for testing:
///
/// ```text
/// 0 --- 1 --- 2
/// |     |     |
/// 3 --- 4 --- 5
/// |     |     |
/// 6 --- 7 --- 8
/// ```
///
/// All edges are bidirectional, 1000m apart.
fn grid_3x3() -> Graph {
    let mut nodes = Vec::new();
    for row in 0..3u32 {
        for col in 0..3u32 {
            let id = row * 3 + col;
            nodes.push(Node {
                id: NodeId(id),
                coord: Coord::new(48.0 + row as f64 * 0.01, 2.0 + col as f64 * 0.01),
                osm_id: id as i64 + 100,
                ch_level: 0,
            });
        }
    }

    let mut edges = Vec::new();
    let mut way_id = 1i64;

    let add_bidi = |edges: &mut Vec<Edge>, from: u32, to: u32, way_id: &mut i64| {
        edges.push(Edge {
            from: NodeId(from),
            to: NodeId(to),
            distance_m: 1000.0,
            duration_s: 60.0,
            way_id: *way_id,
            road_class: 5,
            oneway: false,
            name: None,
            geometry: Vec::new(),
        });
        edges.push(Edge {
            from: NodeId(to),
            to: NodeId(from),
            distance_m: 1000.0,
            duration_s: 60.0,
            way_id: *way_id,
            road_class: 5,
            oneway: false,
            name: None,
            geometry: Vec::new(),
        });
        *way_id += 1;
    };

    // Horizontal edges
    for row in 0..3u32 {
        for col in 0..2u32 {
            let from = row * 3 + col;
            let to = from + 1;
            add_bidi(&mut edges, from, to, &mut way_id);
        }
    }
    // Vertical edges
    for row in 0..2u32 {
        for col in 0..3u32 {
            let from = row * 3 + col;
            let to = from + 3;
            add_bidi(&mut edges, from, to, &mut way_id);
        }
    }

    Graph::build(nodes, edges)
}

/// Build a linear chain: 0 -> 1 -> 2 -> 3 -> 4
fn linear_5() -> Graph {
    let nodes: Vec<Node> = (0..5)
        .map(|i| Node {
            id: NodeId(i),
            coord: Coord::new(48.0 + i as f64 * 0.01, 2.0),
            osm_id: i as i64,
            ch_level: 0,
        })
        .collect();

    let edges: Vec<Edge> = (0..4)
        .map(|i| Edge {
            from: NodeId(i),
            to: NodeId(i + 1),
            distance_m: 500.0,
            duration_s: 30.0,
            way_id: i as i64 + 1,
            road_class: 5,
            oneway: true,
            name: Some(format!("Road {}", i + 1)),
            geometry: Vec::new(),
        })
        .collect();

    Graph::build(nodes, edges)
}

#[test]
fn test_dijkstra_grid_shortest_path() {
    let g = grid_3x3();
    let profile = SpeedProfile::car();
    let route = dijkstra(&g, NodeId(0), NodeId(8), &profile).unwrap();
    // Manhattan distance: 4 hops minimum
    assert_eq!(route.node_ids.len(), 5);
    assert_eq!(*route.node_ids.first().unwrap(), 0);
    assert_eq!(*route.node_ids.last().unwrap(), 8);
}

#[test]
fn test_astar_grid_shortest_path() {
    let g = grid_3x3();
    let profile = SpeedProfile::car();
    let route = astar(&g, NodeId(0), NodeId(8), &profile).unwrap();
    assert_eq!(route.node_ids.len(), 5);
    assert_eq!(*route.node_ids.first().unwrap(), 0);
    assert_eq!(*route.node_ids.last().unwrap(), 8);
}

#[test]
fn test_dijkstra_and_astar_agree() {
    let g = grid_3x3();
    let profile = SpeedProfile::car();

    let d_route = dijkstra(&g, NodeId(0), NodeId(8), &profile).unwrap();
    let a_route = astar(&g, NodeId(0), NodeId(8), &profile).unwrap();

    assert!((d_route.duration_s - a_route.duration_s).abs() < 0.01);
    assert!((d_route.distance_m - a_route.distance_m).abs() < 0.01);
}

#[test]
fn test_ch_gives_same_cost_as_dijkstra() {
    let g = grid_3x3();
    let profile = SpeedProfile::car();

    let ch = ContractionHierarchy::build(&g, &profile);
    let d_route = dijkstra(&g, NodeId(0), NodeId(8), &profile).unwrap();
    let ch_result = ch.query(NodeId(0), NodeId(8), &profile);

    let (ch_cost, _ch_path) = ch_result.unwrap();
    assert!((d_route.duration_s - ch_cost).abs() < 0.1);
}

#[test]
fn test_isochrone_reachability() {
    let g = grid_3x3();
    let profile = SpeedProfile::car();
    // 1000m at 50 km/h = 72s per edge. Budget of 150s should reach 2-hop neighbors.
    let result = isochrone(&g, NodeId(4), 150.0, &profile, DEFAULT_CONCAVITY);

    assert!(result.nodes.len() >= 5);
    assert!(result.boundary.len() >= 3);
}

#[test]
fn test_isochrone_zero_budget() {
    let g = grid_3x3();
    let profile = SpeedProfile::car();
    let result = isochrone(&g, NodeId(0), 0.0, &profile, DEFAULT_CONCAVITY);
    assert_eq!(result.nodes.len(), 1);
    assert_eq!(result.nodes[0].0, NodeId(0));
}

#[test]
fn test_linear_route_steps() {
    let g = linear_5();
    let profile = SpeedProfile::car();
    let route = dijkstra(&g, NodeId(0), NodeId(4), &profile).unwrap();

    assert_eq!(route.node_ids, vec![0, 1, 2, 3, 4]);
    assert_eq!(route.steps.len(), 4);
    assert_eq!(route.distance_m, 2000.0);
}

#[test]
fn test_graph_serialization_roundtrip() {
    let g = grid_3x3();
    let bytes = g.to_bytes();
    let g2 = Graph::from_bytes(&bytes).unwrap();

    assert_eq!(g2.num_nodes(), g.num_nodes());
    assert_eq!(g2.num_edges(), g.num_edges());

    for i in 0..g.num_nodes() {
        let node = NodeId(i as u32);
        assert_eq!(g.outgoing_edges(node).len(), g2.outgoing_edges(node).len());
    }
}

#[test]
fn test_ch_serialization_roundtrip() {
    let g = linear_5();
    let profile = SpeedProfile::car();
    let ch = ContractionHierarchy::build(&g, &profile);

    let bytes = bincode::serialize(&ch).unwrap();
    let ch2: ContractionHierarchy = bincode::deserialize(&bytes).unwrap();

    let r1 = ch.query(NodeId(0), NodeId(4), &profile);
    let r2 = ch2.query(NodeId(0), NodeId(4), &profile);

    assert_eq!(r1.is_some(), r2.is_some());
    if let (Some((c1, _)), Some((c2, _))) = (r1, r2) {
        assert!((c1 - c2).abs() < 0.01);
    }
}

#[test]
fn test_nearest_node() {
    let g = grid_3x3();
    let nearest = g.nearest_node(Coord::new(48.0101, 2.0099)).unwrap();
    assert_eq!(nearest, NodeId(4));
}

#[test]
fn test_incoming_edges() {
    let g = grid_3x3();
    let incoming = g.incoming_edges(NodeId(4));
    assert_eq!(incoming.len(), 4);
}

#[test]
fn test_profiles_different_costs() {
    let g = grid_3x3();
    let car = SpeedProfile::car();
    let ped = SpeedProfile::pedestrian();

    let car_route = dijkstra(&g, NodeId(0), NodeId(8), &car).unwrap();
    let ped_route = dijkstra(&g, NodeId(0), NodeId(8), &ped).unwrap();

    assert!((car_route.distance_m - ped_route.distance_m).abs() < 0.01);
    assert!(ped_route.duration_s > car_route.duration_s);
}

#[test]
fn test_bicycle_avoids_motorway() {
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
            coord: Coord::new(1.0, 0.5),
            osm_id: 3,
            ch_level: 0,
        },
    ];
    let edges = vec![
        Edge {
            from: NodeId(0),
            to: NodeId(1),
            distance_m: 1000.0,
            duration_s: 30.0,
            way_id: 1,
            road_class: 1,
            oneway: true,
            name: None,
            geometry: Vec::new(),
        },
        Edge {
            from: NodeId(0),
            to: NodeId(2),
            distance_m: 1200.0,
            duration_s: 60.0,
            way_id: 2,
            road_class: 7,
            oneway: true,
            name: None,
            geometry: Vec::new(),
        },
        Edge {
            from: NodeId(2),
            to: NodeId(1),
            distance_m: 1200.0,
            duration_s: 60.0,
            way_id: 3,
            road_class: 7,
            oneway: true,
            name: None,
            geometry: Vec::new(),
        },
    ];
    let g = Graph::build(nodes, edges);
    let bike = SpeedProfile::bicycle();

    let route = dijkstra(&g, NodeId(0), NodeId(1), &bike).unwrap();
    assert_eq!(route.node_ids, vec![0, 2, 1]);
}

#[test]
fn test_no_route_disconnected() {
    let nodes = vec![
        Node {
            id: NodeId(0),
            coord: Coord::new(0.0, 0.0),
            osm_id: 1,
            ch_level: 0,
        },
        Node {
            id: NodeId(1),
            coord: Coord::new(1.0, 1.0),
            osm_id: 2,
            ch_level: 0,
        },
    ];
    let g = Graph::build(nodes, Vec::new());
    let profile = SpeedProfile::car();

    assert!(dijkstra(&g, NodeId(0), NodeId(1), &profile).is_err());
}

#[test]
fn test_bearing_north() {
    let a = Coord::new(48.0, 2.0);
    let b = Coord::new(49.0, 2.0);
    let bearing = a.bearing_to(b);
    assert!(
        !(1.0..=359.0).contains(&bearing),
        "Expected ~0°, got {bearing}"
    );
}

#[test]
fn test_bearing_east() {
    let a = Coord::new(48.0, 2.0);
    let b = Coord::new(48.0, 3.0);
    let bearing = a.bearing_to(b);
    assert!((bearing - 90.0).abs() < 5.0, "Expected ~90°, got {bearing}");
}

#[test]
fn test_haversine_known_distance() {
    let paris = Coord::new(48.8566, 2.3522);
    let london = Coord::new(51.5074, -0.1278);
    let dist = paris.distance_to(london);
    assert!(dist > 330_000.0 && dist < 360_000.0, "Got {dist}m");
}

#[test]
fn test_speed_profile_from_name() {
    assert!(SpeedProfile::from_name("car").is_some());
    assert!(SpeedProfile::from_name("bike").is_some());
    assert!(SpeedProfile::from_name("foot").is_some());
    assert!(SpeedProfile::from_name("truck").is_some());
    assert!(SpeedProfile::from_name("helicopter").is_none());
}

#[test]
fn test_turn_restriction_check() {
    use itinera_graph::turn::{RestrictionType, TurnRestriction};

    let mut g = grid_3x3();
    g.restrictions.push(TurnRestriction {
        via_node: NodeId(1),
        from_way: 1,
        to_way: 2,
        restriction_type: RestrictionType::No,
    });

    assert!(g.is_turn_restricted(NodeId(1), 1, 2));
    assert!(!g.is_turn_restricted(NodeId(1), 1, 3));
    assert!(!g.is_turn_restricted(NodeId(2), 1, 2));
}

#[test]
fn test_ch_same_node_query() {
    let g = grid_3x3();
    let profile = SpeedProfile::car();
    let ch = ContractionHierarchy::build(&g, &profile);
    let (cost, path) = ch.query(NodeId(4), NodeId(4), &profile).unwrap();
    assert_eq!(cost, 0.0);
    assert_eq!(path, vec![NodeId(4)]);
}

#[test]
fn test_edge_weight_zero_speed() {
    let g = grid_3x3();
    let mut profile = SpeedProfile::car();
    profile.speeds_kmh[5] = 0.0;

    let edge = &g.outgoing_edges(NodeId(0))[0];
    let weight = g.edge_weight(edge, &profile);
    assert_eq!(weight, f64::INFINITY);
}
