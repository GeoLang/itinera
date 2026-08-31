//! Contraction hierarchy queries on graphs whose contraction adds shortcuts.
//!
//! A grid of identical streets gives every turn an equal-cost detour, so contraction adds no
//! shortcut and a query never has to traverse one. These grids vary class and length so it does.

use itinera_core::{ContractionHierarchy, dijkstra};
use itinera_graph::{Coord, Edge, Graph, Node, NodeId, SpeedProfile};

const CELL_DEGREES: f64 = 0.001;
const ARTERIAL_EVERY: u32 = 4;
const PRIMARY_CLASS: u8 = 3;
const TERTIARY_CLASS: u8 = 5;

/// Deterministic length factor in 0.75 to 1.25, standing in for streets of unequal length.
fn length_factor(from: u32, to: u32) -> f64 {
    let mixed = ((u64::from(from) << 32) | u64::from(to)).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    0.75 + f64::from((mixed >> 40) as u32) / f64::from(u32::MAX >> 8) * 0.5
}

/// Square grid of two-way streets, one way id per line of the grid.
fn varied_grid(side: u32) -> Graph {
    let mut nodes = Vec::with_capacity((side * side) as usize);
    for row in 0..side {
        for col in 0..side {
            let id = row * side + col;
            nodes.push(Node {
                id: NodeId(id),
                coord: Coord::new(f64::from(row) * CELL_DEGREES, f64::from(col) * CELL_DEGREES),
                osm_id: i64::from(id) + 1,
                ch_level: 0,
            });
        }
    }

    let mut edges = Vec::new();
    for row in 0..side {
        for col in 0..side {
            let id = row * side + col;
            for (neighbour_row, neighbour_col) in [(row + 1, col), (row, col + 1)] {
                if neighbour_row >= side || neighbour_col >= side {
                    continue;
                }
                let neighbour = neighbour_row * side + neighbour_col;
                let horizontal = neighbour_row == row;
                let line = if horizontal { row } else { col };
                let distance_m = nodes[id as usize]
                    .coord
                    .distance_to(nodes[neighbour as usize].coord)
                    * length_factor(id, neighbour);
                let road_class = if line.is_multiple_of(ARTERIAL_EVERY) {
                    PRIMARY_CLASS
                } else {
                    TERTIARY_CLASS
                };
                let way_id = i64::from(if horizontal { line } else { side + line }) + 1;
                for (from, to) in [(id, neighbour), (neighbour, id)] {
                    edges.push(Edge {
                        from: NodeId(from),
                        to: NodeId(to),
                        distance_m,
                        duration_s: 0.0,
                        way_id,
                        road_class,
                        oneway: false,
                        name: None,
                        geometry: Vec::new(),
                    });
                }
            }
        }
    }

    Graph::build(nodes, edges)
}

fn shortcut_count(graph: &Graph, hierarchy: &ContractionHierarchy) -> usize {
    hierarchy.graph.num_edges() - graph.num_edges()
}

/// Walk the unpacked path over the original graph, so a path that skips the nodes inside a
/// shortcut fails here even when the reported cost is right.
fn walked_cost(graph: &Graph, path: &[NodeId], profile: &SpeedProfile) -> f64 {
    let mut total = 0.0;
    for hop in path.windows(2) {
        let cheapest = graph
            .outgoing_edges(hop[0])
            .iter()
            .filter(|edge| edge.to == hop[1])
            .map(|edge| graph.edge_weight(edge, profile))
            .fold(f64::INFINITY, f64::min);
        assert!(
            cheapest.is_finite(),
            "path hop {:?} -> {:?} is not an edge of the graph",
            hop[0],
            hop[1]
        );
        total += cheapest;
    }
    total
}

fn assert_matches_dijkstra(graph: &Graph, hierarchy: &ContractionHierarchy, from: u32, to: u32) {
    let profile = SpeedProfile::car();
    let expected = dijkstra(graph, NodeId(from), NodeId(to), &profile)
        .unwrap_or_else(|error| panic!("dijkstra {from} -> {to}: {error}"));
    let (cost, path) = hierarchy
        .query(NodeId(from), NodeId(to), &profile)
        .unwrap_or_else(|| panic!("ch query {from} -> {to} found no path"));

    assert!(
        (cost - expected.duration_s).abs() < 1e-6,
        "ch cost {cost} but dijkstra {} for {from} -> {to}",
        expected.duration_s
    );
    assert_eq!(*path.first().unwrap(), NodeId(from));
    assert_eq!(*path.last().unwrap(), NodeId(to));
    let walked = walked_cost(graph, &path, &profile);
    assert!(
        (walked - cost).abs() < 1e-6,
        "unpacked path costs {walked} but the query reported {cost} for {from} -> {to}"
    );
}

#[test]
fn ch_query_matches_dijkstra_on_small_shortcut_graph() {
    let graph = varied_grid(4);
    let hierarchy = ContractionHierarchy::build(&graph, &SpeedProfile::car());
    assert!(shortcut_count(&graph, &hierarchy) > 0);
    assert_matches_dijkstra(&graph, &hierarchy, 0, 15);
}

#[test]
fn ch_query_matches_dijkstra_for_every_pair() {
    let side = 6;
    let graph = varied_grid(side);
    let hierarchy = ContractionHierarchy::build(&graph, &SpeedProfile::car());
    assert!(shortcut_count(&graph, &hierarchy) > 0);

    let node_count = side * side;
    for from in 0..node_count {
        for to in 0..node_count {
            if from != to {
                assert_matches_dijkstra(&graph, &hierarchy, from, to);
            }
        }
    }
}

#[test]
fn ch_query_matches_dijkstra_on_a_larger_shortcut_graph() {
    let side = 12;
    let graph = varied_grid(side);
    let hierarchy = ContractionHierarchy::build(&graph, &SpeedProfile::car());
    assert!(shortcut_count(&graph, &hierarchy) > 0);

    let node_count = side * side;
    for step in 0..node_count {
        let from = (step * 37) % node_count;
        let to = (step * 101 + 13) % node_count;
        if from != to {
            assert_matches_dijkstra(&graph, &hierarchy, from, to);
        }
    }
}

#[test]
fn ch_query_matches_dijkstra_under_the_pedestrian_profile() {
    let graph = varied_grid(6);
    let profile = SpeedProfile::pedestrian();
    let hierarchy = ContractionHierarchy::build(&graph, &profile);
    assert!(shortcut_count(&graph, &hierarchy) > 0);

    let expected = dijkstra(&graph, NodeId(0), NodeId(35), &profile).unwrap();
    let (cost, path) = hierarchy.query(NodeId(0), NodeId(35), &profile).unwrap();
    assert!((cost - expected.duration_s).abs() < 1e-6);
    assert!((walked_cost(&graph, &path, &profile) - cost).abs() < 1e-6);
}
