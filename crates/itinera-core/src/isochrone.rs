use std::cmp::Ordering;
use std::collections::BinaryHeap;

use geo::ConcaveHull;
use geo::concave_hull::ConcaveHullOptions;
use itinera_graph::{Coord, Graph, NodeId, SpeedProfile};

/// Default hull concavity. Lower values hug the network more closely, infinity gives a convex hull.
pub const DEFAULT_CONCAVITY: f64 = 2.0;

/// Isochrone result — set of reachable nodes within a time budget.
#[derive(Debug, Clone)]
pub struct IsochroneResult {
    /// Nodes reachable within the time budget, with their travel time.
    pub nodes: Vec<(NodeId, f64)>,
    /// Concave hull boundary of the isochrone, as an open ring of coords.
    pub boundary: Vec<Coord>,
}

#[derive(Debug, Clone)]
struct IsoState {
    cost: f64,
    node: NodeId,
}

impl PartialEq for IsoState {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for IsoState {}

impl PartialOrd for IsoState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for IsoState {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

/// Compute an isochrone: all nodes reachable from `source` within `max_seconds`.
///
/// Returns reachable nodes with their travel times, plus a concave hull boundary.
/// See [`DEFAULT_CONCAVITY`] for the `concavity` argument.
pub fn isochrone(
    graph: &Graph,
    source: NodeId,
    max_seconds: f64,
    profile: &SpeedProfile,
    concavity: f64,
) -> IsochroneResult {
    let n = graph.num_nodes();
    let mut dist = vec![f64::INFINITY; n];
    let mut visited = vec![false; n];
    let mut reachable = Vec::new();

    let src_idx = source.0 as usize;
    if src_idx >= n {
        return IsochroneResult {
            nodes: Vec::new(),
            boundary: Vec::new(),
        };
    }

    dist[src_idx] = 0.0;
    let mut heap = BinaryHeap::new();
    heap.push(IsoState {
        cost: 0.0,
        node: source,
    });

    while let Some(IsoState { cost, node }) = heap.pop() {
        let node_idx = node.0 as usize;

        if visited[node_idx] {
            continue;
        }
        visited[node_idx] = true;

        if cost > max_seconds {
            break;
        }

        reachable.push((node, cost));

        for edge in graph.outgoing_edges(node) {
            let weight = graph.edge_weight(edge, profile);
            if weight == f64::INFINITY {
                continue;
            }

            let next = edge.to;
            let next_idx = next.0 as usize;
            let new_cost = cost + weight;

            if new_cost <= max_seconds && new_cost < dist[next_idx] {
                dist[next_idx] = new_cost;
                heap.push(IsoState {
                    cost: new_cost,
                    node: next,
                });
            }
        }
    }

    let coords: Vec<Coord> = reachable
        .iter()
        .filter_map(|(nid, _)| graph.node_coord(*nid))
        .collect();

    let boundary = concave_hull(&coords, concavity);

    IsochroneResult {
        nodes: reachable,
        boundary,
    }
}

/// Boundary ring around `points`, left open so the first coord is not repeated at the end.
fn concave_hull(points: &[Coord], concavity: f64) -> Vec<Coord> {
    let hull_input: Vec<geo::Coord<f64>> = points
        .iter()
        .map(|c| geo::Coord { x: c.lon, y: c.lat })
        .collect();

    let hull = hull_input.concave_hull_with_options(ConcaveHullOptions {
        concavity,
        length_threshold: 0.0,
    });

    let mut ring: Vec<Coord> = hull
        .exterior()
        .0
        .iter()
        .map(|c| Coord::new(c.y, c.x))
        .collect();

    if ring.len() > 1 && ring.first() == ring.last() {
        ring.pop();
    }

    ring
}

#[cfg(test)]
mod tests {
    use super::*;
    use itinera_graph::{Edge, Node};

    fn grid_graph() -> Graph {
        // 3x3 grid:
        // 0-1-2
        // |   |
        // 3-4-5
        // |   |
        // 6-7-8
        let mut nodes = Vec::new();
        for i in 0..9 {
            let row = i / 3;
            let col = i % 3;
            nodes.push(Node {
                id: NodeId(i as u32),
                coord: Coord::new(row as f64 * 0.01, col as f64 * 0.01),
                osm_id: i as i64,
                ch_level: 0,
            });
        }

        let connections = [
            (0, 1),
            (1, 2),
            (0, 3),
            (2, 5),
            (3, 4),
            (4, 5),
            (3, 6),
            (5, 8),
            (6, 7),
            (7, 8),
            // Reverse directions
            (1, 0),
            (2, 1),
            (3, 0),
            (5, 2),
            (4, 3),
            (5, 4),
            (6, 3),
            (8, 5),
            (7, 6),
            (8, 7),
        ];

        let edges: Vec<Edge> = connections
            .iter()
            .enumerate()
            .map(|(i, &(from, to))| Edge {
                from: NodeId(from),
                to: NodeId(to),
                distance_m: 1000.0,
                duration_s: 60.0,
                way_id: i as i64,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            })
            .collect();

        Graph::build(nodes, edges)
    }

    const CELL_DEGREES: f64 = 0.01;

    /// Lattice with an edge in both directions between orthogonally adjacent cells.
    fn lattice_graph(cells: &[(u32, u32)]) -> Graph {
        let nodes: Vec<Node> = cells
            .iter()
            .enumerate()
            .map(|(i, &(x, y))| Node {
                id: NodeId(i as u32),
                coord: Coord::new(f64::from(y) * CELL_DEGREES, f64::from(x) * CELL_DEGREES),
                osm_id: i as i64,
                ch_level: 0,
            })
            .collect();

        let mut edges = Vec::new();
        for (i, &(x, y)) in cells.iter().enumerate() {
            for neighbor in [(x + 1, y), (x, y + 1)] {
                let Some(j) = cells.iter().position(|&c| c == neighbor) else {
                    continue;
                };
                for (from, to) in [(i, j), (j, i)] {
                    let way_id = edges.len() as i64;
                    edges.push(Edge {
                        from: NodeId(from as u32),
                        to: NodeId(to as u32),
                        distance_m: 1000.0,
                        duration_s: 60.0,
                        way_id,
                        road_class: 5,
                        oneway: true,
                        name: None,
                        geometry: vec![],
                    });
                }
            }
        }

        Graph::build(nodes, edges)
    }

    const U_COLUMNS: u32 = 9;
    const U_ROWS: u32 = 7;
    const U_NODE_COUNT: usize = 54;
    /// Middle of the U's mouth, strictly inside the convex hull but off the network.
    const MOUTH_LAT: f64 = 5.5 * CELL_DEGREES;
    const MOUTH_LON: f64 = 4.0 * CELL_DEGREES;

    /// 9x7 lattice with the top middle cut out, so the reachable set is a U with an open mouth.
    fn u_shaped_cells() -> Vec<(u32, u32)> {
        let mouth_columns = 3..=5;
        let mouth_rows = 4..=6;
        (0..U_COLUMNS)
            .flat_map(|x| (0..U_ROWS).map(move |y| (x, y)))
            .filter(|(x, y)| !(mouth_columns.contains(x) && mouth_rows.contains(y)))
            .collect()
    }

    fn u_shaped_isochrone(concavity: f64) -> IsochroneResult {
        let graph = lattice_graph(&u_shaped_cells());
        isochrone(&graph, NodeId(0), 10000.0, &SpeedProfile::car(), concavity)
    }

    fn ring_polygon(ring: &[Coord]) -> geo::Polygon<f64> {
        let exterior: Vec<geo::Coord<f64>> = ring
            .iter()
            .map(|c| geo::Coord { x: c.lon, y: c.lat })
            .collect();
        geo::Polygon::new(geo::LineString::new(exterior), vec![])
    }

    fn ring_contains(ring: &[Coord], lat: f64, lon: f64) -> bool {
        use geo::Contains;
        ring_polygon(ring).contains(&geo::Point::new(lon, lat))
    }

    fn ring_area(ring: &[Coord]) -> f64 {
        use geo::Area;
        ring_polygon(ring).unsigned_area()
    }

    #[test]
    fn test_isochrone_limited_reach() {
        let g = grid_graph();
        let profile = SpeedProfile::car();
        // With 1000m edges at 50km/h class 5 -> each edge is 1000*3.6/50 = 72s
        // With budget of 80s, should reach immediate neighbors only
        let result = isochrone(&g, NodeId(4), 80.0, &profile, DEFAULT_CONCAVITY);
        // Node 4 at cost 0, neighbors 3 and 5 at ~72s each
        assert!(result.nodes.len() >= 2);
        assert!(result.nodes.iter().any(|(n, _)| *n == NodeId(4)));
    }

    #[test]
    fn test_isochrone_full_reach() {
        let g = grid_graph();
        let profile = SpeedProfile::car();
        // Large budget should reach all nodes
        let result = isochrone(&g, NodeId(0), 10000.0, &profile, DEFAULT_CONCAVITY);
        assert_eq!(result.nodes.len(), 9);
    }

    #[test]
    fn test_boundary_excludes_the_mouth_of_a_u_shaped_network() {
        let result = u_shaped_isochrone(DEFAULT_CONCAVITY);

        assert_eq!(result.nodes.len(), U_NODE_COUNT);
        assert!(
            !ring_contains(&result.boundary, MOUTH_LAT, MOUTH_LON),
            "boundary {:?} still covers the unreachable mouth",
            result.boundary
        );
    }

    #[test]
    fn test_infinite_concavity_gives_the_convex_hull() {
        let result = u_shaped_isochrone(f64::INFINITY);

        assert!(ring_contains(&result.boundary, MOUTH_LAT, MOUTH_LON));

        let corners = f64::from(U_COLUMNS - 1) * f64::from(U_ROWS - 1) * CELL_DEGREES.powi(2);
        assert!((ring_area(&result.boundary) - corners).abs() < 1e-12);
    }

    #[test]
    fn test_concavity_changes_the_boundary() {
        let concave = u_shaped_isochrone(DEFAULT_CONCAVITY);
        let convex = u_shaped_isochrone(f64::INFINITY);

        assert!(ring_area(&concave.boundary) < ring_area(&convex.boundary));
    }

    #[test]
    fn test_boundary_with_fewer_than_four_reachable_nodes() {
        let graph = lattice_graph(&u_shaped_cells());
        let profile = SpeedProfile::car();
        // 1000m at 50km/h is 72s per edge, so this budget reaches the source and its two neighbors
        let result = isochrone(&graph, NodeId(0), 100.0, &profile, DEFAULT_CONCAVITY);

        assert_eq!(result.nodes.len(), 3);
        assert_eq!(result.boundary.len(), 3);
    }

    #[test]
    fn test_boundary_with_a_single_reachable_node() {
        let graph = lattice_graph(&u_shaped_cells());
        let result = isochrone(
            &graph,
            NodeId(0),
            0.0,
            &SpeedProfile::car(),
            DEFAULT_CONCAVITY,
        );

        assert_eq!(result.nodes.len(), 1);
        assert_eq!(result.boundary.len(), 1);
    }
}
