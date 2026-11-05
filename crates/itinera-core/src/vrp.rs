//! Multi-stop route optimization (TSP/VRP).
//!
//! Solves the Traveling Salesman Problem for delivery route planning:
//! given a set of stops, find the shortest route visiting all of them.
//!
//! Uses nearest-neighbor heuristic + 2-opt local search improvement.

use itinera_graph::Coord;

/// A stop in a delivery route.
#[derive(Debug, Clone)]
pub struct Stop {
    pub id: String,
    pub lat: f64,
    pub lng: f64,
}

impl Stop {
    fn coord(&self) -> Coord {
        Coord::new(self.lat, self.lng)
    }
}

/// Result of route optimization.
#[derive(Debug, Clone)]
pub struct OptimizedRoute {
    /// Ordered stop indices (into the original stops slice).
    pub order: Vec<usize>,
    /// Total distance in meters.
    pub total_distance: f64,
}

/// Optimize a multi-stop route starting from a depot.
///
/// Returns the optimal visit order minimizing total travel distance.
/// Uses haversine distances for the initial solution, so no road network
/// is required (though road-distance matrices can be substituted).
///
/// * `depot` — starting location (driver's position or warehouse)
/// * `stops` — delivery stops to visit
/// * `return_to_depot` — whether route must end at depot
pub fn optimize_route(depot: &Stop, stops: &[Stop], return_to_depot: bool) -> OptimizedRoute {
    if stops.is_empty() {
        return OptimizedRoute {
            order: Vec::new(),
            total_distance: 0.0,
        };
    }
    if stops.len() == 1 {
        let d = depot.coord().distance_to(stops[0].coord());
        let total = if return_to_depot { d * 2.0 } else { d };
        return OptimizedRoute {
            order: vec![0],
            total_distance: total,
        };
    }

    // Build distance matrix (depot = index 0, stops = 1..n)
    let n = stops.len() + 1;
    let mut dist = vec![vec![0.0f64; n]; n];
    for i in 0..stops.len() {
        dist[0][i + 1] = depot.coord().distance_to(stops[i].coord());
        dist[i + 1][0] = dist[0][i + 1];
        for j in (i + 1)..stops.len() {
            let d = stops[i].coord().distance_to(stops[j].coord());
            dist[i + 1][j + 1] = d;
            dist[j + 1][i + 1] = d;
        }
    }

    // Nearest-neighbor heuristic
    let mut order = nearest_neighbor(&dist, n, return_to_depot);

    // 2-opt improvement
    two_opt(&mut order, &dist, return_to_depot);

    // Calculate total distance
    let total_distance = route_distance(&order, &dist, return_to_depot);

    // Convert from internal indices (1-based) to stop indices (0-based)
    let order: Vec<usize> = order.iter().map(|&i| i - 1).collect();

    OptimizedRoute {
        order,
        total_distance,
    }
}

/// Optimize with a precomputed distance matrix.
///
/// `distances[i][j]` = distance from stop i to stop j.
/// Index 0 = depot, indices 1..n = stops.
pub fn optimize_route_matrix(distances: &[Vec<f64>], return_to_depot: bool) -> OptimizedRoute {
    let n = distances.len();
    if n <= 1 {
        return OptimizedRoute {
            order: Vec::new(),
            total_distance: 0.0,
        };
    }
    if n == 2 {
        let total = if return_to_depot {
            distances[0][1] * 2.0
        } else {
            distances[0][1]
        };
        return OptimizedRoute {
            order: vec![0],
            total_distance: total,
        };
    }

    let mut order = nearest_neighbor(distances, n, return_to_depot);
    two_opt(&mut order, distances, return_to_depot);
    let total_distance = route_distance(&order, distances, return_to_depot);
    let order: Vec<usize> = order.iter().map(|&i| i - 1).collect();

    OptimizedRoute {
        order,
        total_distance,
    }
}

fn nearest_neighbor(dist: &[Vec<f64>], n: usize, _return_to_depot: bool) -> Vec<usize> {
    let mut visited = vec![false; n];
    visited[0] = true; // depot
    let mut tour = Vec::with_capacity(n - 1);
    let mut current = 0;

    for _ in 0..(n - 1) {
        let mut best = usize::MAX;
        let mut best_dist = f64::MAX;
        for j in 1..n {
            if !visited[j] && dist[current][j] < best_dist {
                best = j;
                best_dist = dist[current][j];
            }
        }
        visited[best] = true;
        tour.push(best);
        current = best;
    }

    tour
}

fn two_opt(tour: &mut [usize], dist: &[Vec<f64>], return_to_depot: bool) {
    let n = tour.len();
    if n < 2 {
        return;
    }

    let mut improved = true;
    while improved {
        improved = false;
        for i in 0..n - 1 {
            for j in (i + 1)..n {
                let gain = two_opt_gain(tour, dist, i, j, return_to_depot);
                if gain > 1e-10 {
                    tour[i..=j].reverse();
                    improved = true;
                }
            }
        }
    }
}

fn two_opt_gain(
    tour: &[usize],
    dist: &[Vec<f64>],
    i: usize,
    j: usize,
    return_to_depot: bool,
) -> f64 {
    let n = tour.len();
    let prev_i = if i == 0 { 0 } else { tour[i - 1] }; // depot if first
    let node_i = tour[i];
    let node_j = tour[j];
    let next_j = if j == n - 1 {
        if return_to_depot {
            0
        } else {
            return dist[prev_i][node_i] + dist[node_j][0] - dist[prev_i][node_j] - dist[node_i][0];
        }
    } else {
        tour[j + 1]
    };

    let old_cost = dist[prev_i][node_i] + dist[node_j][next_j];
    let new_cost = dist[prev_i][node_j] + dist[node_i][next_j];
    old_cost - new_cost
}

fn route_distance(tour: &[usize], dist: &[Vec<f64>], return_to_depot: bool) -> f64 {
    if tour.is_empty() {
        return 0.0;
    }
    let mut total = dist[0][tour[0]]; // depot to first
    for i in 0..tour.len() - 1 {
        total += dist[tour[i]][tour[i + 1]];
    }
    if return_to_depot {
        total += dist[*tour.last().unwrap()][0];
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_stop() {
        let depot = Stop {
            id: "depot".into(),
            lat: 0.0,
            lng: 0.0,
        };
        let stops = vec![Stop {
            id: "A".into(),
            lat: 1.0,
            lng: 0.0,
        }];
        let result = optimize_route(&depot, &stops, false);
        assert_eq!(result.order, vec![0]);
        assert!(result.total_distance > 0.0);
    }

    #[test]
    fn test_empty_stops() {
        let depot = Stop {
            id: "depot".into(),
            lat: 0.0,
            lng: 0.0,
        };
        let result = optimize_route(&depot, &[], false);
        assert!(result.order.is_empty());
        assert_eq!(result.total_distance, 0.0);
    }

    #[test]
    fn test_three_stops_optimized() {
        // Triangle: depot at origin, stops at known positions
        let depot = Stop {
            id: "depot".into(),
            lat: 0.0,
            lng: 0.0,
        };
        let stops = vec![
            Stop {
                id: "A".into(),
                lat: 1.0,
                lng: 0.0,
            },
            Stop {
                id: "B".into(),
                lat: 1.0,
                lng: 1.0,
            },
            Stop {
                id: "C".into(),
                lat: 0.0,
                lng: 1.0,
            },
        ];
        let result = optimize_route(&depot, &stops, true);
        assert_eq!(result.order.len(), 3);
        // Should visit all stops
        let mut visited: Vec<usize> = result.order.clone();
        visited.sort();
        assert_eq!(visited, vec![0, 1, 2]);
    }

    #[test]
    fn test_collinear_stops() {
        // Stops in a line — optimal order is sequential
        let depot = Stop {
            id: "depot".into(),
            lat: 0.0,
            lng: 0.0,
        };
        let stops = vec![
            Stop {
                id: "A".into(),
                lat: 1.0,
                lng: 0.0,
            },
            Stop {
                id: "B".into(),
                lat: 2.0,
                lng: 0.0,
            },
            Stop {
                id: "C".into(),
                lat: 3.0,
                lng: 0.0,
            },
        ];
        let result = optimize_route(&depot, &stops, false);
        // Optimal: 0→1→2 (sequential along the line)
        assert_eq!(result.order, vec![0, 1, 2]);
    }

    #[test]
    fn test_return_to_depot_increases_distance() {
        let depot = Stop {
            id: "depot".into(),
            lat: 0.0,
            lng: 0.0,
        };
        let stops = vec![
            Stop {
                id: "A".into(),
                lat: 1.0,
                lng: 0.0,
            },
            Stop {
                id: "B".into(),
                lat: 2.0,
                lng: 0.0,
            },
        ];
        let one_way = optimize_route(&depot, &stops, false);
        let round_trip = optimize_route(&depot, &stops, true);
        assert!(round_trip.total_distance > one_way.total_distance);
    }

    #[test]
    fn test_distance_matrix() {
        // 3 nodes: depot + 2 stops
        let dist = vec![
            vec![0.0, 10.0, 20.0],
            vec![10.0, 0.0, 5.0],
            vec![20.0, 5.0, 0.0],
        ];
        let result = optimize_route_matrix(&dist, false);
        // Optimal: depot→1→2 (cost 15) vs depot→2→1 (cost 25)
        assert_eq!(result.order, vec![0, 1]);
        assert!((result.total_distance - 15.0).abs() < 1e-6);
    }
}
