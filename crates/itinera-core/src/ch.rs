use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use itinera_graph::{Edge, Graph, NodeId, SpeedProfile};

#[derive(Clone)]
struct Neighbour {
    node: NodeId,
    weight: f64,
    first_way: i64,
    last_way: i64,
}

/// Contraction Hierarchies for fast shortest-path queries.
///
/// Preprocessing contracts nodes in order of "importance", adding shortcut edges.
/// Queries then run a bidirectional Dijkstra on the augmented graph, only relaxing
/// edges going "upward" in the hierarchy.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ContractionHierarchy {
    /// The augmented graph with shortcut edges and CH levels set.
    pub graph: Graph,
    /// Node ordering (`node_order[i]` = the i-th node to be contracted).
    pub node_order: Vec<NodeId>,
    /// For each edge in the augmented graph, the middle node if it's a shortcut.
    pub shortcut_middle: Vec<Option<NodeId>>,
    /// First original way on this edge (or shortcut unpacking).
    shortcut_first_way: Vec<i64>,
    /// Last original way on this edge (or shortcut unpacking).
    shortcut_last_way: Vec<i64>,
}

#[derive(Debug, Clone)]
struct CHState {
    cost: f64,
    node: NodeId,
    way: i64,
}

impl PartialEq for CHState {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for CHState {}

impl PartialOrd for CHState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for CHState {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl ContractionHierarchy {
    /// Build contraction hierarchy from a graph.
    ///
    /// Uses a simple node ordering based on edge-difference heuristic:
    /// priority = shortcuts_needed - edges_removed.
    pub fn build(graph: &Graph, profile: &SpeedProfile) -> Self {
        let n = graph.num_nodes();
        let mut nodes = graph.nodes.clone();
        let mut edges = graph.edges.clone();
        let mut contracted = vec![false; n];
        let mut node_order = Vec::with_capacity(n);
        let mut shortcut_middle: Vec<Option<NodeId>> = vec![None; edges.len()];
        let mut first_ways: Vec<i64> = edges.iter().map(|e| e.way_id).collect();
        let mut last_ways: Vec<i64> = first_ways.clone();

        let mut out_adj: Vec<Vec<Neighbour>> = vec![Vec::new(); n];
        let mut in_adj: Vec<Vec<Neighbour>> = vec![Vec::new(); n];

        for edge in edges.iter() {
            let weight = graph.edge_weight(edge, profile);
            if weight < f64::INFINITY {
                out_adj[edge.from.0 as usize].push(Neighbour {
                    node: edge.to,
                    weight,
                    first_way: edge.way_id,
                    last_way: edge.way_id,
                });
                in_adj[edge.to.0 as usize].push(Neighbour {
                    node: edge.from,
                    weight,
                    first_way: edge.way_id,
                    last_way: edge.way_id,
                });
            }
        }

        // Contract nodes in order of priority
        for level in 0..n {
            let mut best_node = None;
            let mut best_priority = i64::MAX;

            for node_idx in 0..n {
                if contracted[node_idx] {
                    continue;
                }
                let shortcuts =
                    count_shortcuts_needed(node_idx, &out_adj, &in_adj, &contracted, graph);
                let in_degree = in_adj[node_idx]
                    .iter()
                    .filter(|nb| !contracted[nb.node.0 as usize])
                    .count() as i64;
                let out_degree = out_adj[node_idx]
                    .iter()
                    .filter(|nb| !contracted[nb.node.0 as usize])
                    .count() as i64;
                let priority = shortcuts - (in_degree + out_degree);

                if priority < best_priority {
                    best_priority = priority;
                    best_node = Some(node_idx);
                }
            }

            let Some(v) = best_node else { break };

            contracted[v] = true;
            nodes[v].ch_level = level as u16;
            node_order.push(NodeId(v as u32));

            let incoming: Vec<_> = in_adj[v]
                .iter()
                .filter(|nb| !contracted[nb.node.0 as usize])
                .cloned()
                .collect();
            let outgoing: Vec<_> = out_adj[v]
                .iter()
                .filter(|nb| !contracted[nb.node.0 as usize])
                .cloned()
                .collect();

            for uv in &incoming {
                for vw in &outgoing {
                    if uv.node == vw.node {
                        continue;
                    }
                    if graph.turn_is_banned(NodeId(v as u32), uv.last_way, vw.first_way) {
                        continue;
                    }
                    let shortcut_cost = uv.weight + vw.weight;

                    if needs_shortcut(
                        uv.node,
                        vw.node,
                        shortcut_cost,
                        v,
                        &out_adj,
                        &contracted,
                        graph,
                    ) {
                        let sc_distance = shortcut_cost * 50.0 / 3.6;
                        edges.push(Edge {
                            from: uv.node,
                            to: vw.node,
                            distance_m: sc_distance,
                            duration_s: shortcut_cost,
                            way_id: -1,
                            road_class: 0,
                            oneway: true,
                            name: None,
                            geometry: Vec::new(),
                        });
                        shortcut_middle.push(Some(NodeId(v as u32)));
                        first_ways.push(uv.first_way);
                        last_ways.push(vw.last_way);
                        out_adj[uv.node.0 as usize].push(Neighbour {
                            node: vw.node,
                            weight: shortcut_cost,
                            first_way: uv.first_way,
                            last_way: vw.last_way,
                        });
                        in_adj[vw.node.0 as usize].push(Neighbour {
                            node: uv.node,
                            weight: shortcut_cost,
                            first_way: uv.first_way,
                            last_way: vw.last_way,
                        });
                    }
                }
            }
        }

        // Keep shortcut metadata aligned after Graph::build sorts edges.
        let mut packed: Vec<(Edge, Option<NodeId>, i64, i64)> = edges
            .into_iter()
            .zip(shortcut_middle)
            .zip(first_ways)
            .zip(last_ways)
            .map(|(((e, m), f), l)| (e, m, f, l))
            .collect();
        packed.sort_by_key(|(e, _, _, _)| e.from);

        let mut sorted_edges = Vec::with_capacity(packed.len());
        let mut sorted_middle = Vec::with_capacity(packed.len());
        let mut sorted_first = Vec::with_capacity(packed.len());
        let mut sorted_last = Vec::with_capacity(packed.len());
        for (e, m, f, l) in packed {
            sorted_edges.push(e);
            sorted_middle.push(m);
            sorted_first.push(f);
            sorted_last.push(l);
        }

        nodes.sort_by_key(|node| node.id);
        let mut augmented = Graph::build(nodes, sorted_edges);
        augmented.restrictions = graph.restrictions.clone();

        Self {
            graph: augmented,
            node_order,
            shortcut_middle: sorted_middle,
            shortcut_first_way: sorted_first,
            shortcut_last_way: sorted_last,
        }
    }

    fn first_way(&self, edge_idx: usize) -> i64 {
        self.shortcut_first_way[edge_idx]
    }

    fn last_way(&self, edge_idx: usize) -> i64 {
        self.shortcut_last_way[edge_idx]
    }

    /// Travel time along an edge of the augmented graph.
    ///
    /// A shortcut has no road class to look a speed up from, so it carries the summed cost of
    /// the two edges it replaces, under the profile the hierarchy was built with.
    fn edge_weight(&self, edge_idx: usize, profile: &SpeedProfile) -> f64 {
        let edge = &self.graph.edges[edge_idx];
        if self.shortcut_middle[edge_idx].is_some() {
            edge.duration_s
        } else {
            self.graph.edge_weight(edge, profile)
        }
    }

    /// Query shortest path using bidirectional CH search.
    pub fn query(
        &self,
        source: NodeId,
        target: NodeId,
        profile: &SpeedProfile,
    ) -> Option<(f64, Vec<NodeId>)> {
        let n = self.graph.num_nodes();
        let src_idx = source.0 as usize;
        let tgt_idx = target.0 as usize;

        if src_idx >= n || tgt_idx >= n {
            return None;
        }

        if source == target {
            return Some((0.0, vec![source]));
        }

        let mut fwd_dist: Vec<HashMap<i64, f64>> = vec![HashMap::new(); n];
        let mut fwd_prev: Vec<HashMap<i64, (u32, i64)>> = vec![HashMap::new(); n];
        let mut fwd_settled: Vec<HashSet<i64>> = vec![HashSet::new(); n];
        fwd_dist[src_idx].insert(0, 0.0);

        let mut bwd_dist: Vec<HashMap<i64, f64>> = vec![HashMap::new(); n];
        let mut bwd_prev: Vec<HashMap<i64, (u32, i64)>> = vec![HashMap::new(); n];
        let mut bwd_settled: Vec<HashSet<i64>> = vec![HashSet::new(); n];
        bwd_dist[tgt_idx].insert(0, 0.0);

        let mut fwd_heap = BinaryHeap::new();
        let mut bwd_heap = BinaryHeap::new();

        fwd_heap.push(CHState {
            cost: 0.0,
            node: source,
            way: 0,
        });
        bwd_heap.push(CHState {
            cost: 0.0,
            node: target,
            way: 0,
        });

        let mut best_cost = f64::INFINITY;
        let mut meeting: Option<(NodeId, i64, i64)> = None;

        loop {
            let fwd_done = fwd_heap.is_empty();
            let bwd_done = bwd_heap.is_empty();

            if fwd_done && bwd_done {
                break;
            }

            if let Some(CHState { cost, node, way }) = fwd_heap.pop() {
                let node_idx = node.0 as usize;

                if cost > best_cost {
                    // prune
                } else if fwd_settled[node_idx].insert(way) {
                    let bwd_arrivals: Vec<(i64, f64)> =
                        bwd_dist[node_idx].iter().map(|(&w, &c)| (w, c)).collect();
                    for (bwd_way, bwd_cost) in bwd_arrivals {
                        if self.graph.turn_is_banned(node, way, bwd_way) {
                            continue;
                        }
                        let total = cost + bwd_cost;
                        if total < best_cost {
                            best_cost = total;
                            meeting = Some((node, way, bwd_way));
                        }
                    }

                    let node_level = self.graph.nodes[node_idx].ch_level;
                    let start = self.graph.offsets[node_idx] as usize;
                    let end = self.graph.offsets[node_idx + 1] as usize;
                    for edge_idx in start..end {
                        let edge = &self.graph.edges[edge_idx];
                        let to_idx = edge.to.0 as usize;
                        if to_idx < n && self.graph.nodes[to_idx].ch_level >= node_level {
                            if self
                                .graph
                                .turn_is_banned(node, way, self.first_way(edge_idx))
                            {
                                continue;
                            }
                            let weight = self.edge_weight(edge_idx, profile);
                            if weight < f64::INFINITY {
                                let new_cost = cost + weight;
                                let next_way = self.last_way(edge_idx);
                                let better = fwd_dist[to_idx]
                                    .get(&next_way)
                                    .copied()
                                    .unwrap_or(f64::INFINITY);
                                if new_cost < better {
                                    fwd_dist[to_idx].insert(next_way, new_cost);
                                    fwd_prev[to_idx].insert(next_way, (node.0, way));
                                    fwd_heap.push(CHState {
                                        cost: new_cost,
                                        node: edge.to,
                                        way: next_way,
                                    });
                                }
                            }
                        }
                    }
                }
            }

            if let Some(CHState { cost, node, way }) = bwd_heap.pop() {
                let node_idx = node.0 as usize;

                if cost > best_cost {
                    // prune
                } else if bwd_settled[node_idx].insert(way) {
                    let fwd_arrivals: Vec<(i64, f64)> =
                        fwd_dist[node_idx].iter().map(|(&w, &c)| (w, c)).collect();
                    for (fwd_way, fwd_cost) in fwd_arrivals {
                        if self.graph.turn_is_banned(node, fwd_way, way) {
                            continue;
                        }
                        let total = fwd_cost + cost;
                        if total < best_cost {
                            best_cost = total;
                            meeting = Some((node, fwd_way, way));
                        }
                    }

                    let node_level = self.graph.nodes[node_idx].ch_level;
                    let rev_start = self.graph.rev_offsets[node_idx] as usize;
                    let rev_end = self.graph.rev_offsets[node_idx + 1] as usize;
                    for &ei in &self.graph.rev_edge_indices[rev_start..rev_end] {
                        let edge = &self.graph.edges[ei as usize];
                        let from_idx = edge.from.0 as usize;
                        if from_idx < n && self.graph.nodes[from_idx].ch_level >= node_level {
                            if self
                                .graph
                                .turn_is_banned(node, self.last_way(ei as usize), way)
                            {
                                continue;
                            }
                            let weight = self.edge_weight(ei as usize, profile);
                            if weight < f64::INFINITY {
                                let new_cost = cost + weight;
                                let next_way = self.first_way(ei as usize);
                                let better = bwd_dist[from_idx]
                                    .get(&next_way)
                                    .copied()
                                    .unwrap_or(f64::INFINITY);
                                if new_cost < better {
                                    bwd_dist[from_idx].insert(next_way, new_cost);
                                    bwd_prev[from_idx].insert(next_way, (node.0, way));
                                    bwd_heap.push(CHState {
                                        cost: new_cost,
                                        node: edge.from,
                                        way: next_way,
                                    });
                                }
                            }
                        }
                    }
                }
            }

            let fwd_min = fwd_heap.peek().map_or(f64::INFINITY, |s| s.cost);
            let bwd_min = bwd_heap.peek().map_or(f64::INFINITY, |s| s.cost);
            if fwd_min >= best_cost && bwd_min >= best_cost {
                break;
            }
        }

        let (meet, meet_fwd_way, meet_bwd_way) = meeting?;

        let mut fwd_path = Vec::new();
        {
            let mut current = meet.0 as usize;
            let mut cur_way = meet_fwd_way;
            while current != src_idx {
                fwd_path.push(NodeId(current as u32));
                let &(prev_node, prev_way) = fwd_prev[current].get(&cur_way)?;
                current = prev_node as usize;
                cur_way = prev_way;
            }
            fwd_path.push(source);
            fwd_path.reverse();
        }

        let mut bwd_path = Vec::new();
        {
            let mut current = meet.0 as usize;
            let mut cur_way = meet_bwd_way;
            while current != tgt_idx {
                let &(next_node, next_way) = bwd_prev[current].get(&cur_way)?;
                current = next_node as usize;
                cur_way = next_way;
                bwd_path.push(NodeId(current as u32));
            }
        }

        let mut packed_path = fwd_path;
        packed_path.extend(bwd_path);

        let full_path = self.unpack_path(&packed_path, profile);

        Some((best_cost, full_path))
    }

    /// Unpack a path that may contain shortcuts into a full node sequence.
    fn unpack_path(&self, path: &[NodeId], profile: &SpeedProfile) -> Vec<NodeId> {
        if path.len() <= 1 {
            return path.to_vec();
        }

        let mut result = Vec::new();
        result.push(path[0]);

        for window in path.windows(2) {
            self.unpack_edge(window[0], window[1], profile, &mut result);
        }

        result
    }

    /// Recursively unpack a single edge (which may be a shortcut) into the result vec.
    fn unpack_edge(
        &self,
        from: NodeId,
        to: NodeId,
        profile: &SpeedProfile,
        result: &mut Vec<NodeId>,
    ) {
        let from_idx = from.0 as usize;
        if from_idx >= self.graph.nodes.len() {
            result.push(to);
            return;
        }

        let start = self.graph.offsets[from_idx] as usize;
        let end = self.graph.offsets[from_idx + 1] as usize;

        let mut best_edge_idx = None;
        let mut best_weight = f64::INFINITY;

        for edge_idx in start..end {
            let edge = &self.graph.edges[edge_idx];
            if edge.to == to {
                let w = self.edge_weight(edge_idx, profile);
                if w < best_weight {
                    best_weight = w;
                    best_edge_idx = Some(edge_idx);
                }
            }
        }

        if let Some(edge_idx) = best_edge_idx {
            if let Some(Some(middle)) = self.shortcut_middle.get(edge_idx) {
                // Shortcut: recursively unpack both halves
                self.unpack_edge(from, *middle, profile, result);
                self.unpack_edge(*middle, to, profile, result);
            } else {
                result.push(to);
            }
        } else {
            result.push(to);
        }
    }
}

fn count_shortcuts_needed(
    v: usize,
    out_adj: &[Vec<Neighbour>],
    in_adj: &[Vec<Neighbour>],
    contracted: &[bool],
    graph: &Graph,
) -> i64 {
    let incoming: Vec<_> = in_adj[v]
        .iter()
        .filter(|nb| !contracted[nb.node.0 as usize])
        .collect();
    let outgoing: Vec<_> = out_adj[v]
        .iter()
        .filter(|nb| !contracted[nb.node.0 as usize])
        .collect();

    let mut count = 0i64;
    for uv in &incoming {
        for vw in &outgoing {
            if uv.node == vw.node {
                continue;
            }
            if graph.turn_is_banned(NodeId(v as u32), uv.last_way, vw.first_way) {
                continue;
            }
            let shortcut_cost = uv.weight + vw.weight;
            if needs_shortcut(
                uv.node,
                vw.node,
                shortcut_cost,
                v,
                out_adj,
                contracted,
                graph,
            ) {
                count += 1;
            }
        }
    }
    count
}

fn needs_shortcut(
    u: NodeId,
    w: NodeId,
    shortcut_cost: f64,
    v: usize,
    out_adj: &[Vec<Neighbour>],
    contracted: &[bool],
    graph: &Graph,
) -> bool {
    let n = out_adj.len();
    let mut dist: HashMap<(u32, i64), f64> = HashMap::new();
    let mut heap = BinaryHeap::new();

    dist.insert((u.0, 0), 0.0);
    heap.push(CHState {
        cost: 0.0,
        node: u,
        way: 0,
    });

    let max_settle = 5 * n.min(100);
    let mut settled = 0;

    while let Some(CHState { cost, node, way }) = heap.pop() {
        if node == w && cost <= shortcut_cost {
            return false;
        }

        settled += 1;
        if settled > max_settle {
            break;
        }

        if cost > shortcut_cost {
            break;
        }

        let node_idx = node.0 as usize;
        if node_idx >= n {
            continue;
        }

        for nb in &out_adj[node_idx] {
            let next_idx = nb.node.0 as usize;
            if next_idx == v || contracted[next_idx] {
                continue;
            }
            if graph.turn_is_banned(node, way, nb.first_way) {
                continue;
            }
            let new_cost = cost + nb.weight;
            let next_key = (nb.node.0, nb.last_way);
            if new_cost < dist.get(&next_key).copied().unwrap_or(f64::INFINITY)
                && new_cost <= shortcut_cost
            {
                dist.insert(next_key, new_cost);
                heap.push(CHState {
                    cost: new_cost,
                    node: nb.node,
                    way: nb.last_way,
                });
            }
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::route_from_path;
    use itinera_graph::{Coord, Node, TurnRestriction};

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
                coord: Coord::new(0.0, 2.0),
                osm_id: 3,
                ch_level: 0,
            },
            Node {
                id: NodeId(3),
                coord: Coord::new(0.0, 3.0),
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
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(1),
                to: NodeId(2),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 2,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(2),
                to: NodeId(3),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 3,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(1),
                to: NodeId(0),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 1,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(2),
                to: NodeId(1),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 2,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            },
            Edge {
                from: NodeId(3),
                to: NodeId(2),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 3,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            },
        ];

        Graph::build(nodes, edges)
    }

    #[test]
    fn test_ch_build() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let ch = ContractionHierarchy::build(&g, &profile);
        assert_eq!(ch.graph.num_nodes(), 4);
        assert!(ch.graph.num_edges() >= 6);
        assert_eq!(ch.node_order.len(), 4);
    }

    #[test]
    fn test_ch_query_finds_path() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let ch = ContractionHierarchy::build(&g, &profile);

        let result = ch.query(NodeId(0), NodeId(3), &profile);
        assert!(result.is_some());
        let (cost, path) = result.unwrap();
        assert!(cost > 0.0);
        assert_eq!(*path.first().unwrap(), NodeId(0));
        assert_eq!(*path.last().unwrap(), NodeId(3));
    }

    #[test]
    fn test_ch_query_same_node() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let ch = ContractionHierarchy::build(&g, &profile);

        let result = ch.query(NodeId(1), NodeId(1), &profile);
        assert_eq!(result, Some((0.0, vec![NodeId(1)])));
    }

    #[test]
    fn test_ch_query_distance_and_steps() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let ch = ContractionHierarchy::build(&g, &profile);
        let (cost, path) = ch.query(NodeId(0), NodeId(3), &profile).unwrap();
        let node_ids: Vec<u32> = path.iter().map(|n| n.0).collect();
        let route = route_from_path(&g, &node_ids, &profile, cost);

        assert!((route.distance_m - 1500.0).abs() < 1e-6);
        assert!(!route.steps.is_empty());
        assert_eq!(route.steps.len(), node_ids.len() - 1);

        let ped = SpeedProfile::pedestrian();
        let ch_ped = ContractionHierarchy::build(&g, &ped);
        let (ped_cost, ped_path) = ch_ped.query(NodeId(0), NodeId(3), &ped).unwrap();
        let ped_ids: Vec<u32> = ped_path.iter().map(|n| n.0).collect();
        let ped_route = route_from_path(&g, &ped_ids, &ped, ped_cost);
        assert!((ped_route.distance_m - 1500.0).abs() < 1e-6);
        let fabricated = ped_cost * 50.0 / 3.6;
        assert!((fabricated - ped_route.distance_m).abs() > 100.0);
        assert!(!ped_route.steps.is_empty());
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
        g.restrictions.push(TurnRestriction {
            via_node: NodeId(1),
            from_way: 1,
            to_way: 2,
            restriction_type: itinera_graph::turn::RestrictionType::No,
        });
        g
    }

    #[test]
    fn test_ch_copies_restrictions_and_avoids_banned_turn() {
        let g = banned_left_graph();
        let profile = SpeedProfile::car();
        let ch = ContractionHierarchy::build(&g, &profile);
        assert_eq!(ch.graph.restrictions.len(), 1);

        let (cost, path) = ch.query(NodeId(0), NodeId(2), &profile).unwrap();
        let node_ids: Vec<u32> = path.iter().map(|n| n.0).collect();
        assert_eq!(node_ids, vec![0, 3, 2]);
        assert!(!node_ids.windows(2).any(|hop| hop == [1, 2]));

        let route = route_from_path(&g, &node_ids, &profile, cost);
        assert!((route.distance_m - 1600.0).abs() < 1e-6);
        assert!(!route.steps.is_empty());
    }
}
