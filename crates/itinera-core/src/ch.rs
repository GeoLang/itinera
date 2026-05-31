use std::cmp::Ordering;
use std::collections::BinaryHeap;

use itinera_graph::{Edge, Graph, NodeId, SpeedProfile};

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
}

#[derive(Debug, Clone)]
struct CHState {
    cost: f64,
    node: NodeId,
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

        // Build adjacency lists for contraction
        let mut out_adj: Vec<Vec<(NodeId, f64, usize)>> = vec![Vec::new(); n];
        let mut in_adj: Vec<Vec<(NodeId, f64, usize)>> = vec![Vec::new(); n];

        for (idx, edge) in edges.iter().enumerate() {
            let weight = graph.edge_weight(edge, profile);
            if weight < f64::INFINITY {
                out_adj[edge.from.0 as usize].push((edge.to, weight, idx));
                in_adj[edge.to.0 as usize].push((edge.from, weight, idx));
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
                let shortcuts = count_shortcuts_needed(node_idx, &out_adj, &in_adj, &contracted);
                let in_degree = in_adj[node_idx]
                    .iter()
                    .filter(|(from, _, _)| !contracted[from.0 as usize])
                    .count() as i64;
                let out_degree = out_adj[node_idx]
                    .iter()
                    .filter(|(to, _, _)| !contracted[to.0 as usize])
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
                .filter(|(from, _, _)| !contracted[from.0 as usize])
                .cloned()
                .collect();
            let outgoing: Vec<_> = out_adj[v]
                .iter()
                .filter(|(to, _, _)| !contracted[to.0 as usize])
                .cloned()
                .collect();

            for &(u, w_uv, _) in &incoming {
                for &(w, w_vw, _) in &outgoing {
                    if u == w {
                        continue;
                    }
                    let shortcut_cost = w_uv + w_vw;

                    if needs_shortcut(u, w, shortcut_cost, v, &out_adj, &contracted) {
                        let edge_idx = edges.len();
                        let sc_distance = shortcut_cost * 50.0 / 3.6;
                        edges.push(Edge {
                            from: u,
                            to: w,
                            distance_m: sc_distance,
                            duration_s: shortcut_cost,
                            way_id: -1,
                            road_class: 0,
                            oneway: true,
                            name: None,
                            geometry: Vec::new(),
                        });
                        shortcut_middle.push(Some(NodeId(v as u32)));
                        out_adj[u.0 as usize].push((w, shortcut_cost, edge_idx));
                        in_adj[w.0 as usize].push((u, shortcut_cost, edge_idx));
                    }
                }
            }
        }

        // To preserve shortcut_middle alignment after Graph::build sorts edges,
        // sort the parallel arrays together.
        let mut edge_with_middle: Vec<(Edge, Option<NodeId>)> =
            edges.into_iter().zip(shortcut_middle).collect();
        edge_with_middle.sort_by_key(|(e, _)| e.from);

        let sorted_edges: Vec<Edge> = edge_with_middle.iter().map(|(e, _)| e.clone()).collect();
        let sorted_middle: Vec<Option<NodeId>> =
            edge_with_middle.into_iter().map(|(_, m)| m).collect();

        nodes.sort_by_key(|node| node.id);

        // Build CSR offsets manually (same logic as Graph::build)
        let num_nodes = nodes.len();
        let mut offsets = vec![0u32; num_nodes + 1];
        for edge in &sorted_edges {
            let src = edge.from.0 as usize;
            if src < num_nodes {
                offsets[src + 1] += 1;
            }
        }
        for i in 1..=num_nodes {
            offsets[i] += offsets[i - 1];
        }

        // Build reverse CSR
        let mut rev_offsets = vec![0u32; num_nodes + 1];
        for edge in &sorted_edges {
            let tgt = edge.to.0 as usize;
            if tgt < num_nodes {
                rev_offsets[tgt + 1] += 1;
            }
        }
        for i in 1..=num_nodes {
            rev_offsets[i] += rev_offsets[i - 1];
        }

        let mut rev_edge_indices = vec![0u32; sorted_edges.len()];
        let mut rev_pos = rev_offsets.clone();
        for (idx, edge) in sorted_edges.iter().enumerate() {
            let tgt = edge.to.0 as usize;
            if tgt < num_nodes {
                let pos = rev_pos[tgt] as usize;
                rev_edge_indices[pos] = idx as u32;
                rev_pos[tgt] += 1;
            }
        }

        let augmented = Graph {
            nodes,
            edges: sorted_edges,
            offsets,
            rev_edge_indices,
            rev_offsets,
            restrictions: Vec::new(),
        };

        Self {
            graph: augmented,
            node_order,
            shortcut_middle: sorted_middle,
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

        let mut fwd_dist = vec![f64::INFINITY; n];
        let mut fwd_prev: Vec<Option<(u32, usize)>> = vec![None; n];
        let mut fwd_visited = vec![false; n];
        fwd_dist[src_idx] = 0.0;

        let mut bwd_dist = vec![f64::INFINITY; n];
        let mut bwd_prev: Vec<Option<(u32, usize)>> = vec![None; n];
        let mut bwd_visited = vec![false; n];
        bwd_dist[tgt_idx] = 0.0;

        let mut fwd_heap = BinaryHeap::new();
        let mut bwd_heap = BinaryHeap::new();

        fwd_heap.push(CHState {
            cost: 0.0,
            node: source,
        });
        bwd_heap.push(CHState {
            cost: 0.0,
            node: target,
        });

        let mut best_cost = f64::INFINITY;
        let mut meeting_node: Option<NodeId> = None;

        loop {
            let fwd_done = fwd_heap.is_empty();
            let bwd_done = bwd_heap.is_empty();

            if fwd_done && bwd_done {
                break;
            }

            // Forward step
            if let Some(CHState { cost, node }) = fwd_heap.pop() {
                let node_idx = node.0 as usize;

                if cost > best_cost {
                    // prune
                } else if !fwd_visited[node_idx] {
                    fwd_visited[node_idx] = true;

                    if bwd_dist[node_idx] < f64::INFINITY {
                        let total = cost + bwd_dist[node_idx];
                        if total < best_cost {
                            best_cost = total;
                            meeting_node = Some(node);
                        }
                    }

                    let node_level = self.graph.nodes[node_idx].ch_level;
                    let start = self.graph.offsets[node_idx] as usize;
                    let end = self.graph.offsets[node_idx + 1] as usize;
                    for edge_idx in start..end {
                        let edge = &self.graph.edges[edge_idx];
                        let to_idx = edge.to.0 as usize;
                        if to_idx < n && self.graph.nodes[to_idx].ch_level >= node_level {
                            let weight = self.graph.edge_weight(edge, profile);
                            if weight < f64::INFINITY {
                                let new_cost = cost + weight;
                                if new_cost < fwd_dist[to_idx] {
                                    fwd_dist[to_idx] = new_cost;
                                    fwd_prev[to_idx] = Some((node.0, edge_idx));
                                    fwd_heap.push(CHState {
                                        cost: new_cost,
                                        node: edge.to,
                                    });
                                }
                            }
                        }
                    }
                }
            }

            // Backward step (using reverse CSR)
            if let Some(CHState { cost, node }) = bwd_heap.pop() {
                let node_idx = node.0 as usize;

                if cost > best_cost {
                    // prune
                } else if !bwd_visited[node_idx] {
                    bwd_visited[node_idx] = true;

                    if fwd_dist[node_idx] < f64::INFINITY {
                        let total = fwd_dist[node_idx] + cost;
                        if total < best_cost {
                            best_cost = total;
                            meeting_node = Some(node);
                        }
                    }

                    let node_level = self.graph.nodes[node_idx].ch_level;
                    let rev_start = self.graph.rev_offsets[node_idx] as usize;
                    let rev_end = self.graph.rev_offsets[node_idx + 1] as usize;
                    for &ei in &self.graph.rev_edge_indices[rev_start..rev_end] {
                        let edge = &self.graph.edges[ei as usize];
                        let from_idx = edge.from.0 as usize;
                        if from_idx < n && self.graph.nodes[from_idx].ch_level >= node_level {
                            let weight = self.graph.edge_weight(edge, profile);
                            if weight < f64::INFINITY {
                                let new_cost = cost + weight;
                                if new_cost < bwd_dist[from_idx] {
                                    bwd_dist[from_idx] = new_cost;
                                    bwd_prev[from_idx] = Some((node.0, ei as usize));
                                    bwd_heap.push(CHState {
                                        cost: new_cost,
                                        node: edge.from,
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

        let meeting = meeting_node?;

        // Reconstruct forward path (source -> meeting)
        let mut fwd_path = Vec::new();
        {
            let mut current = meeting.0 as usize;
            while current != src_idx {
                fwd_path.push(NodeId(current as u32));
                let (prev_node, _) = fwd_prev[current]?;
                current = prev_node as usize;
            }
            fwd_path.push(source);
            fwd_path.reverse();
        }

        // Reconstruct backward path (meeting -> target)
        let mut bwd_path = Vec::new();
        {
            let mut current = meeting.0 as usize;
            while current != tgt_idx {
                let (prev_node, _) = bwd_prev[current]?;
                current = prev_node as usize;
                bwd_path.push(NodeId(current as u32));
            }
        }

        let mut packed_path = fwd_path;
        packed_path.extend(bwd_path);

        // Unpack shortcuts into full path
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
                let w = self.graph.edge_weight(edge, profile);
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
    out_adj: &[Vec<(NodeId, f64, usize)>],
    in_adj: &[Vec<(NodeId, f64, usize)>],
    contracted: &[bool],
) -> i64 {
    let incoming: Vec<_> = in_adj[v]
        .iter()
        .filter(|(from, _, _)| !contracted[from.0 as usize])
        .collect();
    let outgoing: Vec<_> = out_adj[v]
        .iter()
        .filter(|(to, _, _)| !contracted[to.0 as usize])
        .collect();

    let mut count = 0i64;
    for &(u, w_uv, _) in &incoming {
        for &(w, w_vw, _) in &outgoing {
            if u == w {
                continue;
            }
            let shortcut_cost = w_uv + w_vw;
            if needs_shortcut(*u, *w, shortcut_cost, v, out_adj, contracted) {
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
    out_adj: &[Vec<(NodeId, f64, usize)>],
    contracted: &[bool],
) -> bool {
    let n = out_adj.len();
    let mut dist = vec![f64::INFINITY; n];
    let mut heap = BinaryHeap::new();

    dist[u.0 as usize] = 0.0;
    heap.push(CHState { cost: 0.0, node: u });

    let max_settle = 5 * n.min(100);
    let mut settled = 0;

    while let Some(CHState { cost, node }) = heap.pop() {
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

        for &(next, weight, _) in &out_adj[node_idx] {
            let next_idx = next.0 as usize;
            if next_idx == v || contracted[next_idx] {
                continue;
            }
            let new_cost = cost + weight;
            if new_cost < dist[next_idx] && new_cost <= shortcut_cost {
                dist[next_idx] = new_cost;
                heap.push(CHState {
                    cost: new_cost,
                    node: next,
                });
            }
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use itinera_graph::{Coord, Node};

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
}
