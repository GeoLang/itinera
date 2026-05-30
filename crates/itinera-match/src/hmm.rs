//! HMM-based map matching using Viterbi algorithm.

use std::collections::HashMap;

use uuid::Uuid;

use crate::geo::haversine;
use crate::network::RoadNetwork;
use crate::types::{MapMatchRequest, MapMatchResult, MatchProfile, MatchedPoint, MatchedSegment};

/// Perform map matching on a GPS trace using a road network.
///
/// Uses a Hidden Markov Model with:
/// - Gaussian emission probabilities (GPS noise model)
/// - Exponential transition probabilities (route vs. GPS distance)
/// - Connectivity bonus for same-segment transitions
pub fn match_trace(request: &MapMatchRequest, network: &RoadNetwork) -> MapMatchResult {
    let sigma_z = 4.07; // GPS noise standard deviation (meters)
    let beta = 2.0; // transition probability parameter

    let n_points = request.trace.len();
    if n_points == 0 {
        return MapMatchResult {
            id: Uuid::new_v4(),
            matched_points: vec![],
            matched_route: vec![],
            confidence: 0.0,
            total_distance_m: 0.0,
            road_segments: vec![],
        };
    }

    // Collect candidates for each GPS point
    let all_candidates: Vec<Vec<(usize, f64, [f64; 2])>> = request
        .trace
        .iter()
        .map(|gps| network.candidates(gps.longitude, gps.latitude, request.search_radius_m))
        .collect();

    for (i, cands) in all_candidates.iter().enumerate() {
        if cands.is_empty() {
            tracing::debug!(point_index = i, "no road candidates within search radius");
        }
    }

    // Viterbi algorithm
    let mut viterbi_prob: Vec<Vec<f64>> = Vec::with_capacity(n_points);
    let mut viterbi_prev: Vec<Vec<usize>> = Vec::with_capacity(n_points);

    // Initialization
    if all_candidates[0].is_empty() {
        return build_fallback_result(request);
    }
    let init_probs: Vec<f64> = all_candidates[0]
        .iter()
        .map(|(_, dist, _)| emission_log_prob(*dist, sigma_z))
        .collect();
    viterbi_prob.push(init_probs);
    viterbi_prev.push(vec![0; all_candidates[0].len()]);

    // Recursion
    for t in 1..n_points {
        if all_candidates[t].is_empty() {
            viterbi_prob.push(vec![]);
            viterbi_prev.push(vec![]);
            continue;
        }
        let prev_cands = &all_candidates[t - 1];
        let curr_cands = &all_candidates[t];
        let prev_probs = &viterbi_prob[t - 1];

        let gps_dist = haversine(
            request.trace[t - 1].latitude,
            request.trace[t - 1].longitude,
            request.trace[t].latitude,
            request.trace[t].longitude,
        );

        let mut probs = vec![f64::NEG_INFINITY; curr_cands.len()];
        let mut prevs = vec![0usize; curr_cands.len()];

        for (j, (curr_seg_idx, curr_dist, _)) in curr_cands.iter().enumerate() {
            let emission = emission_log_prob(*curr_dist, sigma_z);

            for (i, (prev_seg_idx, _, prev_snap)) in prev_cands.iter().enumerate() {
                if prev_probs.is_empty() {
                    continue;
                }
                let prev_snap_pt = *prev_snap;
                let curr_snap_pt = curr_cands[j].2;

                let route_dist = haversine(
                    prev_snap_pt[1],
                    prev_snap_pt[0],
                    curr_snap_pt[1],
                    curr_snap_pt[0],
                );

                let transition = transition_log_prob(route_dist, gps_dist, beta);

                let connectivity_bonus = if curr_seg_idx == prev_seg_idx {
                    0.5_f64.ln()
                } else {
                    0.0
                };

                let prob = prev_probs[i] + transition + emission + connectivity_bonus;
                if prob > probs[j] {
                    probs[j] = prob;
                    prevs[j] = i;
                }
            }
        }
        viterbi_prob.push(probs);
        viterbi_prev.push(prevs);
    }

    // Backtrack
    let mut best_sequence: Vec<Option<usize>> = vec![None; n_points];
    let last_valid = (0..n_points).rev().find(|&t| !viterbi_prob[t].is_empty());
    if let Some(last_t) = last_valid {
        let mut best_j = 0;
        let mut best_p = f64::NEG_INFINITY;
        for (j, &p) in viterbi_prob[last_t].iter().enumerate() {
            if p > best_p {
                best_p = p;
                best_j = j;
            }
        }
        best_sequence[last_t] = Some(best_j);

        let mut j = best_j;
        for t in (1..=last_t).rev() {
            if !viterbi_prev[t].is_empty() {
                j = viterbi_prev[t][j];
                if !viterbi_prob[t - 1].is_empty() {
                    best_sequence[t - 1] = Some(j);
                }
            }
        }
    }

    // Build result
    let mut matched_points = Vec::with_capacity(n_points);
    let mut route = Vec::with_capacity(n_points);
    let mut total_distance = 0.0;
    let mut segment_distances: HashMap<usize, f64> = HashMap::new();
    let mut confidence_sum = 0.0;
    let mut confidence_count = 0;

    for (t, gps) in request.trace.iter().enumerate() {
        if let Some(cand_idx) = best_sequence[t]
            && cand_idx < all_candidates[t].len()
        {
            let (seg_idx, dist, snap) = &all_candidates[t][cand_idx];
            let seg = &network.segments[*seg_idx];
            matched_points.push(MatchedPoint {
                original: [gps.longitude, gps.latitude],
                snapped: *snap,
                distance_from_road_m: *dist,
                road_name: Some(seg.name.clone()),
            });
            route.push(*snap);

            if t > 0
                && let Some(prev) = route.get(route.len().wrapping_sub(2))
            {
                let d = haversine(prev[1], prev[0], snap[1], snap[0]);
                total_distance += d;
                *segment_distances.entry(*seg_idx).or_insert(0.0) += d;
            }

            let conf = (-0.5 * (dist / sigma_z).powi(2)).exp();
            confidence_sum += conf;
            confidence_count += 1;
            continue;
        }
        matched_points.push(MatchedPoint {
            original: [gps.longitude, gps.latitude],
            snapped: [gps.longitude, gps.latitude],
            distance_from_road_m: 0.0,
            road_name: None,
        });
        route.push([gps.longitude, gps.latitude]);
    }

    let confidence = if confidence_count > 0 {
        (confidence_sum / confidence_count as f64).clamp(0.0, 1.0)
    } else {
        0.0
    };

    let mut road_segments = Vec::new();
    for (seg_idx, dist_m) in &segment_distances {
        let seg = &network.segments[*seg_idx];
        let speed = match request.profile {
            MatchProfile::Driving => seg.speed_limit_kmh,
            MatchProfile::Cycling => seg.speed_limit_kmh.min(25.0),
            MatchProfile::Walking => 5.0,
        };
        let duration = if speed > 0.0 {
            dist_m / (speed * 1000.0 / 3600.0)
        } else {
            0.0
        };
        road_segments.push(MatchedSegment {
            road_name: seg.name.clone(),
            road_class: seg.road_class.clone(),
            distance_m: *dist_m,
            duration_secs: duration,
            speed_kmh: speed,
        });
    }
    road_segments.sort_by(|a, b| b.distance_m.partial_cmp(&a.distance_m).unwrap());

    MapMatchResult {
        id: Uuid::new_v4(),
        matched_points,
        matched_route: route,
        confidence,
        total_distance_m: total_distance,
        road_segments,
    }
}

/// Gaussian emission log-probability.
fn emission_log_prob(distance_m: f64, sigma_z: f64) -> f64 {
    -0.5 * (distance_m / sigma_z).powi(2) - (sigma_z * (2.0 * std::f64::consts::PI).sqrt()).ln()
}

/// Exponential transition log-probability.
fn transition_log_prob(route_dist: f64, gps_dist: f64, beta: f64) -> f64 {
    let diff = (route_dist - gps_dist).abs();
    -diff / beta - beta.ln()
}

/// Fallback result when no candidates are found.
fn build_fallback_result(request: &MapMatchRequest) -> MapMatchResult {
    let matched_points: Vec<MatchedPoint> = request
        .trace
        .iter()
        .map(|gps| MatchedPoint {
            original: [gps.longitude, gps.latitude],
            snapped: [gps.longitude, gps.latitude],
            distance_from_road_m: 0.0,
            road_name: None,
        })
        .collect();
    let route: Vec<[f64; 2]> = request
        .trace
        .iter()
        .map(|gps| [gps.longitude, gps.latitude])
        .collect();
    MapMatchResult {
        id: Uuid::new_v4(),
        matched_points,
        matched_route: route,
        confidence: 0.0,
        total_distance_m: 0.0,
        road_segments: vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::RoadNetwork;
    use crate::types::{GpsPoint, MatchProfile};

    #[test]
    fn test_map_match() {
        let network = RoadNetwork::demo();
        let req = MapMatchRequest {
            trace: vec![
                GpsPoint {
                    latitude: 37.7749,
                    longitude: -122.4194,
                    timestamp: Some(1000.0),
                    accuracy_m: Some(10.0),
                    speed_mps: None,
                    bearing_deg: None,
                },
                GpsPoint {
                    latitude: 37.7755,
                    longitude: -122.4185,
                    timestamp: Some(1005.0),
                    accuracy_m: Some(8.0),
                    speed_mps: None,
                    bearing_deg: None,
                },
                GpsPoint {
                    latitude: 37.7762,
                    longitude: -122.4170,
                    timestamp: Some(1010.0),
                    accuracy_m: Some(12.0),
                    speed_mps: None,
                    bearing_deg: None,
                },
            ],
            profile: MatchProfile::Driving,
            search_radius_m: 200.0,
        };
        let result = match_trace(&req, &network);
        assert_eq!(result.matched_points.len(), 3);
        assert!(result.confidence > 0.0);
        assert!(result.total_distance_m > 0.0);
    }

    #[test]
    fn test_empty_trace() {
        let network = RoadNetwork::demo();
        let req = MapMatchRequest {
            trace: vec![],
            profile: MatchProfile::Driving,
            search_radius_m: 50.0,
        };
        let result = match_trace(&req, &network);
        assert!(result.matched_points.is_empty());
        assert_eq!(result.confidence, 0.0);
    }

    #[test]
    fn test_different_traces() {
        let network = RoadNetwork::demo();
        let req_market = MapMatchRequest {
            trace: vec![
                GpsPoint {
                    latitude: 37.7740,
                    longitude: -122.4200,
                    timestamp: Some(0.0),
                    accuracy_m: None,
                    speed_mps: None,
                    bearing_deg: None,
                },
                GpsPoint {
                    latitude: 37.7770,
                    longitude: -122.4150,
                    timestamp: Some(5.0),
                    accuracy_m: None,
                    speed_mps: None,
                    bearing_deg: None,
                },
            ],
            profile: MatchProfile::Driving,
            search_radius_m: 200.0,
        };
        let req_mission = MapMatchRequest {
            trace: vec![
                GpsPoint {
                    latitude: 37.7720,
                    longitude: -122.4200,
                    timestamp: Some(0.0),
                    accuracy_m: None,
                    speed_mps: None,
                    bearing_deg: None,
                },
                GpsPoint {
                    latitude: 37.7750,
                    longitude: -122.4150,
                    timestamp: Some(5.0),
                    accuracy_m: None,
                    speed_mps: None,
                    bearing_deg: None,
                },
            ],
            profile: MatchProfile::Driving,
            search_radius_m: 200.0,
        };

        let result_market = match_trace(&req_market, &network);
        let result_mission = match_trace(&req_mission, &network);

        let market_road = result_market.matched_points[0].road_name.as_deref();
        let mission_road = result_mission.matched_points[0].road_name.as_deref();
        assert_ne!(market_road, mission_road);
    }
}
