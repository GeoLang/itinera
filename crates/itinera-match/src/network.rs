use uuid::Uuid;

use crate::geo::{haversine, point_to_segment_distance};
use crate::types::RoadSegment;

/// A road network for map matching.
pub struct RoadNetwork {
    pub segments: Vec<RoadSegment>,
}

impl RoadNetwork {
    pub fn new(segments: Vec<RoadSegment>) -> Self {
        Self { segments }
    }

    /// Find candidate road segments within search_radius_m of a point.
    /// Returns (segment_index, distance_m, snapped_point).
    pub fn candidates(&self, lon: f64, lat: f64, radius_m: f64) -> Vec<(usize, f64, [f64; 2])> {
        let mut result = Vec::new();
        for (idx, seg) in self.segments.iter().enumerate() {
            if seg.geometry.len() < 2 {
                continue;
            }
            let mut best_dist = f64::MAX;
            let mut best_snap = [lon, lat];
            for w in seg.geometry.windows(2) {
                let (_d, snap) = point_to_segment_distance([lon, lat], w[0], w[1]);
                let d_m = haversine(lat, lon, snap[1], snap[0]);
                if d_m < best_dist {
                    best_dist = d_m;
                    best_snap = snap;
                }
            }
            if best_dist <= radius_m {
                result.push((idx, best_dist, best_snap));
            }
        }
        result
    }

    /// Create a demo road network around San Francisco.
    pub fn demo() -> Self {
        Self::new(vec![
            RoadSegment {
                id: Uuid::new_v4(),
                name: "Market Street".into(),
                road_class: "primary".into(),
                geometry: vec![
                    [-122.4260, 37.7700],
                    [-122.4200, 37.7740],
                    [-122.4150, 37.7770],
                    [-122.4100, 37.7800],
                    [-122.4050, 37.7830],
                ],
                speed_limit_kmh: 40.0,
                oneway: false,
            },
            RoadSegment {
                id: Uuid::new_v4(),
                name: "Mission Street".into(),
                road_class: "secondary".into(),
                geometry: vec![
                    [-122.4260, 37.7685],
                    [-122.4200, 37.7720],
                    [-122.4150, 37.7750],
                    [-122.4100, 37.7780],
                    [-122.4050, 37.7810],
                ],
                speed_limit_kmh: 35.0,
                oneway: false,
            },
            RoadSegment {
                id: Uuid::new_v4(),
                name: "3rd Street".into(),
                road_class: "secondary".into(),
                geometry: vec![
                    [-122.3940, 37.7700],
                    [-122.3940, 37.7750],
                    [-122.3940, 37.7800],
                    [-122.3940, 37.7850],
                ],
                speed_limit_kmh: 35.0,
                oneway: false,
            },
            RoadSegment {
                id: Uuid::new_v4(),
                name: "Howard Street".into(),
                road_class: "secondary".into(),
                geometry: vec![
                    [-122.4260, 37.7730],
                    [-122.4200, 37.7730],
                    [-122.4150, 37.7730],
                    [-122.4100, 37.7730],
                ],
                speed_limit_kmh: 30.0,
                oneway: true,
            },
            RoadSegment {
                id: Uuid::new_v4(),
                name: "Folsom Street".into(),
                road_class: "secondary".into(),
                geometry: vec![
                    [-122.4260, 37.7715],
                    [-122.4200, 37.7715],
                    [-122.4150, 37.7715],
                    [-122.4100, 37.7715],
                ],
                speed_limit_kmh: 30.0,
                oneway: true,
            },
        ])
    }
}
