use rstar::primitives::{GeomWithData, Rectangle};
use rstar::{AABB, RTree};
use uuid::Uuid;

use crate::geo::{METERS_PER_DEGREE, haversine, point_to_segment_distance};
use crate::types::RoadSegment;

/// Bounding rectangle of a segment's geometry, tagged with its index in `segments`.
type IndexedSegment = GeomWithData<Rectangle<[f64; 2]>, usize>;

const POLE_LATITUDE_DEG: f64 = 90.0;
/// Half the width of the world, so a wider span can only cover it twice.
const MAX_LONGITUDE_SPAN_DEG: f64 = 180.0;

/// A road network for map matching.
pub struct RoadNetwork {
    pub segments: Vec<RoadSegment>,
    index: RTree<IndexedSegment>,
}

impl RoadNetwork {
    pub fn new(segments: Vec<RoadSegment>) -> Self {
        let index = RTree::bulk_load(
            segments
                .iter()
                .enumerate()
                .filter(|(_, seg)| seg.geometry.len() >= 2)
                .map(|(idx, seg)| IndexedSegment::new(bounding_rectangle(&seg.geometry), idx))
                .collect(),
        );
        Self { segments, index }
    }

    /// Find candidate road segments within search_radius_m of a point.
    /// Returns (segment_index, distance_m, snapped_point), ordered by segment index.
    pub fn candidates(&self, lon: f64, lat: f64, radius_m: f64) -> Vec<(usize, f64, [f64; 2])> {
        let mut nearby: Vec<usize> = self
            .index
            .locate_in_envelope_intersecting(&search_envelope(lon, lat, radius_m))
            .map(|indexed| indexed.data)
            .collect();
        nearby.sort_unstable();

        let mut result = Vec::new();
        for idx in nearby {
            let seg = &self.segments[idx];
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

fn bounding_rectangle(geometry: &[[f64; 2]]) -> Rectangle<[f64; 2]> {
    let mut min = [f64::MAX, f64::MAX];
    let mut max = [f64::MIN, f64::MIN];
    for point in geometry {
        min[0] = min[0].min(point[0]);
        min[1] = min[1].min(point[1]);
        max[0] = max[0].max(point[0]);
        max[1] = max[1].max(point[1]);
    }
    Rectangle::from_corners(min, max)
}

/// Box around a point holding every segment that could be within radius_m.
fn search_envelope(lon: f64, lat: f64, radius_m: f64) -> AABB<[f64; 2]> {
    let latitude_span = radius_m / METERS_PER_DEGREE;
    // a degree of longitude is shortest at the edge of the box furthest from the equator
    let widest_latitude = (lat.abs() + latitude_span).min(POLE_LATITUDE_DEG);
    let longitude_span =
        (latitude_span / widest_latitude.to_radians().cos()).min(MAX_LONGITUDE_SPAN_DEG);
    AABB::from_corners(
        [lon - longitude_span, lat - latitude_span],
        [lon + longitude_span, lat + latitude_span],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn candidates_match_a_full_scan_of_the_demo_network() {
        let network = RoadNetwork::demo();
        let radius_m = 150.0;
        for (lon, lat) in [
            (-122.4194, 37.7749),
            (-122.3940, 37.7800),
            (-122.4260, 37.7715),
            (-100.0, 40.0),
        ] {
            let scanned: Vec<usize> = network
                .segments
                .iter()
                .enumerate()
                .filter(|(_, seg)| seg.geometry.len() >= 2)
                .filter_map(|(idx, seg)| {
                    let best = seg
                        .geometry
                        .windows(2)
                        .map(|w| {
                            let (_d, snap) = point_to_segment_distance([lon, lat], w[0], w[1]);
                            haversine(lat, lon, snap[1], snap[0])
                        })
                        .fold(f64::MAX, f64::min);
                    (best <= radius_m).then_some(idx)
                })
                .collect();

            let indexed: Vec<usize> = network
                .candidates(lon, lat, radius_m)
                .into_iter()
                .map(|(idx, _, _)| idx)
                .collect();
            assert_eq!(indexed, scanned);
        }
    }

    #[test]
    fn search_envelope_covers_the_radius_near_the_pole() {
        let envelope = search_envelope(0.0, 89.999, 500.0);
        assert!(envelope.lower()[0] <= -MAX_LONGITUDE_SPAN_DEG);
        assert!(envelope.upper()[0] >= MAX_LONGITUDE_SPAN_DEG);
    }
}
