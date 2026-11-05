use itinera_graph::{Coord, EARTH_RADIUS_M};

pub const METERS_PER_DEGREE: f64 = EARTH_RADIUS_M * std::f64::consts::PI / 180.0;

pub fn lon_lat_coord(point: [f64; 2]) -> Coord {
    Coord::new(point[1], point[0])
}

/// Perpendicular distance from a point to a line segment, and the nearest point.
/// All coordinates are [lon, lat]. Returns (distance_degrees, snapped_point).
pub fn point_to_segment_distance(
    point: [f64; 2],
    seg_start: [f64; 2],
    seg_end: [f64; 2],
) -> (f64, [f64; 2]) {
    let dx = seg_end[0] - seg_start[0];
    let dy = seg_end[1] - seg_start[1];
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-14 {
        let d = ((point[0] - seg_start[0]).powi(2) + (point[1] - seg_start[1]).powi(2)).sqrt();
        return (d, seg_start);
    }
    let t = ((point[0] - seg_start[0]) * dx + (point[1] - seg_start[1]) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let snap = [seg_start[0] + t * dx, seg_start[1] + t * dy];
    let d = ((point[0] - snap[0]).powi(2) + (point[1] - snap[1]).powi(2)).sqrt();
    (d, snap)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_to_segment_distance() {
        let (d, snap) = point_to_segment_distance([0.5, 0.0], [0.0, 0.0], [1.0, 0.0]);
        assert!(d < 1e-10);
        assert!((snap[0] - 0.5).abs() < 1e-10);

        let (d, snap) = point_to_segment_distance([0.5, 1.0], [0.0, 0.0], [1.0, 0.0]);
        assert!((d - 1.0).abs() < 1e-10);
        assert!((snap[0] - 0.5).abs() < 1e-10);
        assert!(snap[1].abs() < 1e-10);
    }

    #[test]
    fn lon_lat_coord_swaps_the_axes() {
        assert_eq!(lon_lat_coord([7.41, 43.72]), Coord::new(43.72, 7.41));
    }
}
