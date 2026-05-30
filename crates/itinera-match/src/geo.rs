/// Haversine distance in meters between two lat/lon points.
pub fn haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let r = 6_371_000.0;
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
    2.0 * r * a.sqrt().asin()
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
    fn test_haversine_zero() {
        assert_eq!(haversine(0.0, 0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn test_haversine_known_distance() {
        // ~111 km per degree at equator
        let d = haversine(0.0, 0.0, 1.0, 0.0);
        assert!((d - 111_195.0).abs() < 100.0);
    }
}
