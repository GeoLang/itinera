use serde::{Deserialize, Serialize};

use crate::Coord;
use crate::coord::EARTH_RADIUS_M;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min_lat: f64,
    pub min_lon: f64,
    pub max_lat: f64,
    pub max_lon: f64,
}

impl BoundingBox {
    #[must_use]
    pub fn from_corners(lat_a: f64, lon_a: f64, lat_b: f64, lon_b: f64) -> Self {
        Self {
            min_lat: lat_a.min(lat_b),
            min_lon: lon_a.min(lon_b),
            max_lat: lat_a.max(lat_b),
            max_lon: lon_a.max(lon_b),
        }
    }

    #[must_use]
    pub fn enclosing(coords: impl IntoIterator<Item = Coord>) -> Option<Self> {
        let mut coords = coords.into_iter();
        let first = coords.next()?;
        let mut bbox = Self::from_corners(first.lat, first.lon, first.lat, first.lon);
        for coord in coords {
            bbox.min_lat = bbox.min_lat.min(coord.lat);
            bbox.min_lon = bbox.min_lon.min(coord.lon);
            bbox.max_lat = bbox.max_lat.max(coord.lat);
            bbox.max_lon = bbox.max_lon.max(coord.lon);
        }
        Some(bbox)
    }

    #[must_use]
    pub fn grown_by(self, distance_m: f64) -> Self {
        let latitude_degrees = (distance_m / EARTH_RADIUS_M).to_degrees();
        let widest_latitude = self.min_lat.abs().max(self.max_lat.abs());
        // longitude degrees shrink towards the poles
        let longitude_degrees = (latitude_degrees / widest_latitude.to_radians().cos()).min(180.0);
        Self {
            min_lat: (self.min_lat - latitude_degrees).max(-90.0),
            min_lon: (self.min_lon - longitude_degrees).max(-180.0),
            max_lat: (self.max_lat + latitude_degrees).min(90.0),
            max_lon: (self.max_lon + longitude_degrees).min(180.0),
        }
    }

    #[must_use]
    pub fn contains(&self, coord: Coord) -> bool {
        coord.lat >= self.min_lat
            && coord.lat <= self.max_lat
            && coord.lon >= self.min_lon
            && coord.lon <= self.max_lon
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corners_sort_themselves() {
        let bbox = BoundingBox::from_corners(48.10, 2.20, 47.90, 2.00);

        assert_eq!(bbox.min_lat, 47.90);
        assert_eq!(bbox.max_lat, 48.10);
        assert_eq!(bbox.min_lon, 2.00);
        assert_eq!(bbox.max_lon, 2.20);
    }

    #[test]
    fn enclosing_covers_every_coord() {
        let bbox = BoundingBox::enclosing([
            Coord::new(48.00, 2.00),
            Coord::new(47.50, 2.50),
            Coord::new(48.25, 1.75),
        ])
        .unwrap();

        assert_eq!(bbox.min_lat, 47.50);
        assert_eq!(bbox.max_lat, 48.25);
        assert_eq!(bbox.min_lon, 1.75);
        assert_eq!(bbox.max_lon, 2.50);
    }

    #[test]
    fn enclosing_nothing_is_none() {
        assert!(BoundingBox::enclosing([]).is_none());
    }

    #[test]
    fn growing_by_a_kilometre_moves_each_side_by_about_a_kilometre() {
        let bbox = BoundingBox::from_corners(48.00, 2.00, 48.01, 2.01).grown_by(1000.0);

        let south_west = Coord::new(bbox.min_lat, bbox.min_lon);
        assert!((Coord::new(48.00, bbox.min_lon).distance_to(south_west) - 1000.0).abs() < 1.0);
        assert!((Coord::new(bbox.min_lat, 2.00).distance_to(south_west) - 1000.0).abs() < 1.0);
    }

    #[test]
    fn growing_stops_at_the_poles_and_the_antimeridian() {
        let bbox = BoundingBox::from_corners(-89.99, -179.99, 89.99, 179.99).grown_by(500_000.0);

        assert_eq!(bbox.min_lat, -90.0);
        assert_eq!(bbox.max_lat, 90.0);
        assert_eq!(bbox.min_lon, -180.0);
        assert_eq!(bbox.max_lon, 180.0);
    }

    #[test]
    fn edges_and_corners_count_as_inside() {
        let bbox = BoundingBox::from_corners(48.00, 2.00, 48.01, 2.01);

        assert!(bbox.contains(Coord::new(48.005, 2.005)));
        assert!(bbox.contains(Coord::new(48.00, 2.00)));
        assert!(bbox.contains(Coord::new(48.01, 2.01)));
        assert!(!bbox.contains(Coord::new(48.02, 2.005)));
        assert!(!bbox.contains(Coord::new(48.005, 1.99)));
    }
}
