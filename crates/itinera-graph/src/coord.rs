use serde::{Deserialize, Serialize};

pub(crate) const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// WGS84 coordinate.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Coord {
    pub lat: f64,
    pub lon: f64,
}

impl Coord {
    #[must_use]
    pub fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }

    /// Haversine distance in meters.
    #[must_use]
    pub fn distance_to(self, other: Self) -> f64 {
        let d_lat = (other.lat - self.lat).to_radians();
        let d_lon = (other.lon - self.lon).to_radians();
        let lat1 = self.lat.to_radians();
        let lat2 = other.lat.to_radians();

        let a = (d_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (d_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().asin();
        EARTH_RADIUS_M * c
    }

    /// Bearing from self to other in degrees [0, 360).
    #[must_use]
    pub fn bearing_to(self, other: Self) -> f64 {
        let lat1 = self.lat.to_radians();
        let lat2 = other.lat.to_radians();
        let d_lon = (other.lon - self.lon).to_radians();

        let y = d_lon.sin() * lat2.cos();
        let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * d_lon.cos();
        let bearing = y.atan2(x).to_degrees();
        (bearing + 360.0) % 360.0
    }
}
