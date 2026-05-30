use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// A raw GPS trace point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsPoint {
    pub latitude: f64,
    pub longitude: f64,
    pub timestamp: Option<f64>,
    pub accuracy_m: Option<f64>,
    pub speed_mps: Option<f64>,
    pub bearing_deg: Option<f64>,
}

/// Map matching request.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapMatchRequest {
    pub trace: Vec<GpsPoint>,
    pub profile: MatchProfile,
    pub search_radius_m: f64,
}

/// Matching profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum MatchProfile {
    Driving,
    Walking,
    Cycling,
}

/// Map matching result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapMatchResult {
    pub id: Uuid,
    pub matched_points: Vec<MatchedPoint>,
    pub matched_route: Vec<[f64; 2]>,
    pub confidence: f64,
    pub total_distance_m: f64,
    pub road_segments: Vec<MatchedSegment>,
}

/// A matched (snapped) point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatchedPoint {
    pub original: [f64; 2],
    pub snapped: [f64; 2],
    pub distance_from_road_m: f64,
    pub road_name: Option<String>,
}

/// A matched road segment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatchedSegment {
    pub road_name: String,
    pub road_class: String,
    pub distance_m: f64,
    pub duration_secs: f64,
    pub speed_kmh: f64,
}

/// A road segment in the network.
#[derive(Debug, Clone)]
pub struct RoadSegment {
    pub id: Uuid,
    pub name: String,
    pub road_class: String,
    pub geometry: Vec<[f64; 2]>,
    pub speed_limit_kmh: f64,
    pub oneway: bool,
}
