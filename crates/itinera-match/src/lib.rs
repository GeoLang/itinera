//! # itinera-match
//!
//! HMM-based map matching: snap noisy GPS traces to road networks
//! using the Viterbi algorithm with emission and transition probabilities.

mod geo;
mod hmm;
mod network;
mod types;

pub use hmm::match_trace;
pub use network::RoadNetwork;
pub use types::{
    GpsPoint, MapMatchRequest, MapMatchResult, MatchProfile, MatchedPoint, MatchedSegment,
    RoadSegment,
};
