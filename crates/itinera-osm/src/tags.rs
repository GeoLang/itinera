/// Map OSM highway tag to road class (1-7).
/// Returns 0 for non-routable highways.
#[must_use]
pub fn highway_to_road_class(highway: &str) -> u8 {
    match highway {
        "motorway" | "motorway_link" => 1,
        "trunk" | "trunk_link" => 2,
        "primary" | "primary_link" => 3,
        "secondary" | "secondary_link" => 4,
        "tertiary" | "tertiary_link" => 5,
        "unclassified" | "road" => 6,
        "residential" | "living_street" | "service" => 7,
        _ => 0,
    }
}

/// Determine if a way is one-way from OSM tags.
#[must_use]
pub fn is_oneway(tags: &[(String, String)], highway: &str) -> bool {
    // Motorways are one-way by default
    if highway == "motorway" || highway == "motorway_link" {
        // Check for explicit two-way override
        if tags.iter().any(|(k, v)| k == "oneway" && v == "no") {
            return false;
        }
        return true;
    }

    tags.iter()
        .any(|(k, v)| k == "oneway" && (v == "yes" || v == "1" || v == "true"))
}

/// Extract max speed from OSM tags (km/h). Returns None if not specified.
#[must_use]
pub fn max_speed_from_tags(tags: &[(String, String)]) -> Option<f64> {
    for (k, v) in tags {
        if k == "maxspeed" {
            // Handle "50", "50 km/h", "30 mph"
            let v = v.trim();
            if let Some(mph_str) = v.strip_suffix("mph") {
                if let Ok(mph) = mph_str.trim().parse::<f64>() {
                    return Some(mph * 1.60934);
                }
            } else if let Some(kmh_str) = v.strip_suffix("km/h") {
                if let Ok(kmh) = kmh_str.trim().parse::<f64>() {
                    return Some(kmh);
                }
            } else if let Ok(kmh) = v.parse::<f64>() {
                return Some(kmh);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_highway_classes() {
        assert_eq!(highway_to_road_class("motorway"), 1);
        assert_eq!(highway_to_road_class("primary"), 3);
        assert_eq!(highway_to_road_class("residential"), 7);
        assert_eq!(highway_to_road_class("footway"), 0);
    }

    #[test]
    fn every_road_class_name_imports_back_to_its_own_class() {
        for name in itinera_graph::ROAD_CLASS_NAMES {
            assert_eq!(
                itinera_graph::road_class_name(highway_to_road_class(name)),
                name
            );
        }
    }

    #[test]
    fn test_oneway() {
        let tags = vec![("oneway".to_string(), "yes".to_string())];
        assert!(is_oneway(&tags, "primary"));

        let tags_no: Vec<(String, String)> = vec![];
        assert!(!is_oneway(&tags_no, "primary"));

        // Motorway default
        assert!(is_oneway(&[], "motorway"));
    }

    #[test]
    fn test_maxspeed() {
        let tags = vec![("maxspeed".to_string(), "50".to_string())];
        assert_eq!(max_speed_from_tags(&tags), Some(50.0));

        let tags_mph = vec![("maxspeed".to_string(), "30 mph".to_string())];
        let speed = max_speed_from_tags(&tags_mph).unwrap();
        assert!((speed - 48.28).abs() < 0.1);

        let tags_none: Vec<(String, String)> = vec![];
        assert_eq!(max_speed_from_tags(&tags_none), None);
    }
}
