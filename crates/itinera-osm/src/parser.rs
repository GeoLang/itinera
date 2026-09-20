use std::collections::HashMap;
use std::io::BufRead;

use itinera_graph::{BoundingBox, Coord, Edge, Graph, Node, NodeId, TurnRestriction};

use crate::error::OsmError;
use crate::tags::{highway_to_road_class, is_oneway};

/// Statistics from an OSM import.
#[derive(Debug, Clone, Default)]
pub struct ImportStats {
    pub nodes_parsed: usize,
    pub ways_parsed: usize,
    pub edges_created: usize,
    pub nodes_in_graph: usize,
}

/// OSM road network importer.
///
/// Parses OSM XML data and produces a `Graph` suitable for routing.
pub struct OsmImporter {
    /// All parsed OSM nodes (id -> coord).
    osm_nodes: HashMap<i64, Coord>,
    /// Parsed ways.
    ways: Vec<OsmWay>,
    /// Parsed turn restrictions from relations.
    restrictions: Vec<OsmRestriction>,
    declared_bounds: Option<BoundingBox>,
}

struct OsmWay {
    way_id: i64,
    node_refs: Vec<i64>,
    tags: Vec<(String, String)>,
    highway: String,
}

struct OsmRestriction {
    from_way: i64,
    to_way: i64,
    via_node: i64,
    restriction_type: String,
}

impl OsmImporter {
    #[must_use]
    pub fn new() -> Self {
        Self {
            osm_nodes: HashMap::new(),
            ways: Vec::new(),
            restrictions: Vec::new(),
            declared_bounds: None,
        }
    }

    /// Parse OSM XML from a reader.
    pub fn parse_xml<R: BufRead>(&mut self, reader: R) -> Result<(), OsmError> {
        let mut in_way = false;
        let mut in_relation = false;
        let mut current_way: Option<OsmWay> = None;
        let mut current_relation: Option<OsmRelationBuilder> = None;

        for line in reader.lines() {
            let line = line?;
            let trimmed = line.trim();

            if trimmed.starts_with("<bounds ") {
                self.declared_bounds = parse_bounds(trimmed);
            }
            // Parse <node> elements
            else if trimmed.starts_with("<node ") {
                if let Some((id, lat, lon)) = parse_node_attrs(trimmed) {
                    self.osm_nodes.insert(id, Coord::new(lat, lon));
                }
            }
            // Parse <way> elements
            else if trimmed.starts_with("<way ") {
                in_way = true;
                let way_id = extract_attr(trimmed, "id")
                    .and_then(|s| s.parse::<i64>().ok())
                    .unwrap_or(0);
                current_way = Some(OsmWay {
                    way_id,
                    node_refs: Vec::new(),
                    tags: Vec::new(),
                    highway: String::new(),
                });
            } else if trimmed == "</way>" || trimmed.starts_with("</way>") {
                in_way = false;
                if let Some(way) = current_way.take()
                    && !way.highway.is_empty()
                    && highway_to_road_class(&way.highway) > 0
                {
                    self.ways.push(way);
                }
            }
            // Parse <relation> elements (for turn restrictions)
            else if trimmed.starts_with("<relation ") {
                in_relation = true;
                current_relation = Some(OsmRelationBuilder::new());
            } else if trimmed == "</relation>" || trimmed.starts_with("</relation>") {
                in_relation = false;
                if let Some(rel) = current_relation.take()
                    && let Some(restriction) = rel.build()
                {
                    self.restrictions.push(restriction);
                }
            } else if in_way {
                if let Some(ref mut way) = current_way {
                    if trimmed.starts_with("<nd ") {
                        if let Some(ref_id) = parse_nd_ref(trimmed) {
                            way.node_refs.push(ref_id);
                        }
                    } else if trimmed.starts_with("<tag ")
                        && let Some((k, v)) = parse_tag(trimmed)
                    {
                        if k == "highway" {
                            way.highway = v.clone();
                        }
                        way.tags.push((k, v));
                    }
                }
            } else if in_relation && let Some(ref mut rel) = current_relation {
                if trimmed.starts_with("<member ") {
                    let member_type = extract_attr(trimmed, "type").unwrap_or_default();
                    let member_ref = extract_attr(trimmed, "ref")
                        .and_then(|s| s.parse::<i64>().ok())
                        .unwrap_or(0);
                    let role = extract_attr(trimmed, "role").unwrap_or_default();

                    match (member_type.as_str(), role.as_str()) {
                        ("way", "from") => rel.from_way = Some(member_ref),
                        ("way", "to") => rel.to_way = Some(member_ref),
                        ("node", "via") => rel.via_node = Some(member_ref),
                        _ => {}
                    }
                } else if trimmed.starts_with("<tag ")
                    && let Some((k, v)) = parse_tag(trimmed)
                {
                    if k == "type" {
                        rel.relation_type = Some(v);
                    } else if k == "restriction" {
                        rel.restriction = Some(v);
                    }
                }
            }
        }

        Ok(())
    }

    /// Build a routing graph from the parsed OSM data.
    pub fn build_graph(self) -> Result<(Graph, ImportStats), OsmError> {
        let mut stats = ImportStats {
            nodes_parsed: self.osm_nodes.len(),
            ways_parsed: self.ways.len(),
            ..Default::default()
        };

        // Determine which nodes are intersections (referenced by multiple ways or are endpoints)
        let mut node_usage: HashMap<i64, u32> = HashMap::new();
        for way in &self.ways {
            for (i, &nref) in way.node_refs.iter().enumerate() {
                let count = node_usage.entry(nref).or_insert(0);
                // Endpoints always become graph nodes
                if i == 0 || i == way.node_refs.len() - 1 {
                    *count += 2;
                } else {
                    *count += 1;
                }
            }
        }

        // Nodes that appear in 2+ ways or are endpoints become graph nodes
        let graph_node_osm_ids: Vec<i64> = node_usage
            .iter()
            .filter(|&(_, &count)| count >= 2)
            .map(|(&id, _)| id)
            .collect();

        // Assign internal IDs
        let mut osm_to_internal: HashMap<i64, u32> = HashMap::new();
        let mut nodes = Vec::new();

        for (idx, &osm_id) in graph_node_osm_ids.iter().enumerate() {
            if let Some(&coord) = self.osm_nodes.get(&osm_id) {
                osm_to_internal.insert(osm_id, idx as u32);
                nodes.push(Node {
                    id: NodeId(idx as u32),
                    coord,
                    osm_id,
                    ch_level: 0,
                });
            }
        }

        stats.nodes_in_graph = nodes.len();

        // Build edges from ways
        let mut edges = Vec::new();

        for way in &self.ways {
            let road_class = highway_to_road_class(&way.highway);
            if road_class == 0 {
                continue;
            }

            let oneway = is_oneway(&way.tags, &way.highway);
            let name: Option<String> = way
                .tags
                .iter()
                .find(|(k, _)| k == "name")
                .map(|(_, v)| v.clone());

            let way_id = way.way_id;

            // Split way into segments between graph nodes
            let mut segment_start = 0;
            for i in 1..way.node_refs.len() {
                let nref = way.node_refs[i];
                let is_graph_node = osm_to_internal.contains_key(&nref);
                let is_last = i == way.node_refs.len() - 1;

                if is_graph_node || is_last {
                    let from_osm = way.node_refs[segment_start];
                    let to_osm = nref;

                    if let (Some(&from_id), Some(&to_id)) =
                        (osm_to_internal.get(&from_osm), osm_to_internal.get(&to_osm))
                    {
                        // Calculate distance
                        let mut distance = 0.0;
                        let mut geometry = Vec::new();
                        for j in segment_start..i {
                            let a_osm = way.node_refs[j];
                            let b_osm = way.node_refs[j + 1];
                            if let (Some(&a_coord), Some(&b_coord)) =
                                (self.osm_nodes.get(&a_osm), self.osm_nodes.get(&b_osm))
                            {
                                distance += a_coord.distance_to(b_coord);
                                if j > segment_start {
                                    geometry.push(a_coord);
                                }
                            }
                        }

                        // Forward edge
                        edges.push(Edge {
                            from: NodeId(from_id),
                            to: NodeId(to_id),
                            distance_m: distance,
                            duration_s: 0.0, // Computed from profile at query time
                            way_id,
                            road_class,
                            oneway,
                            name: name.clone(),
                            geometry: geometry.clone(),
                        });

                        // Reverse edge (if not oneway)
                        if !oneway {
                            let mut rev_geom = geometry;
                            rev_geom.reverse();
                            edges.push(Edge {
                                from: NodeId(to_id),
                                to: NodeId(from_id),
                                distance_m: distance,
                                duration_s: 0.0,
                                way_id,
                                road_class,
                                oneway: false,
                                name: name.clone(),
                                geometry: rev_geom,
                            });
                        }
                    }

                    segment_start = i;
                }
            }
        }

        stats.edges_created = edges.len();
        let mut graph = Graph::build(nodes, edges);
        graph.declared_bounds = self.declared_bounds;

        // Add turn restrictions
        for restriction in &self.restrictions {
            if let Some(&via_internal) = osm_to_internal.get(&restriction.via_node) {
                let restriction_type = if restriction.restriction_type.starts_with("no_") {
                    itinera_graph::turn::RestrictionType::No
                } else if restriction.restriction_type.starts_with("only_") {
                    itinera_graph::turn::RestrictionType::Only
                } else {
                    continue;
                };

                graph.restrictions.push(TurnRestriction {
                    via_node: NodeId(via_internal),
                    from_way: restriction.from_way,
                    to_way: restriction.to_way,
                    restriction_type,
                });
            }
        }

        Ok((graph, stats))
    }
}

impl Default for OsmImporter {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for parsing OSM relation elements into turn restrictions.
struct OsmRelationBuilder {
    from_way: Option<i64>,
    to_way: Option<i64>,
    via_node: Option<i64>,
    relation_type: Option<String>,
    restriction: Option<String>,
}

impl OsmRelationBuilder {
    fn new() -> Self {
        Self {
            from_way: None,
            to_way: None,
            via_node: None,
            relation_type: None,
            restriction: None,
        }
    }

    fn build(self) -> Option<OsmRestriction> {
        // Only process turn restriction relations
        let rel_type = self.relation_type.as_deref()?;
        if rel_type != "restriction" {
            return None;
        }

        let restriction = self.restriction?;
        let from_way = self.from_way?;
        let to_way = self.to_way?;
        let via_node = self.via_node?;

        Some(OsmRestriction {
            from_way,
            to_way,
            via_node,
            restriction_type: restriction,
        })
    }
}

/// Parse id, lat, lon from a <node> element.
fn parse_node_attrs(line: &str) -> Option<(i64, f64, f64)> {
    let id = extract_attr(line, "id")?.parse::<i64>().ok()?;
    let lat = extract_attr(line, "lat")?.parse::<f64>().ok()?;
    let lon = extract_attr(line, "lon")?.parse::<f64>().ok()?;
    Some((id, lat, lon))
}

// <bounds minlat=".." minlon=".." maxlat=".." maxlon=".."/>
fn parse_bounds(line: &str) -> Option<BoundingBox> {
    let corner = |attr: &str| extract_attr(line, attr)?.parse::<f64>().ok();
    Some(BoundingBox::from_corners(
        corner("minlat")?,
        corner("minlon")?,
        corner("maxlat")?,
        corner("maxlon")?,
    ))
}

/// Parse ref from <nd ref="..."/>.
fn parse_nd_ref(line: &str) -> Option<i64> {
    extract_attr(line, "ref")?.parse::<i64>().ok()
}

/// Parse k,v from <tag k="..." v="..."/>.
fn parse_tag(line: &str) -> Option<(String, String)> {
    let k = extract_attr(line, "k")?;
    let v = extract_attr(line, "v")?;
    Some((k, v))
}

/// Extract an XML attribute value.
fn extract_attr(line: &str, attr: &str) -> Option<String> {
    let pattern = format!("{attr}=\"");
    let start = line.find(&pattern)? + pattern.len();
    let rest = &line[start..];
    let end = rest.find('"')?;
    Some(rest[..end].to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_OSM: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <node id="1" lat="48.8566" lon="2.3522"/>
  <node id="2" lat="48.8606" lon="2.3376"/>
  <node id="3" lat="48.8570" lon="2.3450"/>
  <node id="4" lat="48.8738" lon="2.2950"/>
  <way id="100">
    <nd ref="1"/>
    <nd ref="3"/>
    <nd ref="2"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Rue de Rivoli"/>
  </way>
  <way id="101">
    <nd ref="2"/>
    <nd ref="4"/>
    <tag k="highway" v="motorway"/>
    <tag k="name" v="Avenue des Champs-Élysées"/>
  </way>
</osm>"#;

    #[test]
    fn test_parse_and_build() {
        let mut importer = OsmImporter::new();
        importer.parse_xml(SAMPLE_OSM.as_bytes()).unwrap();

        let (graph, stats) = importer.build_graph().unwrap();

        assert_eq!(stats.nodes_parsed, 4);
        assert_eq!(stats.ways_parsed, 2);
        assert!(stats.nodes_in_graph >= 3);
        assert!(stats.edges_created >= 3); // 2 forward from way1 (bidirectional=4) + 1 forward from motorway
        assert!(graph.num_nodes() >= 3);
    }

    const BOUNDED_OSM: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <bounds minlat="48.8000" minlon="2.2000" maxlat="48.9000" maxlon="2.4000"/>
  <node id="1" lat="48.8566" lon="2.3522"/>
  <node id="2" lat="48.8606" lon="2.3376"/>
  <way id="100">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="highway" v="primary"/>
  </way>
</osm>"#;

    fn graph_from(xml: &str) -> Graph {
        let mut importer = OsmImporter::new();
        importer.parse_xml(xml.as_bytes()).unwrap();
        importer.build_graph().unwrap().0
    }

    #[test]
    fn the_bounds_element_becomes_the_graph_coverage() {
        let coverage = graph_from(BOUNDED_OSM).coverage().unwrap();

        assert_eq!(coverage.min_lat, 48.8);
        assert_eq!(coverage.max_lat, 48.9);
        assert_eq!(coverage.min_lon, 2.2);
        assert_eq!(coverage.max_lon, 2.4);
    }

    #[test]
    fn a_file_without_bounds_falls_back_to_the_nodes() {
        let graph = graph_from(SAMPLE_OSM);

        assert!(graph.declared_bounds.is_none());
        assert!(
            graph
                .coverage()
                .unwrap()
                .contains(Coord::new(48.8566, 2.3522))
        );
    }

    #[test]
    fn test_extract_attr() {
        let line = r#"<node id="12345" lat="48.8" lon="2.3"/>"#;
        assert_eq!(extract_attr(line, "id"), Some("12345".to_string()));
        assert_eq!(extract_attr(line, "lat"), Some("48.8".to_string()));
    }
}
