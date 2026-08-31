//! One benchmark per row of the README's performance targets, on grids built here
//! because the repo ships no road network to measure.

use std::hint::black_box;

use criterion::{BatchSize, Criterion, Throughput, criterion_group, criterion_main};
use itinera_core::{ContractionHierarchy, DEFAULT_CONCAVITY, isochrone};
use itinera_graph::{Coord, Edge, Graph, Node, NodeId, SpeedProfile};
use itinera_osm::OsmImporter;

/// Spacing between neighbouring intersections, about 111 m.
const CELL_DEGREES: f64 = 0.001;
const MOTORWAY_CLASS: u8 = 1;
const PRIMARY_CLASS: u8 = 3;
const TERTIARY_CLASS: u8 = 5;
const ARTERIAL_EVERY: u32 = 8;
const MOTORWAY_EVERY: u32 = 32;

/// Serialization, CSR build and isochrone all run on this one.
const LARGE_GRID_SIDE: u32 = 512;
/// Smaller, because the OSM XML for it is held in memory as a string.
const IMPORT_GRID_SIDE: u32 = 256;
/// Contraction is quadratic in node count, so this is as large as a 10 sample run allows.
const CONTRACTION_GRID_SIDE: u32 = 24;
const ISOCHRONE_BUDGET_SECONDS: f64 = 600.0;

#[derive(Clone, Copy)]
enum GridStyle {
    /// One road class and one street length, so every turn has an equal-cost detour and
    /// contraction adds no shortcuts.
    UniformStreets,
    /// Faster classes every few lines and a length varied per street, which makes
    /// contraction add shortcuts.
    ArterialsAndVariedLengths,
}

fn road_class(style: GridStyle, line: u32) -> u8 {
    match style {
        GridStyle::UniformStreets => TERTIARY_CLASS,
        GridStyle::ArterialsAndVariedLengths if line.is_multiple_of(MOTORWAY_EVERY) => {
            MOTORWAY_CLASS
        }
        GridStyle::ArterialsAndVariedLengths if line.is_multiple_of(ARTERIAL_EVERY) => {
            PRIMARY_CLASS
        }
        GridStyle::ArterialsAndVariedLengths => TERTIARY_CLASS,
    }
}

/// Deterministic length factor in 0.75 to 1.25, standing in for streets of unequal length.
fn length_factor(style: GridStyle, from: u32, to: u32) -> f64 {
    match style {
        GridStyle::UniformStreets => 1.0,
        GridStyle::ArterialsAndVariedLengths => {
            let mixed =
                ((u64::from(from) << 32) | u64::from(to)).wrapping_mul(0x9E37_79B9_7F4A_7C15);
            0.75 + f64::from((mixed >> 40) as u32) / f64::from(u32::MAX >> 8) * 0.5
        }
    }
}

/// Way id of a grid line, matching an OSM import where one street spans many edges.
fn way_id(side: u32, line: u32, horizontal: bool) -> i64 {
    let offset = if horizontal { 0 } else { side };
    i64::from(offset + line) + 1
}

/// Square grid of two-way streets, both directions present as separate edges.
fn grid_parts(side: u32, style: GridStyle) -> (Vec<Node>, Vec<Edge>) {
    let mut nodes = Vec::with_capacity((side * side) as usize);
    for row in 0..side {
        for col in 0..side {
            let id = row * side + col;
            nodes.push(Node {
                id: NodeId(id),
                coord: Coord::new(f64::from(row) * CELL_DEGREES, f64::from(col) * CELL_DEGREES),
                osm_id: i64::from(id) + 1,
                ch_level: 0,
            });
        }
    }

    let mut edges = Vec::with_capacity((4 * side * (side - 1)) as usize);
    for row in 0..side {
        for col in 0..side {
            let id = row * side + col;
            for (neighbour_row, neighbour_col) in [(row + 1, col), (row, col + 1)] {
                if neighbour_row >= side || neighbour_col >= side {
                    continue;
                }
                let neighbour = neighbour_row * side + neighbour_col;
                let horizontal = neighbour_row == row;
                let line = if horizontal { row } else { col };
                let distance_m = nodes[id as usize]
                    .coord
                    .distance_to(nodes[neighbour as usize].coord)
                    * length_factor(style, id, neighbour);
                for (from, to) in [(id, neighbour), (neighbour, id)] {
                    edges.push(Edge {
                        from: NodeId(from),
                        to: NodeId(to),
                        distance_m,
                        duration_s: 0.0,
                        way_id: way_id(side, line, horizontal),
                        road_class: road_class(style, line),
                        oneway: false,
                        name: None,
                        geometry: Vec::new(),
                    });
                }
            }
        }
    }
    (nodes, edges)
}

fn grid_graph(side: u32, style: GridStyle) -> Graph {
    let (nodes, edges) = grid_parts(side, style);
    Graph::build(nodes, edges)
}

/// The same grid as OSM XML, one way per line of the grid.
fn grid_osm_xml(side: u32) -> String {
    let mut xml =
        String::from("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<osm version=\"0.6\">\n");
    for row in 0..side {
        for col in 0..side {
            let id = row * side + col + 1;
            let lat = f64::from(row) * CELL_DEGREES;
            let lon = f64::from(col) * CELL_DEGREES;
            xml.push_str(&format!(
                "  <node id=\"{id}\" lat=\"{lat}\" lon=\"{lon}\"/>\n"
            ));
        }
    }
    for line in 0..side {
        for horizontal in [true, false] {
            xml.push_str(&format!(
                "  <way id=\"{}\">\n",
                way_id(side, line, horizontal)
            ));
            for step in 0..side {
                let (row, col) = if horizontal {
                    (line, step)
                } else {
                    (step, line)
                };
                xml.push_str(&format!("    <nd ref=\"{}\"/>\n", row * side + col + 1));
            }
            xml.push_str("    <tag k=\"highway\" v=\"tertiary\"/>\n");
            xml.push_str("  </way>\n");
        }
    }
    xml.push_str("</osm>\n");
    xml
}

fn import_osm_xml(xml: &str) -> Graph {
    let mut importer = OsmImporter::new();
    importer.parse_xml(xml.as_bytes()).expect("xml parses");
    let (graph, _stats) = importer.build_graph().expect("graph builds");
    graph
}

fn graph_build(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("graph_build");

    let (nodes, edges) = grid_parts(LARGE_GRID_SIDE, GridStyle::UniformStreets);
    let csr_id = format!("csr_{}_nodes_{}_edges", nodes.len(), edges.len());
    group.sample_size(20);
    group.bench_function(csr_id, |bencher| {
        bencher.iter_batched(
            || (nodes.clone(), edges.clone()),
            |(nodes, edges)| Graph::build(nodes, edges),
            BatchSize::PerIteration,
        );
    });
    drop((nodes, edges));

    let xml = grid_osm_xml(IMPORT_GRID_SIDE);
    let imported = import_osm_xml(&xml);
    let import_id = format!(
        "osm_xml_import_{}_nodes_{}_edges",
        imported.num_nodes(),
        imported.num_edges()
    );
    drop(imported);
    group.sample_size(10);
    group.throughput(Throughput::Bytes(xml.len() as u64));
    group.bench_function(import_id, |bencher| {
        bencher.iter(|| import_osm_xml(black_box(&xml)));
    });

    group.finish();
}

fn contraction_hierarchy_preprocessing(criterion: &mut Criterion) {
    let profile = SpeedProfile::car();
    let mut group = criterion.benchmark_group("ch_preprocessing");
    group.sample_size(10);

    for (label, style) in [
        ("uniform_streets", GridStyle::UniformStreets),
        ("arterials", GridStyle::ArterialsAndVariedLengths),
    ] {
        let graph = grid_graph(CONTRACTION_GRID_SIDE, style);
        let shortcuts = ContractionHierarchy::build(&graph, &profile)
            .graph
            .num_edges()
            - graph.num_edges();
        let id = format!(
            "{label}_{}_nodes_{}_edges_{shortcuts}_shortcuts",
            graph.num_nodes(),
            graph.num_edges()
        );
        group.bench_function(id, |bencher| {
            bencher.iter(|| ContractionHierarchy::build(black_box(&graph), &profile));
        });
    }

    group.finish();
}

fn contraction_hierarchy_query(criterion: &mut Criterion) {
    let profile = SpeedProfile::car();
    let graph = grid_graph(CONTRACTION_GRID_SIDE, GridStyle::UniformStreets);
    let hierarchy = ContractionHierarchy::build(&graph, &profile);
    let corner = NodeId(graph.num_nodes() as u32 - 1);
    let id = format!(
        "uniform_streets_{}_nodes_corner_to_corner",
        graph.num_nodes()
    );

    let mut group = criterion.benchmark_group("ch_query");
    group.bench_function(id, |bencher| {
        bencher.iter(|| {
            hierarchy
                .query(black_box(NodeId(0)), black_box(corner), &profile)
                .expect("corner to corner path exists")
        });
    });
    group.finish();
}

fn isochrone_from_centre(criterion: &mut Criterion) {
    let profile = SpeedProfile::car();
    let graph = grid_graph(LARGE_GRID_SIDE, GridStyle::UniformStreets);
    let centre = NodeId(LARGE_GRID_SIDE * LARGE_GRID_SIDE / 2 + LARGE_GRID_SIDE / 2);
    let reached = isochrone(
        &graph,
        centre,
        ISOCHRONE_BUDGET_SECONDS,
        &profile,
        DEFAULT_CONCAVITY,
    )
    .nodes
    .len();
    let id = format!(
        "600s_budget_{reached}_of_{}_nodes_reached",
        graph.num_nodes()
    );

    let mut group = criterion.benchmark_group("isochrone");
    group.bench_function(id, |bencher| {
        bencher.iter(|| {
            isochrone(
                black_box(&graph),
                centre,
                ISOCHRONE_BUDGET_SECONDS,
                &profile,
                DEFAULT_CONCAVITY,
            )
        });
    });
    group.finish();
}

fn graph_binary_format(criterion: &mut Criterion) {
    let graph = grid_graph(LARGE_GRID_SIDE, GridStyle::UniformStreets);
    let bytes = graph.to_bytes();
    let size = format!(
        "{}_nodes_{}_edges_{}_MiB",
        graph.num_nodes(),
        graph.num_edges(),
        bytes.len() / (1024 * 1024)
    );

    let mut group = criterion.benchmark_group("graph_binary");
    group.sample_size(20);
    group.throughput(Throughput::Bytes(bytes.len() as u64));
    group.bench_function(format!("to_bytes_{size}"), |bencher| {
        bencher.iter(|| black_box(&graph).to_bytes());
    });
    group.bench_function(format!("from_bytes_{size}"), |bencher| {
        bencher.iter(|| Graph::from_bytes(black_box(&bytes)).expect("graph deserializes"));
    });
    group.finish();
}

criterion_group!(
    benches,
    graph_build,
    contraction_hierarchy_preprocessing,
    contraction_hierarchy_query,
    isochrone_from_centre,
    graph_binary_format,
);
criterion_main!(benches);
