use std::time::Instant;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "itinera", version, about = "Pure-Rust routing engine")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Import OSM data and build a routing graph
    Import {
        /// Path to OSM XML or PBF file
        #[arg(short, long)]
        input: String,
        /// Path to output graph file (binary format)
        #[arg(short, long, default_value = "graph.bin")]
        output: String,
    },
    /// Pre-build contraction hierarchy for fast queries
    Preprocess {
        /// Path to graph file
        #[arg(short, long, default_value = "graph.bin")]
        graph: String,
        /// Path to output CH file
        #[arg(short, long, default_value = "graph.ch")]
        output: String,
        /// Routing profile: car, bicycle, pedestrian, truck
        #[arg(short, long, default_value = "car")]
        profile: String,
    },
    /// Start the HTTP routing server
    Serve {
        /// Bind address
        #[arg(short, long, default_value = "0.0.0.0:5000")]
        bind: String,
        /// Path to graph file
        #[arg(short, long, default_value = "graph.bin")]
        graph: String,
        /// Path to CH file (optional, enables fast CH queries)
        #[arg(short, long)]
        ch: Option<String>,
        /// Routing profile: car, bicycle, pedestrian, truck
        #[arg(short, long, default_value = "car")]
        profile: String,
    },
    /// Compute a route between two points
    Route {
        /// Source coordinate (lat,lon)
        #[arg(long)]
        from: String,
        /// Target coordinate (lat,lon)
        #[arg(long)]
        to: String,
        /// Path to graph file
        #[arg(short, long, default_value = "graph.bin")]
        graph: String,
        /// Algorithm: astar, dijkstra
        #[arg(short, long, default_value = "astar")]
        algorithm: String,
        /// Routing profile: car, bicycle, pedestrian, truck
        #[arg(short, long, default_value = "car")]
        profile: String,
    },
    /// Compute an isochrone from a point
    Isochrone {
        /// Center coordinate (lat,lon)
        #[arg(long)]
        center: String,
        /// Maximum travel time in seconds
        #[arg(long)]
        max_seconds: f64,
        /// Path to graph file
        #[arg(short, long, default_value = "graph.bin")]
        graph: String,
        /// Routing profile: car, bicycle, pedestrian, truck
        #[arg(short, long, default_value = "car")]
        profile: String,
    },
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Import { input, output } => {
            let start = Instant::now();
            println!("Importing OSM data from: {input}");

            let (graph, stats) = if input.ends_with(".pbf") || input.ends_with(".osm.pbf") {
                itinera_osm::parse_pbf(std::path::Path::new(&input))?
            } else {
                let file = std::fs::File::open(&input)?;
                let reader = std::io::BufReader::new(file);
                let mut importer = itinera_osm::OsmImporter::new();
                importer.parse_xml(reader)?;
                importer.build_graph()?
            };

            println!("Import complete ({:.2}s):", start.elapsed().as_secs_f64());
            println!("  Nodes parsed: {}", stats.nodes_parsed);
            println!("  Ways parsed: {}", stats.ways_parsed);
            println!("  Graph nodes: {}", stats.nodes_in_graph);
            println!("  Graph edges: {}", stats.edges_created);

            let data = graph.to_bytes();
            std::fs::write(&output, &data)?;
            println!(
                "Graph saved to: {output} ({:.2} MB)",
                data.len() as f64 / 1_048_576.0
            );
        }
        Commands::Preprocess {
            graph: graph_path,
            output,
            profile: profile_name,
        } => {
            let start = Instant::now();
            let profile = resolve_profile(&profile_name)?;

            println!("Loading graph from: {graph_path}");
            let data = std::fs::read(&graph_path)?;
            let graph = itinera_graph::Graph::from_bytes(&data)?;
            println!(
                "Graph loaded: {} nodes, {} edges",
                graph.num_nodes(),
                graph.num_edges()
            );

            println!("Building contraction hierarchy...");
            let ch = itinera_core::ContractionHierarchy::build(&graph, &profile);
            println!(
                "CH built ({:.2}s): {} edges in augmented graph",
                start.elapsed().as_secs_f64(),
                ch.graph.num_edges()
            );

            let ch_data = bincode::serialize(&ch)?;
            std::fs::write(&output, &ch_data)?;
            println!(
                "CH saved to: {output} ({:.2} MB)",
                ch_data.len() as f64 / 1_048_576.0
            );
        }
        Commands::Serve {
            bind,
            graph: graph_path,
            ch: ch_path,
            profile: profile_name,
        } => {
            itinera_server::init_tracing();
            let profile = resolve_profile(&profile_name)?;

            println!("Loading graph from: {graph_path}");
            let data = std::fs::read(&graph_path)?;
            let g = itinera_graph::Graph::from_bytes(&data)?;
            println!(
                "Graph loaded: {} nodes, {} edges",
                g.num_nodes(),
                g.num_edges()
            );

            let mut state = itinera_server::AppState::new(g, profile.clone());

            if let Some(ch_file) = ch_path {
                println!("Loading CH from: {ch_file}");
                let ch_data = std::fs::read(&ch_file)?;
                let ch: itinera_core::ContractionHierarchy = bincode::deserialize(&ch_data)?;
                println!("CH loaded: {} edges", ch.graph.num_edges());
                state = state.with_ch(ch);
            }

            let app = itinera_server::router(state);
            println!("Starting server on {bind}");
            let listener = tokio::net::TcpListener::bind(&bind).await?;
            axum::serve(listener, app).await?;
        }
        Commands::Route {
            from,
            to,
            graph: graph_path,
            algorithm,
            profile: profile_name,
        } => {
            let profile = resolve_profile(&profile_name)?;
            let data = std::fs::read(&graph_path)?;
            let g = itinera_graph::Graph::from_bytes(&data)?;

            let src_coord = parse_coord(&from)?;
            let dst_coord = parse_coord(&to)?;

            let source = g.nearest_node(src_coord).ok_or("no node near source")?;
            let target = g.nearest_node(dst_coord).ok_or("no node near target")?;

            let start = Instant::now();
            let route = match algorithm.as_str() {
                "dijkstra" => itinera_core::dijkstra(&g, source, target, &profile)?,
                _ => itinera_core::astar(&g, source, target, &profile)?,
            };
            let elapsed = start.elapsed();

            println!("Route found ({:.3}ms):", elapsed.as_secs_f64() * 1000.0);
            println!("  Distance: {:.1} m", route.distance_m);
            println!("  Duration: {:.1} s", route.duration_s);
            println!("  Nodes: {}", route.node_ids.len());
            println!("  Steps: {}", route.steps.len());

            for step in &route.steps {
                println!(
                    "    {:?} on {} ({:.0}m, {:.0}s)",
                    step.maneuver,
                    step.name.as_deref().unwrap_or("unnamed"),
                    step.distance_m,
                    step.duration_s,
                );
            }
        }
        Commands::Isochrone {
            center,
            max_seconds,
            graph: graph_path,
            profile: profile_name,
        } => {
            let profile = resolve_profile(&profile_name)?;
            let data = std::fs::read(&graph_path)?;
            let g = itinera_graph::Graph::from_bytes(&data)?;

            let coord = parse_coord(&center)?;
            let source = g.nearest_node(coord).ok_or("no node near center")?;

            let start = Instant::now();
            let result = itinera_core::isochrone(
                &g,
                source,
                max_seconds,
                &profile,
                itinera_core::DEFAULT_CONCAVITY,
            );
            let elapsed = start.elapsed();

            println!(
                "Isochrone ({max_seconds}s budget, {:.3}ms):",
                elapsed.as_secs_f64() * 1000.0
            );
            println!("  Reachable nodes: {}", result.nodes.len());
            println!("  Boundary points: {}", result.boundary.len());

            let geojson = serde_json::json!({
                "type": "Feature",
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [result.boundary.iter().map(|c| [c.lon, c.lat]).collect::<Vec<_>>()]
                },
                "properties": {
                    "max_seconds": max_seconds,
                    "reachable_nodes": result.nodes.len()
                }
            });
            println!("{}", serde_json::to_string_pretty(&geojson)?);
        }
    }

    Ok(())
}

fn parse_coord(s: &str) -> Result<itinera_graph::Coord, String> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        return Err(format!("invalid coordinate: '{s}', expected 'lat,lon'"));
    }
    let lat = parts[0]
        .trim()
        .parse::<f64>()
        .map_err(|e| format!("invalid lat: {e}"))?;
    let lon = parts[1]
        .trim()
        .parse::<f64>()
        .map_err(|e| format!("invalid lon: {e}"))?;
    Ok(itinera_graph::Coord::new(lat, lon))
}

fn resolve_profile(name: &str) -> Result<itinera_graph::SpeedProfile, String> {
    itinera_graph::SpeedProfile::from_name(name)
        .ok_or_else(|| format!("unknown profile '{name}'; valid: car, bicycle, pedestrian, truck"))
}
