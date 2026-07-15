#!/bin/sh
set -e

# bind-mounted /data is often owned by the host user; take ownership so the
# unprivileged itinera user can build and read the graph
if [ "$(id -u)" = "0" ]; then
    chown -R itinera:itinera /data
    run_as="setpriv --reuid itinera --regid itinera --clear-groups"
else
    run_as=""
fi

# If OSM file exists and graph hasn't been built yet, import it
if [ -f /data/region.osm.pbf ] && [ ! -f /data/graph.bin ]; then
    echo "Building routing graph from /data/region.osm.pbf..."
    $run_as itinera import --input /data/region.osm.pbf --output /data/graph.bin
    echo "Graph built successfully."
fi

# If no graph file exists at all, warn and exit gracefully
if [ ! -f /data/graph.bin ]; then
    echo "WARNING: No graph file at /data/graph.bin"
    echo "Place an OSM extract at /data/region.osm.pbf and restart, or run:"
    echo "  itinera import --input /path/to/extract.osm.pbf --output /data/graph.bin"
    echo "Sleeping to keep container alive for debugging..."
    exec sleep infinity
fi

exec $run_as itinera "$@"
