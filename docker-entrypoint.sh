#!/bin/sh
set -e

# Drop privileges to whoever owns /data so the graph can be written either way:
# a bind mount arrives owned by the host user (chowning it would clobber the
# host's own files), a named volume comes up root-owned and is handed to itinera.
if [ "$(id -u)" = "0" ]; then
    data_uid=$(stat -c %u /data)
    data_gid=$(stat -c %g /data)
    if [ "$data_uid" = "0" ]; then
        chown -R itinera:itinera /data
        data_uid=$(id -u itinera)
        data_gid=$(id -g itinera)
    fi
    run_as="setpriv --reuid $data_uid --regid $data_gid --clear-groups"
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
