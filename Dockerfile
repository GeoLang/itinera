FROM rust:bookworm AS builder

WORKDIR /app
COPY . .
RUN cargo build --release -p itinera-cli

FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -r -s /bin/false itinera && \
    mkdir -p /data && chown itinera:itinera /data

COPY --from=builder /app/target/release/itinera /usr/local/bin/itinera
COPY --chmod=755 docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

# no USER: the entrypoint starts as root, then drops to whoever owns /data
# (the host user for a bind mount, itinera for a named volume) via setpriv.
# A baked-in USER can't write a bind-mounted host directory.

ENV RUST_LOG=info,itinera=debug
ENV ITINERA_PORT=3000

EXPOSE 3000
VOLUME /data

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:3000/health || exit 1

ENTRYPOINT ["docker-entrypoint.sh"]
CMD ["serve", "--graph", "/data/graph.bin", "--bind", "0.0.0.0:3000"]
