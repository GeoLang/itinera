FROM rust:1.87-bookworm AS builder

WORKDIR /app
COPY . .
RUN cargo build --release -p itinera-cli

FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -r -s /bin/false itinera

COPY --from=builder /app/target/release/itinera /usr/local/bin/itinera

USER itinera

ENV RUST_LOG=info,itinera=debug
ENV ITINERA_PORT=3000

EXPOSE 3000
VOLUME /data

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:3000/health || exit 1

ENTRYPOINT ["itinera"]
CMD ["serve", "--data", "/data", "--bind", "0.0.0.0:3000"]
