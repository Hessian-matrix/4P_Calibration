ARG BASE_IMAGE=10.21.12.53:5050/corp/rd/camera_corelib/ci-rust:1.98.1-bookworm-v1
FROM ${BASE_IMAGE}

ARG ZIG_SHA256=d45312e61ebcc48032b77bc4cf7fd6915c11fa16e4aad116b66c9468211230ea
ARG CARGO_ZIGBUILD_VERSION=0.20.1

ENV RUSTUP_DIST_SERVER="https://rsproxy.cn"
ENV RUSTUP_UPDATE_ROOT="https://rsproxy.cn/rustup"
ENV CARGO_REGISTRIES_CRATES_IO_INDEX="sparse+https://rsproxy.cn/index/"


RUN apt-get update \
    && apt-get install --yes --no-install-recommends gcc-aarch64-linux-gnu \
    && rm -rf /var/lib/apt/lists/*
COPY ci/zig-linux-x86_64-0.13.0.tar.xz /tmp/zig.tar.xz

RUN rustup component add clippy \
    && rustup target add \
        aarch64-unknown-linux-gnu \
        x86_64-pc-windows-gnullvm \
        aarch64-apple-darwin

RUN echo "${ZIG_SHA256}  /tmp/zig.tar.xz" | sha256sum -c - \
    && mkdir -p /opt/zig \
    && tar -xJf /tmp/zig.tar.xz --strip-components=1 -C /opt/zig \
    && rm /tmp/zig.tar.xz

ENV PATH="/opt/zig:${PATH}"

RUN cargo install cargo-zigbuild --version "${CARGO_ZIGBUILD_VERSION}" --locked

RUN clippy-driver --version \
    && zig version \
    && cargo zigbuild --help >/dev/null
