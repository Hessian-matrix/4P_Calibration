ARG BASE_IMAGE=10.21.12.53:5050/corp/rd/camera_corelib/ci-rust:1.98.1-bookworm-v1
FROM ${BASE_IMAGE}

ARG ZIG_VERSION=0.13.0
ARG ZIG_SHA256=d45312e61ebcc48032b77bc4cf7fd6915c11fa16e4aad116b66c9468211230ea
ARG CARGO_ZIGBUILD_VERSION=0.20.1

RUN rustup component add clippy \
    && rustup target add \
        aarch64-unknown-linux-gnu \
        x86_64-pc-windows-gnu \
        aarch64-apple-darwin \
    && python3 -c 'import urllib.request; urllib.request.urlretrieve("https://ziglang.org/download/'"${ZIG_VERSION}"'/zig-linux-x86_64-'"${ZIG_VERSION}"'.tar.xz", "/tmp/zig.tar.xz")' \
    && echo "${ZIG_SHA256}  /tmp/zig.tar.xz" | sha256sum -c - \
    && mkdir -p /opt/zig \
    && tar -xJf /tmp/zig.tar.xz --strip-components=1 -C /opt/zig \
    && rm /tmp/zig.tar.xz \
    && cargo install cargo-zigbuild --version "${CARGO_ZIGBUILD_VERSION}" --locked \
    && clippy-driver --version \
    && /opt/zig/zig version \
    && cargo zigbuild --version

ENV PATH="/opt/zig:${PATH}"
