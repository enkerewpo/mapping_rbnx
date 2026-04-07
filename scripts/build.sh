#!/usr/bin/env bash
# build.sh — Build the mapping_rbnx package.
# Called by `rbnx build -p .`; stamps rbnx-build/.rbnx-built on success.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(dirname "$SCRIPT_DIR")}"
BUILD_DIR="$PKG_ROOT/rbnx-build"
PROTO_GEN="$PKG_ROOT/proto_gen"

# Locate robonix workspace root (expects sibling or configurable via env)
ROBONIX_ROOT="${ROBONIX_ROOT:-$(cd "$PKG_ROOT/../robonix" 2>/dev/null && pwd || echo "")}"
RUST_ROOT="$ROBONIX_ROOT/rust"

echo "=== mapping_rbnx build ==="

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
    rm -rf "$BUILD_DIR" "$PROTO_GEN"
fi

# ── 1. Ensure FAST-LIVO2 submodule is initialized ────────────────────────────
LIVO2_DIR="$PKG_ROOT/third_party/FAST-LIVO2"
if [ ! -f "$LIVO2_DIR/CMakeLists.txt" ]; then
    echo "[build] Initializing FAST-LIVO2 submodule..."
    cd "$PKG_ROOT"
    git submodule update --init --recursive third_party/FAST-LIVO2
fi

# ── 2. Install Python dependencies ──────────────────────────────────────────
echo "[build] Installing Python dependencies..."
pip install --quiet --no-cache-dir \
    grpcio>=1.60.0 \
    grpcio-tools>=1.60.0 \
    protobuf>=4.25.0 \
    numpy>=1.24.0 \
    pyyaml>=6.0 \
    2>/dev/null || echo "[build] pip install skipped (not in venv or already satisfied)"

# ── 3. Proto codegen (robonix-codegen → grpc_tools.protoc) ───────────────────
if [[ -d "$RUST_ROOT" ]]; then
    INTERFACES_LIB="$RUST_ROOT/crates/robonix-interfaces/lib"
    CONTRACTS_DIR="$RUST_ROOT/contracts"
    INTERFACES_DIR="$RUST_ROOT/crates/robonix-interfaces/robonix_proto"
    RUNTIME_DIR="$RUST_ROOT/proto"

    if [[ -x /usr/bin/cargo ]]; then
        CARGO_BIN=/usr/bin/cargo
    else
        CARGO_BIN="${CARGO:-cargo}"
    fi

    # Step 1: regenerate .proto from ROS IDL + contracts
    echo "[build] robonix-codegen --lang proto ..."
    "$CARGO_BIN" run -p robonix-codegen --manifest-path "$RUST_ROOT/Cargo.toml" -- \
        --lang proto \
        -I "$INTERFACES_LIB" \
        --contracts "$CONTRACTS_DIR" \
        -o "$INTERFACES_DIR"

    # Step 2: generate Python pb2 / pb2_grpc stubs
    if python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
        echo "[build] generating proto_gen stubs..."
        mkdir -p "$PROTO_GEN"
        python3 -m grpc_tools.protoc \
            -I "$RUNTIME_DIR" \
            -I "$INTERFACES_DIR" \
            --python_out="$PROTO_GEN" \
            --grpc_python_out="$PROTO_GEN" \
            "$RUNTIME_DIR"/*.proto \
            "$INTERFACES_DIR"/*.proto 2>/dev/null || true
        echo "[build] proto_gen: $(ls "$PROTO_GEN"/*.py 2>/dev/null | wc -l) files"
    else
        echo "[build] WARNING: grpcio-tools not found — skipping proto_gen"
    fi
else
    echo "[build] WARNING: robonix repo not found at $ROBONIX_ROOT — skipping codegen"
    echo "[build]   set ROBONIX_ROOT env var to point to the robonix repo"
fi

# ── 4. Build Docker images ───────────────────────────────────────────────────
echo "[build] Building Docker images..."
cd "$PKG_ROOT"
if [ -f /etc/nv_tegra_release ] 2>/dev/null; then
    echo "[build] Jetson platform — building with Jetson Dockerfile"
    docker build -f docker/Dockerfile.jetson -t mapping_rbnx:jetson . || true
elif command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    echo "[build] x86 GPU platform — building with GPU Dockerfile"
    docker build -f docker/Dockerfile -t mapping_rbnx:latest . || true
else
    echo "[build] CPU-only platform — building with base Dockerfile"
    docker build -f docker/Dockerfile -t mapping_rbnx:latest . || true
fi

# ── 5. Stamp ─────────────────────────────────────────────────────────────────
mkdir -p "$BUILD_DIR"
date -Iseconds > "$BUILD_DIR/.rbnx-built"
echo "=== mapping_rbnx build complete ==="
