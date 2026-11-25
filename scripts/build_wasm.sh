#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

SYSROOT="$(rustc --print sysroot)"
if [ -d "$SYSROOT/lib/rustlib/wasm32-unknown-unknown" ]; then
    cargo build -p ch585-wasm --target wasm32-unknown-unknown --release
else
    WASM_LD="${CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_LINKER:-/opt/homebrew/opt/lld/bin/wasm-ld}"
    if [ ! -x "$WASM_LD" ]; then
        echo "Error: wasm-ld not found at $WASM_LD and wasm32-unknown-unknown sysroot not installed." >&2
        exit 1
    fi
    RUSTC_BOOTSTRAP=1 \
    CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_LINKER="$WASM_LD" \
    RUSTFLAGS="${RUSTFLAGS:-} -C panic=abort" \
    cargo build -p ch585-wasm --target wasm32-unknown-unknown --release -Z build-std=std,panic_abort
fi

mkdir -p "$ROOT_DIR/web"
cp "$ROOT_DIR/target/wasm32-unknown-unknown/release/ch585_wasm.wasm" "$ROOT_DIR/web/ch585_wasm.wasm"
ls -lh "$ROOT_DIR/web/ch585_wasm.wasm"
