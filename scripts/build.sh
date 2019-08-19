#!/usr/bin/env bash
set -e

export RUSTFLAGS="-D warnings"

cargo build --verbose --examples --all-features
