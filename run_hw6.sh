#!/usr/bin/env bash
set -e

# Always run from this directory
cd "$(dirname "$0")"

echo "[1] Generating CAD images (OpenSCAD)..."
(
    cd transmission_scad
    ./build_images.sh
)

echo "[2] Running RRT motion planner..."
python3 src/main.py --max_iters 30000 --seed 550 --results_dir results

echo "[3] Results written to ./results:"
ls -1 results

