#!/usr/bin/env bash
set -e

# always run from this directory
cd "$(dirname "$0")"

# ensure virtual env is activated if you use one
# (optional) source hw6env/bin/activate

echo "[1] Generating CAD figures..."
(cd transmission_scad && ./build_images.sh)

echo "[2] Running motion planner..."
python3 src/main.py --max_iters 30000 --seed 550

echo "[3] Results available in results/:"
ls -1 results | sed 's/^/   /'

