#!/usr/bin/env bash
set -e

# Always run from the HW6_Transmission root
cd "$(dirname "$0")"

RESULTS_DIR="results"

echo "[1] Generating CAD figures (PNGs + STLs)..."
./build_images.sh

echo "[2] Running motion planner..."
rm -rf "${RESULTS_DIR}"
python3 src/main.py --max_iters 30000 --seed 550 --results_dir "${RESULTS_DIR}"

echo "[2a] Copying CAD PNGs + STLs into results/ for report..."
mkdir -p "${RESULTS_DIR}/cad_images"
cp transmission_scad/images/*.png  "${RESULTS_DIR}/cad_images/" || true
cp transmission_scad/images/*.stl  "${RESULTS_DIR}/cad_images/" || true
# also copy README
cp transmission_scad/images/README.md "${RESULTS_DIR}/cad_images/" || true

echo "[3] Results:"
echo "   CAD images / STLs: transmission_scad/images/"
echo "   Planner outputs:   ${RESULTS_DIR}/"
echo "   CAD copies (for report): ${RESULTS_DIR}/cad_images/"

echo
echo "[3a] ${RESULTS_DIR}/ contents:"
ls -1 "${RESULTS_DIR}" || echo "   (no results directory?)"

