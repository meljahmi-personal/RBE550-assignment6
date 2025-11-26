#!/usr/bin/env bash
set -e

# Always run from the directory that contains this script
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Location of the SCAD archive (from assignment)
ARCHIVE_PATH="${SCRIPT_DIR}/RBE550_assignment_transmission_SCAD_files.tar.xz"

# Parent directory where SCAD and images live
SCAD_PARENT_DIR="${SCRIPT_DIR}/transmission_scad"

# Directory where the SCAD files should live (nested, like your old setup)
SCAD_DIR="${SCAD_PARENT_DIR}/transmission_scad"

# Directory where we put all images and STLs
OUTPUT_DIR="${SCAD_PARENT_DIR}/images"

# README that we want to ship alongside generated PNG/STL
SCAD_README_SRC="${SCRIPT_DIR}/scd_stl_docs/README_SCAD_IMAGES.md"
SCAD_README_DST="${OUTPUT_DIR}/README.md"

mkdir -p "${SCAD_DIR}"
mkdir -p "${OUTPUT_DIR}"

echo "[build_images] Using SCAD directory: ${SCAD_DIR}"
echo "[build_images] Output directory:     ${OUTPUT_DIR}"

# If the core SCAD files are missing, extract them directly into SCAD_DIR
if [ ! -f "${SCAD_DIR}/transmission.scad" ]; then
    if [ -f "${ARCHIVE_PATH}" ]; then
        echo "[build_images] Core SCAD files not found in ${SCAD_DIR}; extracting archive:"
        echo "               ${ARCHIVE_PATH}"
        tar -xJf "${ARCHIVE_PATH}" -C "${SCAD_DIR}"
    else
        echo "[build_images] ERROR: SCAD archive not found at:"
        echo "  ${ARCHIVE_PATH}"
        exit 1
    fi
fi

# Sanity-check the main SCAD files from the archive
for f in \
    "${SCAD_DIR}/transmission.scad" \
    "${SCAD_DIR}/apart_transmission.scad" \
    "${SCAD_DIR}/case_end.scad" \
    "${SCAD_DIR}/primary_shaft.scad" \
    "${SCAD_DIR}/counter_shaft.scad"
do
    if [ ! -f "${f}" ]; then
        echo "[build_images] ERROR: Missing core SCAD file: ${f}"
        exit 1
    fi
done

echo "[build_images] Creating view_*.scad wrapper files..."

# Wrapper that shows full transmission
cat > "${SCAD_DIR}/view_transmission.scad" <<'EOF'
use <transmission.scad>;
transmission();
EOF

# Wrapper that shows apart transmission
cat > "${SCAD_DIR}/view_transmission_apart.scad" <<'EOF'
use <transmission.scad>;
transmission_apart();
EOF

# Wrapper that shows the case
cat > "${SCAD_DIR}/view_case.scad" <<'EOF'
use <transmission.scad>;
case();
EOF

# Wrapper that shows primary shaft assembly
cat > "${SCAD_DIR}/view_primary.scad" <<'EOF'
use <transmission.scad>;
primary_shaft_assembly();
EOF

# Wrapper that shows secondary shaft (countershaft) assembly
cat > "${SCAD_DIR}/view_secondary.scad" <<'EOF'
use <transmission.scad>;
secondary_shaft_assembly();
EOF

# SCAD view files we just created
VIEW_TRANSMISSION="${SCAD_DIR}/view_transmission.scad"
VIEW_TRANSMISSION_APART="${SCAD_DIR}/view_transmission_apart.scad"
VIEW_CASE="${SCAD_DIR}/view_case.scad"
VIEW_PRIMARY="${SCAD_DIR}/view_primary.scad"
VIEW_SECONDARY="${SCAD_DIR}/view_secondary.scad"

# Convenience function: render a PNG using xvfb + OpenSCAD
render_png () {
    local scad_file="$1"
    local png_file="$2"

    echo "[build_images] PNG: ${png_file} from ${scad_file}"
    xvfb-run -a -s "-screen 0 1600x1200x24" \
        openscad "${scad_file}" \
        -o "${png_file}" \
        --imgsize=1600,1200 \
        --viewall \
        --autocenter
}

# Convenience function: generate an STL (no xvfb needed)
render_stl () {
    local scad_file="$1"
    local stl_file="$2"

    echo "[build_images] STL: ${stl_file} from ${scad_file}"
    openscad "${scad_file}" -o "${stl_file}"
}

# ---- PNG renders ----
render_png "${VIEW_TRANSMISSION}"       "${OUTPUT_DIR}/transmission.png"
render_png "${VIEW_TRANSMISSION_APART}" "${OUTPUT_DIR}/transmission_apart.png"
render_png "${VIEW_CASE}"               "${OUTPUT_DIR}/case.png"
render_png "${VIEW_PRIMARY}"            "${OUTPUT_DIR}/primary_shaft.png"
render_png "${VIEW_SECONDARY}"          "${OUTPUT_DIR}/secondary_shaft.png"

# ---- STL exports ----
render_stl "${VIEW_TRANSMISSION}"       "${OUTPUT_DIR}/transmission.stl"
render_stl "${VIEW_CASE}"               "${OUTPUT_DIR}/case.stl"
render_stl "${VIEW_PRIMARY}"            "${OUTPUT_DIR}/primary_shaft.stl"
render_stl "${VIEW_SECONDARY}"          "${OUTPUT_DIR}/secondary_shaft.stl"

# ---- Copy SCAD/STL README into images/ ----
if [ -f "${SCAD_README_SRC}" ]; then
    echo "[build_images] Copying SCAD/STL README into images/ as README.md"
    cp "${SCAD_README_SRC}" "${SCAD_README_DST}"
else
    echo "[build_images] WARNING: README_SCAD_IMAGES.md not found at:"
    echo "  ${SCAD_README_SRC}"
fi

echo "[build_images] Done. Images, STLs, and README.md are in: ${OUTPUT_DIR}"

