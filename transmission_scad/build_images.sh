#!/usr/bin/env bash
set -euo pipefail

# --- paths ---
ARCHIVE="${1:-RBE550_assignment_transmission_SCAD_files.tar.xz}"
ROOT_DIR="$(pwd)"
SCAD_DIR="${ROOT_DIR}/transmission_scad"
IMG_DIR="${ROOT_DIR}/images"

# --- sanity checks (do installs yourself; see README.md) ---
command -v openscad >/dev/null || { echo "ERROR: openscad not found"; exit 1; }
command -v xvfb-run >/dev/null || { echo "ERROR: xvfb-run not found"; exit 1; }

# --- unpack ---
mkdir -p "$SCAD_DIR" "$IMG_DIR"
tar -xf "$ARCHIVE" -C "$SCAD_DIR"

# --- wrappers (call actual modules) ---
cat > "$SCAD_DIR/view_transmission.scad" << 'EOF'
include <transmission.scad>
transmission();
EOF

cat > "$SCAD_DIR/view_transmission_apart.scad" << 'EOF'
include <transmission.scad>
transmission_apart();
EOF

cat > "$SCAD_DIR/view_case.scad" << 'EOF'
include <transmission.scad>
case();
EOF

cat > "$SCAD_DIR/view_primary.scad" << 'EOF'
include <transmission.scad>
primary_shaft_assembly();
EOF

cat > "$SCAD_DIR/view_secondary.scad" << 'EOF'
include <transmission.scad>
secondary_shaft_assembly();
EOF

# --- helper: render one PNG headlessly (no GUI/OpenGL) ---
render_png () {
  local in_scad="$1"
  local out_png="$2"
  xvfb-run -a --server-args="-screen 0 1600x1200x24" \
    openscad "$in_scad" -o "$out_png" --imgsize=1600,1200 --viewall --autocenter
}

# --- render all PNGs ---
cd "$SCAD_DIR"
render_png view_transmission.scad       transmission.png
render_png view_transmission_apart.scad transmission_apart.png
render_png view_case.scad               case.png
render_png view_primary.scad            primary_shaft.png
render_png view_secondary.scad          secondary_shaft.png
cd "$ROOT_DIR"

# --- collect outputs ---
cp -f "$SCAD_DIR"/*.png "$IMG_DIR"/
echo "Done. Images in: $IMG_DIR"

