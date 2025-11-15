# OpenSCAD CAD Extraction (Headless)

## Requirements
sudo apt install -y openscad xvfb

## Usage
cd ..
./build_images.sh /full/path/to/RBE550_assignment_transmission_SCAD_files.tar.xz
# Outputs -> images/transmission.png, transmission_apart.png, case.png, primary_shaft.png, secondary_shaft.png

Notes:
- Script creates wrapper .scad files that call:
  transmission(), transmission_apart(), case(),
  primary_shaft_assembly(), secondary_shaft_assembly()
- Rendering uses xvfb-run so no GUI/OpenGL is required.

