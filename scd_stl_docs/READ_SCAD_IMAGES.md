# Transmission CAD Geometry (Generated)

This directory contains PNG and STL geometry generated from the OpenSCAD
models provided in the assignment archive.

To regenerate these files, run in the project root:

    ./build_images.sh

SCAD source files are located in:

    transmission_scad/transmission_scad/

Generated files include:

- transmission.png / transmission.stl
- transmission_apart.png
- case.png / case.stl
- primary_shaft.png / primary_shaft.stl
- secondary_shaft.png / secondary_shaft.stl

These files are used for visualization only.
Collision checking in the motion planner uses the simplified capsule
model implemented in `src/collision.py`.

