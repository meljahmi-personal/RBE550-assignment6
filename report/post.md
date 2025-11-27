Finished RBE 550 – HW6, which required building a full 6-DOF RRT motion planner to extract the mainshaft from an SM-465 transmission housing.

Highlights:

Fully reproducible OpenSCAD pipeline (auto-generated PNG + STL geometry)

Capsule-based collision model for shafts, gears, and case interior

SE(3) RRT with goal bias, edge collision checks, and path extraction

Clean and mesh-based animations (anim_simple.gif, anim_mesh.gif)

Complete report + figures generated via a single command:

```bash
./run_hw6.sh
```


Everything is automated, reproducible, and packaged for submission.
