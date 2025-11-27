# RBE 550 – HW6: Transmission (SM-465) Mainshaft Removal

This project implements a 6-DOF RRT-based motion planner to extract the
mainshaft from an SM-465 transmission housing. The planner uses a
capsule-based collision checker and produces the full set of figures and
animations required by the assignment.

---

## Setup & Installation

### 1. Clone the Repository
```bash
git clone git@github.com:meljahmi-personal/RBE550-assignment6.git
cd HW6_Transmission
```

### 2. Create and Activate a Virtual Environment
```bash
python3 -m venv hw6env
source hw6env/bin/activate    # On Linux/Mac
# or
hw6env\Scripts\activate     # On Windows
```

### 3. Install dependencies
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

## 4. Running the Assignment

```bash
./run_hw6.sh
```

# This script:

# Generates STL and PNG geometry using OpenSCAD

# Runs the 6-DOF RRT planner

# Produces:

# results/path_3d.png

# results/rrt_tree.png

# results/anim_simple.gif (clean animation)

# results/anim_mesh.gif (rich animation with STL models)

## Code Structure:

src/
 ├── collision.py          # Capsule-based collision model
 ├── rrt.py               # 6-DOF RRT planner
 └── main.py              # Pipeline, visualization, GIF generation
transmission_scad/
 └── images/              # STL files generated from assignment OpenSCAD
results/                  # Output figures and GIFs

# Collision Model

# Implements:

# Case interior as AABB

# Capsule model for mainshaft & countershaft

# Capsule-plane checks

# Open front face for exit

# Planner Details

# SE(3) state: position + roll-pitch-yaw


# Fixed translation/rotation step

# Discrete collision checks along edges

# Skips first edge sample to allow expansion from tight start

## Requirements:

numpy
matplotlib
trimesh (optional for STL GIF)
imageio


## Process flow
Start
│
│ Build scene (case, shafts)
│ Load start & goal SE(3) poses
│
▼
Initialize RRT with start node
│
▼
Repeat until goal reached or max iterations:
├─ Sample random pose (goal-biased)
├─ Find nearest node in tree (XYZ distance)
├─ Steer toward sample using fixed SE(3) step
├─ Edge collision check (skip t=0)
├─ If collision-free:
│ add node + parent link
│ check goal region
└─ Continue
│
▼
Extract final path
│
▼
Generate plots + GIFs
