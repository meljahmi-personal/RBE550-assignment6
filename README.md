# RBE 550 – HW6: Transmission (SM-465) Mainshaft Removal

This project implements a 6-DOF RRT-based motion planner to extract the mainshaft from an SM-465 transmission housing.
The planner uses a capsule-based collision model and automatically produces all CAD figures, STL geometry, and required animations.

---

## 1. Setup & Installation

### Clone the Repository
```bash
git clone git@github.com:meljahmi-personal/RBE550-assignment6.git
cd RBE550-assignment6
```

### Create and Activate a Virtual Environment
```bash
python3 -m venv hw6env
source hw6env/bin/activate        # Linux/Mac
# or:
hw6env\Scripts\activate           # Windows
```

### Install Dependencies
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### Python Requirements
```
numpy
matplotlib
imageio
trimesh
shapely
```

### Development Environment
- Ubuntu 22.04 LTS  
- Python 3.10  
- OpenSCAD 2021.01  
- ImageMagick  
- xvfb-run (for headless OpenSCAD rendering)

---

## 2. Running the Assignment

Run the complete pipeline:

```bash
./run_hw6.sh
```

This script:

- Extracts the SCAD archive  
- Builds view_*.scad wrappers  
- Generates all PNGs and STLs  
- Runs the 6-DOF RRT planner  
- Produces all animations  
- Copies CAD outputs into results/cad_images/  

---

## 3. Generated Outputs

### Planner Figures (results/)
- path_3d.png — Final extracted path  
- rrt_tree.png — RRT exploration tree  

### Animations
- anim_simple.gif — Capsule-based animation  
- anim_mesh.gif — Full STL mesh animation  

### CAD Images & STL Files (results/cad_images/)
- transmission.png  
- transmission_apart.png  
- case.png  
- primary_shaft.png  
- secondary_shaft.png  

**STLs:**  
- transmission.stl  
- case.stl  
- primary_shaft.stl  
- secondary_shaft.stl  

---

## 4. Directory Structure

Directory tree after running `run_hw6.sh`:

```
├── build_images.sh
├── hw6env
│   ├── bin
│   │   ├── activate
│   │   ├── activate.csh
│   │   ├── activate.fish
│   │   ├── Activate.ps1
│   │   ├── f2py
│   │   ├── fonttools
│   │   ├── imageio_download_bin
│   │   ├── imageio_remove_bin
│   │   ├── pip
│   │   ├── pip3
│   │   ├── pip3.10
│   │   ├── pyftmerge
│   │   ├── pyftsubset
│   │   ├── python -> python3
│   │   ├── python3 -> /usr/bin/python3
│   │   ├── python3.10 -> python3
│   │   └── ttx
│   ├── include
│   ├── lib
│   │   └── python3.10
│   ├── lib64 -> lib
│   ├── pyvenv.cfg
│   └── share
│       └── man
├── RBE550_assignment_transmission_SCAD_files.tar.xz
├── README.md
├── report
│   ├── meljahmi_RBE550_HW6.docx
│   └── meljahmi_RBE550_HW6.pdf
├── requirements.txt
├── results
│   ├── anim_mesh_frames
│   │   ├── frame_0000.png
│   │   ├── frame_0001.png
│   │   ├── frame_0002.png
│   │   ├── frame_0003.png
│   │   ├── frame_0004.png
│   │   ├── frame_0005.png
│   │   ├── frame_0006.png
│   │   ├── frame_0007.png
│   │   ├── frame_0008.png
│   │   ├── frame_0009.png
│   │   ├── frame_0010.png
│   │   ├── frame_0011.png
│   │   ├── frame_0012.png
│   │   ├── frame_0013.png
│   │   ├── frame_0014.png
│   │   ├── frame_0015.png
│   │   ├── frame_0016.png
│   │   └── frame_0017.png
│   ├── anim_mesh.gif
│   ├── anim_simple_frames
│   │   ├── frame_0000.png
│   │   ├── frame_0001.png
│   │   ├── frame_0002.png
│   │   ├── frame_0003.png
│   │   ├── frame_0004.png
│   │   ├── frame_0005.png
│   │   ├── frame_0006.png
│   │   ├── frame_0007.png
│   │   ├── frame_0008.png
│   │   ├── frame_0009.png
│   │   ├── frame_0010.png
│   │   ├── frame_0011.png
│   │   ├── frame_0012.png
│   │   ├── frame_0013.png
│   │   ├── frame_0014.png
│   │   ├── frame_0015.png
│   │   ├── frame_0016.png
│   │   └── frame_0017.png
│   ├── anim_simple.gif
│   ├── cad_images
│   │   ├── case.png
│   │   ├── case.stl
│   │   ├── primary_shaft.png
│   │   ├── primary_shaft.stl
│   │   ├── secondary_shaft.png
│   │   ├── secondary_shaft.stl
│   │   ├── transmission_apart.png
│   │   ├── transmission.png
│   │   └── transmission.stl
│   ├── path_3d.png
│   └── rrt_tree.png
├── run_hw6.sh
├── scd_stl_docs
│   └── READ_SCAD_IMAGES.md
├── src
│   ├── collision.py
│   ├── main.py
│   ├── __pycache__
│   │   ├── collision.cpython-310.pyc
│   │   └── rrt.cpython-310.pyc
│   └── rrt.py
└── transmission_scad
    ├── images
    │   ├── case.png
    │   ├── case.stl
    │   ├── primary_shaft.png
    │   ├── primary_shaft.stl
    │   ├── secondary_shaft.png
    │   ├── secondary_shaft.stl
    │   ├── transmission_apart.png
    │   ├── transmission.png
    │   └── transmission.stl
    └── transmission_scad
        ├── apart_transmission.scad
        ├── case_end.scad
        ├── counter_shaft.scad
        ├── main_transmission.scad
        ├── primary_shaft.scad
        ├── transmission.scad
        ├── view_case.scad
        ├── view_primary.scad
        ├── view_secondary.scad
        ├── view_transmission_apart.scad
        └── view_transmission.scad


```

---

## 5. Collision Model (Summary)

Implements:

- Transmission case as an AABB (280×210×300 mm interior)  
- Mainshaft & countershaft approximated by capsules  
- Capsule–capsule collision checks  
- Capsule–plane checks against case walls  
- Open front face for extraction  
- Edge sampling inside RRT (skip t=0)  

---

## 6. Planner Details

- State space: SE(3) (XYZ + roll, pitch, yaw)  
- Fixed translation and rotation step size  
- Goal-biased sampling  
- Nearest neighbor via Euclidean XYZ  
- Edge discretization + collision checking  
- Path extraction and visualization  

---

## 7. High-Level Flow

```
Start
│
├── Build capsule model from SCAD geometry
├── Load start/goal poses
│
├── Initialize RRT
│
├── Loop:
│     ├── Sample pose
│     ├── Nearest node
│     ├── Steer toward sample
│     ├── Collision-check edge
│     └── Add node if free
│
├── Extract final path
└── Generate figures + animations
```

