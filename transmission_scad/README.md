# RBE-550 HW6 – Transmission (Reproducible Build)

## Dependencies (Ubuntu 22.04)
sudo apt update
sudo apt install -y openscad xvfb python3 python3-pip
pip install numpy matplotlib imageio

## Build CAD figures
chmod +x build_images.sh
./build_images.sh             # assumes the SCAD archive is here
# Outputs -> images/*.png

## Run planner
cd src
python3 main.py --max_iters 20000 --seed 550
# Outputs -> ../results/path_3d.png, rrt_tree.png, anim.gif

