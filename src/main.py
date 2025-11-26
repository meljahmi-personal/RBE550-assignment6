#!/usr/bin/env python3
"""
RBE 550 – HW6 Transmission

Main script for the homework. It does three things:

  1. Builds the transmission scene (case + mainshaft + countershaft)
  2. Runs an RRT-based motion planner in SE(3) for the mainshaft
  3. Saves figures (path, RRT tree, animation) into a results directory

The idea is to remove the mainshaft from the case without collision.
"""

from __future__ import annotations

import argparse
import os
from typing import List

import numpy as np
import matplotlib.pyplot as plt

# Optional dependency for GIF animation. If not installed, the code
# still runs and just skips making the GIF.
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Optional dependency for GIF animation
try:
    import imageio.v2 as imageio
except Exception:
    imageio = None

HAVE_IMAGEIO: bool = imageio is not None

# Optional dependency for STL visualization
try:
    import trimesh
except Exception:
    trimesh = None

HAVE_TRIMESH: bool = trimesh is not None

from rrt import RRT, Pose as RrtPose
from collision import TransmissionScene, Pose as ScenePose, rotation_matrix_from_rpy





# ---------------------------------------------------------------------------
# Small utility helpers
# ---------------------------------------------------------------------------

def ensure_results_directory(results_dir: str) -> None:
    """Create the results directory if it does not exist."""
    os.makedirs(results_dir, exist_ok=True)


def save_3d_path_figure(
    path_pose_list: List[RrtPose],
    case_inner_xyz_mm: np.ndarray,
    output_path_png: str,
) -> None:
    """
    Save a simple 3D line plot of the mainshaft path (XYZ only).

    The case inner volume is drawn as a wireframe box, to give some
    context for how the path moves relative to the housing.
    """
    fig = plt.figure(figsize=(6.0, 5.0))
    axis = fig.add_subplot(111, projection="3d")

    if path_pose_list:
        xyz = np.array([[p.x, p.y, p.z] for p in path_pose_list], dtype=float)
        axis.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], linewidth=2.0)
        axis.scatter([xyz[0, 0]],  [xyz[0, 1]],  [xyz[0, 2]],  s=30)  # start
        axis.scatter([xyz[-1, 0]], [xyz[-1, 1]], [xyz[-1, 2]], s=30)  # goal

    # Draw inner case as a wireframe box
    hx, hy, hz = 0.5 * case_inner_xyz_mm
    corners = [
        np.array([sx * hx, sy * hy, sz * hz])
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]

    for i in range(8):
        for j in range(i + 1, 8):
            # Edge if they differ along exactly one axis
            if int(np.isclose(corners[i], corners[j]).sum()) == 2:
                xs, ys, zs = zip(corners[i], corners[j])
                axis.plot(xs, ys, zs, alpha=0.35)

    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.set_title("Mainshaft Path (position only)")
    axis.view_init(elev=24, azim=-60)
    plt.tight_layout()
    fig.savefig(output_path_png, dpi=160)
    plt.close(fig)


def save_rrt_tree_figure(rrt: RRT, output_path_png: str) -> None:
    """
    Save a 3D plot of the RRT exploration tree.

    Each edge is drawn between a node and its parent, projected into XYZ.
    """
    fig = plt.figure(figsize=(6.0, 5.0))
    axis = fig.add_subplot(111, projection="3d")

    for node_index, parent_index in enumerate(rrt.parent_index_list):
        if parent_index is None:
            continue

        parent_pose = rrt.node_list[parent_index]
        node_pose = rrt.node_list[node_index]
        pa = parent_pose.position_vector()
        pb = node_pose.position_vector()

        axis.plot(
            [pa[0], pb[0]],
            [pa[1], pb[1]],
            [pa[2], pb[2]],
            alpha=0.35,
            linewidth=0.7,
        )

    axis.set_xlabel("X (mm)")
    axis.set_ylabel("Y (mm)")
    axis.set_zlabel("Z (mm)")
    axis.set_title("RRT Tree (XYZ projection)")
    axis.view_init(elev=24, azim=-60)
    plt.tight_layout()
    fig.savefig(output_path_png, dpi=160)
    plt.close(fig)



def save_animation_frames_and_gif_simple(
    path_pose_list: List[RrtPose],
    case_inner_xyz_mm: np.ndarray,
    frames_dir: str,
    gif_path: str,
) -> None:
    """
    Simple animation: draw case as a box and the mainshaft as a line segment.
    This is the clean, robust version that is guaranteed to work.
    """
    os.makedirs(frames_dir, exist_ok=True)

    # Precompute case box edges
    hx, hy, hz = 0.5 * case_inner_xyz_mm
    corners = [
        np.array([sx * hx, sy * hy, sz * hz])
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]
    edges = [
        (i, j)
        for i in range(8)
        for j in range(i + 1, 8)
        if int(np.isclose(corners[i], corners[j]).sum()) == 2
    ]

    frames_paths: List[str] = []

    fig = plt.figure(figsize=(6.0, 5.0))
    axis = fig.add_subplot(111, projection="3d")

    for frame_index, pose in enumerate(path_pose_list):
        axis.cla()

        # Case wireframe
        for i, j in edges:
            xs, ys, zs = zip(corners[i], corners[j])
            axis.plot(xs, ys, zs, alpha=0.25)

        # Mainshaft axis (simple straight segment, length ~ 384 mm)
        axis.plot(
            [pose.x - 192.0, pose.x + 192.0],
            [pose.y, pose.y],
            [pose.z, pose.z],
            linewidth=3.0,
        )

        axis.set_xlim(-220, 420)
        axis.set_ylim(-220, 220)
        axis.set_zlim(-220, 220)
        axis.set_xlabel("X (mm)")
        axis.set_ylabel("Y (mm)")
        axis.set_zlabel("Z (mm)")
        axis.set_title(f"Simple frame {frame_index + 1}/{len(path_pose_list)}")
        axis.view_init(elev=24, azim=-60)
        plt.tight_layout()

        frame_path = os.path.join(frames_dir, f"frame_{frame_index:04d}.png")
        fig.savefig(frame_path, dpi=140)
        frames_paths.append(frame_path)

    plt.close(fig)

    if HAVE_IMAGEIO and len(frames_paths) > 1:
        images = [imageio.imread(path) for path in frames_paths]
        imageio.mimsave(gif_path, images, fps=10)


def save_animation_frames_and_gif_mesh(
    path_pose_list: List[RrtPose],
    case_inner_xyz_mm: np.ndarray,
    frames_dir: str,
    gif_path: str,
    stl_case_path: str,
    stl_primary_path: str,
    stl_secondary_path: str,
) -> None:
    """
    Rich animation: draws case + secondary shaft from STL meshes as static,
    and moves the primary shaft mesh along the RRT path.

    Falls back to doing nothing if trimesh is not available or STL load fails.
    """
    if not HAVE_TRIMESH:
        print("[WARN] trimesh not available; skipping STL animation.")
        return

    os.makedirs(frames_dir, exist_ok=True)

    try:
        case_mesh = trimesh.load(stl_case_path)
        primary_mesh = trimesh.load(stl_primary_path)
        secondary_mesh = trimesh.load(stl_secondary_path)
    except Exception as exc:
        print(f"[WARN] Failed to load STL meshes for animation: {exc}")
        return

    # Treat the primary shaft mesh as centered at its centroid.
    primary_center = primary_mesh.centroid

    # Precompute case box edges for extra context
    hx, hy, hz = 0.5 * case_inner_xyz_mm
    corners = [
        np.array([sx * hx, sy * hy, sz * hz])
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]
    edges = [
        (i, j)
        for i in range(8)
        for j in range(i + 1, 8)
        if int(np.isclose(corners[i], corners[j]).sum()) == 2
    ]

    frames_paths: List[str] = []

    fig = plt.figure(figsize=(6.0, 5.0))
    axis = fig.add_subplot(111, projection="3d")

    for frame_index, pose in enumerate(path_pose_list):
        axis.cla()

        # Case wireframe box
        for i, j in edges:
            xs, ys, zs = zip(corners[i], corners[j])
            axis.plot(xs, ys, zs, alpha=0.10)

        # Static case mesh
        axis.add_collection3d(
            Poly3DCollection(case_mesh.triangles, alpha=0.25, linewidths=0.1)
        )

        # Static secondary shaft mesh
        axis.add_collection3d(
            Poly3DCollection(secondary_mesh.triangles, alpha=0.4, linewidths=0.1)
        )

        # Moving primary shaft mesh
        R = rotation_matrix_from_rpy(pose.roll, pose.pitch, pose.yaw)
        t = np.array([pose.x, pose.y, pose.z], dtype=float)

        V = primary_mesh.vertices - primary_center
        V_tf = (R @ V.T).T + t  # (N,3)
        primary_triangles = V_tf[primary_mesh.faces]

        axis.add_collection3d(
            Poly3DCollection(primary_triangles, alpha=0.9, linewidths=0.1)
        )

        axis.set_xlim(-220, 420)
        axis.set_ylim(-220, 220)
        axis.set_zlim(-220, 220)
        axis.set_xlabel("X (mm)")
        axis.set_ylabel("Y (mm)")
        axis.set_zlabel("Z (mm)")
        axis.set_title(f"STL frame {frame_index + 1}/{len(path_pose_list)}")
        axis.view_init(elev=24, azim=-60)
        plt.tight_layout()

        frame_path = os.path.join(frames_dir, f"frame_{frame_index:04d}.png")
        fig.savefig(frame_path, dpi=140)
        frames_paths.append(frame_path)

    plt.close(fig)

    if HAVE_IMAGEIO and len(frames_paths) > 1:
        images = [imageio.imread(path) for path in frames_paths]
        imageio.mimsave(gif_path, images, fps=10)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--max_iters",
        type=int,
        default=20000,
        help="Maximum number of RRT iterations",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=550,
        help="Random seed",
    )
    parser.add_argument(
        "--results_dir",
        type=str,
        default="../results",
        help="Output directory for figures and animation",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Shorter run for quick test",
    )

    args = parser.parse_args()

    if args.demo:
        # For demo runs, cap the iteration budget so it finishes quickly.
        args.max_iters = min(args.max_iters, 4000)

    # Make sure results directory exists
    ensure_results_directory(args.results_dir)

    # Build the scene and get start/goal poses
    scene = TransmissionScene()
    start_scene_pose = scene.start_pose()
    goal_scene_pose = scene.goal_pose()

    # Collision adapter: planner Pose -> scene Pose
    def collision_adapter(planner_pose: RrtPose) -> bool:
        scene_pose = ScenePose(
            planner_pose.x,
            planner_pose.y,
            planner_pose.z,
            planner_pose.roll,
            planner_pose.pitch,
            planner_pose.yaw,
        )
        return scene.pose_is_in_collision(scene_pose)

    # RRT planner configuration
    rrt = RRT(
        collision_function=collision_adapter,
        start_pose=RrtPose(
            start_scene_pose.x,
            start_scene_pose.y,
            start_scene_pose.z,
            start_scene_pose.roll,
            start_scene_pose.pitch,
            start_scene_pose.yaw,
        ),
        goal_pose=RrtPose(
            goal_scene_pose.x,
            goal_scene_pose.y,
            goal_scene_pose.z,
            goal_scene_pose.roll,
            goal_scene_pose.pitch,
            goal_scene_pose.yaw,
        ),
        random_seed=args.seed,
        max_iterations=args.max_iters,
        goal_sample_probability=0.20,
        translation_step_mm=25.0,
        rotation_step_rad=np.deg2rad(8.0),
        edge_sample_count=12,
    )

    # Run the planner
    path_pose_list = rrt.solve()
    if not path_pose_list:
        print("[NOTE] No path found with current settings. "
              "Try increasing --max_iters or loosening tolerances.")
        return

    case_inner_xyz_mm = scene.inner_max_xyz - scene.inner_min_xyz

    # Save path and RRT tree figures
    path_png = os.path.join(args.results_dir, "path_3d.png")
    tree_png = os.path.join(args.results_dir, "rrt_tree.png")
    save_3d_path_figure(path_pose_list, case_inner_xyz_mm, path_png)
    save_rrt_tree_figure(rrt, tree_png)

    # Save animation frames + optional GIF
    frames_dir = os.path.join(args.results_dir, "anim_frames")
    gif_path = os.path.join(args.results_dir, "anim.gif")

    # STL paths for richer visualization
    stl_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),  # go up from src/
        "transmission_scad",
        "images",
    )

    # --- Simple GIF (clean, robust) ---
    simple_frames_dir = os.path.join(args.results_dir, "anim_simple_frames")
    simple_gif_path = os.path.join(args.results_dir, "anim_simple.gif")
    save_animation_frames_and_gif_simple(
        path_pose_list,
        case_inner_xyz_mm,
        simple_frames_dir,
        simple_gif_path,
    )

    # --- STL-based GIF (rich geometry) ---
    stl_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),  # from src/ up to repo root
        "transmission_scad",
        "images",
    )
    case_stl = os.path.join(stl_dir, "case.stl")
    primary_stl = os.path.join(stl_dir, "primary_shaft.stl")
    secondary_stl = os.path.join(stl_dir, "secondary_shaft.stl")

    mesh_frames_dir = os.path.join(args.results_dir, "anim_mesh_frames")
    mesh_gif_path = os.path.join(args.results_dir, "anim_mesh.gif")
    save_animation_frames_and_gif_mesh(
        path_pose_list,
        case_inner_xyz_mm,
        mesh_frames_dir,
        mesh_gif_path,
        stl_case_path=case_stl,
        stl_primary_path=primary_stl,
        stl_secondary_path=secondary_stl,
    )

    print("Done.")
    print(f"- Path figure:          {path_png}")
    print(f"- RRT figure:           {tree_png}")
    print(f"- Simple animation:     {simple_gif_path}")
    print(f"- STL animation (rich): {mesh_gif_path}")


if __name__ == "__main__":
    main()

