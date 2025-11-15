#!/usr/bin/env python3
"""
Very direct collision model for the transmission problem.

- The case interior is an axis-aligned box centered at (0,0,0).
- The mainshaft is represented as a capsule (line segment + radius).
- The countershaft is a fixed capsule, offset in Z.
- Poses are applied to the mainshaft only.

Units: millimeters and radians.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple
import math
import numpy as np


# --------- Geometry parameters (adjust if you measured the STLs) ------------
# Case outer size and wall thickness (from assignment brief)
CASE_OUTER_XYZ_MM = np.array([280.0, 210.0, 300.0], dtype=float)
CASE_WALL_THICKNESS_MM = 25.0
CASE_INNER_XYZ_MM = CASE_OUTER_XYZ_MM - 2.0 * CASE_WALL_THICKNESS_MM

# Mainshaft simplified dimensions (capsule along +X in local frame)
MAIN_SHAFT_LENGTH_MM = 384.0
MAIN_SHAFT_RADIUS_MM = 32.0

# Countershaft simplified dimensions and vertical offset
COUNTER_SHAFT_LENGTH_MM = 384.0
COUNTER_SHAFT_RADIUS_MM = 35.0
COUNTER_SHAFT_Z_OFFSET_MM = -60.0  # countershaft sits below mainshaft

# Exit goal (outside +X side, slightly below the inner floor)
GOAL_TABLE_Z_MM = -(CASE_INNER_XYZ_MM[2] / 2.0) - 40.0
GOAL_CLEARANCE_X_MM = CASE_OUTER_XYZ_MM[0] / 2.0 + 80.0


# ----------------------- Basic types and transforms --------------------------

@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def transform_point_world(pose: Pose, local_point_xyz: np.ndarray) -> np.ndarray:
    rotation_world_from_local = rotation_matrix_from_rpy(pose.roll, pose.pitch, pose.yaw)
    translation_xyz = np.array([pose.x, pose.y, pose.z], dtype=float)
    return translation_xyz + rotation_world_from_local @ np.asarray(local_point_xyz, dtype=float)


# ---------------------------- Geometry helpers ------------------------------

def axis_aligned_inner_box_bounds_mm() -> Tuple[np.ndarray, np.ndarray]:
    half_sizes = 0.5 * CASE_INNER_XYZ_MM
    return -half_sizes, half_sizes  # (min_xyz, max_xyz)


def segment_segment_distance_mm(a0: np.ndarray, a1: np.ndarray,
                                b0: np.ndarray, b1: np.ndarray) -> float:
    """Shortest distance between two segments in 3D (Ericson)."""
    u = a1 - a0
    v = b1 - b0
    w = a0 - b0
    a = float(np.dot(u, u)) + 1e-9
    b = float(np.dot(u, v))
    c = float(np.dot(v, v)) + 1e-9
    d = float(np.dot(u, w))
    e = float(np.dot(v, w))
    D = a * c - b * b

    sN, sD = 0.0, D
    tN, tD = 0.0, c

    if D < 1e-9:
        sN, sD, tN, tD = 0.0, 1.0, e, c
    else:
        sN = (b * e - c * d)
        tN = (a * e - b * d)
        if sN < 0.0:
            sN, tN, tD = 0.0, e, c
        elif sN > D:
            sN, tN, tD = D, e + b, c

    if tN < 0.0:
        tN = 0.0
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = D
        else:
            sN, sD = -d, a
    elif tN > tD:
        tN = tD
        if (-d + b) < 0.0:
            sN = 0.0
        elif (-d + b) > a:
            sN = D
        else:
            sN, sD = (-d + b), a

    sc = 0.0 if abs(sN) < 1e-9 else sN / sD
    tc = 0.0 if abs(tN) < 1e-9 else tN / tD
    closest_vector = w + sc * u - tc * v
    return float(np.linalg.norm(closest_vector))


# ----------------------------- Scene wrapper --------------------------------

class TransmissionScene:
    """
    Holds the case and the countershaft, and checks collision for the moving mainshaft.
    """

    def __init__(self) -> None:
        # Precompute inner box bounds
        self.inner_min_xyz, self.inner_max_xyz = axis_aligned_inner_box_bounds_mm()

        # Mainshaft capsule (local axis along X)
        half_main_length = 0.5 * MAIN_SHAFT_LENGTH_MM
        self.mainshaft_local_a = np.array([-half_main_length, 0.0, 0.0], dtype=float)
        self.mainshaft_local_b = np.array([+half_main_length, 0.0, 0.0], dtype=float)
        self.mainshaft_radius_mm = float(MAIN_SHAFT_RADIUS_MM)

        # Countershaft capsule (fixed in world, also along X)
        half_counter_length = 0.5 * COUNTER_SHAFT_LENGTH_MM
        self.counter_local_a = np.array([-half_counter_length, 0.0, 0.0], dtype=float)
        self.counter_local_b = np.array([+half_counter_length, 0.0, 0.0], dtype=float)
        self.counter_radius_mm = float(COUNTER_SHAFT_RADIUS_MM)
        self.counter_world_pose = Pose(0.0, 0.0, float(COUNTER_SHAFT_Z_OFFSET_MM), 0.0, 0.0, 0.0)
        

    def mainshaft_inside_case(self, mainshaft_pose: Pose) -> bool:
        """
        Inside the case we enforce:
          - X: only the negative (left) wall
          - Y, Z: both walls
        Once a sample point is at/after the right opening (x >= inner_max_x - r),
        we stop enforcing Y/Z for that sample so the shaft can exit and drop.
        """
        r = self.mainshaft_radius_mm
        a_world = transform_point_world(mainshaft_pose, self.mainshaft_local_a)
        b_world = transform_point_world(mainshaft_pose, self.mainshaft_local_b)

        for t in np.linspace(0.0, 1.0, 9):
            p = a_world * (1.0 - t) + b_world * t

            # Always enforce only the negative X wall
            if p[0] < self.inner_min_xyz[0] + r:
                return False

            # If this sample point is still inside the right wall, enforce Y/Z
            if p[0] <= self.inner_max_xyz[0] - r:
                if not (self.inner_min_xyz[1] + r <= p[1] <= self.inner_max_xyz[1] - r):
                    return False
                if not (self.inner_min_xyz[2] + r <= p[2] <= self.inner_max_xyz[2] - r):
                    return False

            # Else: sample is out the opening → no Y/Z checks for this point

        return True



    def capsules_overlap(self, pose_a: Pose, a_local_a: np.ndarray, a_local_b: np.ndarray, radius_a: float,
                         pose_b: Pose, b_local_a: np.ndarray, b_local_b: np.ndarray, radius_b: float) -> bool:
        a0 = transform_point_world(pose_a, a_local_a)
        a1 = transform_point_world(pose_a, a_local_b)
        b0 = transform_point_world(pose_b, b_local_a)
        b1 = transform_point_world(pose_b, b_local_b)
        distance_mm = segment_segment_distance_mm(a0, a1, b0, b1)
        return distance_mm < (radius_a + radius_b)

    def pose_is_in_collision(self, mainshaft_pose: Pose) -> bool:
        if not self.mainshaft_inside_case(mainshaft_pose):
            return True
        if self.capsules_overlap(
            mainshaft_pose, self.mainshaft_local_a, self.mainshaft_local_b, self.mainshaft_radius_mm,
            self.counter_world_pose, self.counter_local_a, self.counter_local_b, self.counter_radius_mm
        ):
            return True
        return False


    # Convenience getters used by main.py
    def start_pose(self) -> Pose:
        """
        Start slightly forward in +X so the back tip clears the left wall,
        centered in Y, and well above the countershaft in Z.
        """
        # Required: x >= inner_min_x + radius + half_length
        # With inner_min_x ≈ -115, radius=32, half_length=192 → x >= 109
        start = Pose(120.0, 0.0, 90.0, 0.0, 0.0, 0.0)
        assert not self.pose_is_in_collision(start), "Start pose still colliding; tweak start X/Z."
        
        
        # Debug: check back tip against min X wall
        back_tip_x = start.x - 0.5 * MAIN_SHAFT_LENGTH_MM
        required_min_x = self.inner_min_xyz[0] + self.mainshaft_radius_mm
        print(f"back_tip_x={back_tip_x:.1f}, required_min_x={required_min_x:.1f}")

        return start


    def goal_pose(self) -> Pose:
        """Place goal outside on +X and below the inner floor (like resting on a table)."""
        return Pose(float(GOAL_CLEARANCE_X_MM), 0.0, float(GOAL_TABLE_Z_MM), 0.0, 0.0, 0.0)

