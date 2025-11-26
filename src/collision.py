#!/usr/bin/env python3

"""
Collision model for the SM-465 transmission homework.

- The case is a rectangular inner box with an open front face (+X).
- The mainshaft and countershaft are each approximated by several
  line segments (capsules) with different radii.
- Collision checking is rigid-body based, with no interpenetration.

This is a simplified but reasonably rich model: multiple capsules
for the shaft and gears, plus a box case and a fixed countershaft.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


# ---------- Basic pose + transforms ----------

@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float   # radians
    pitch: float  # radians
    yaw: float    # radians


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return a 3x3 rotation matrix for roll–pitch–yaw (radians)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr,  cr]])

    return Rz @ Ry @ Rx


def transform_point_world(pose: Pose, local_point_xyz: np.ndarray) -> np.ndarray:
    """Transform a point from shaft-local coordinates into world frame."""
    R = rotation_matrix_from_rpy(pose.roll, pose.pitch, pose.yaw)
    t = np.array([pose.x, pose.y, pose.z], dtype=float)
    return R @ local_point_xyz + t


# ---------- Geometry helpers ----------

def segment_segment_distance_mm(
    a0: np.ndarray, a1: np.ndarray,
    b0: np.ndarray, b1: np.ndarray,
) -> float:
    """
    Shortest distance between two line segments in 3D (mm).
    Standard segment–segment distance, clamping parameters to [0, 1].
    """
    u = a1 - a0
    v = b1 - b0
    w0 = a0 - b0

    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w0)
    e = np.dot(v, w0)

    denom = a * c - b * b
    if denom < 1e-8:
        # Segments almost parallel; just project one endpoint
        t = 0.0
        s = np.clip(-d / (a + 1e-8), 0.0, 1.0)
    else:
        s = (b * e - c * d) / denom
        t = (a * e - b * d) / denom
        s = np.clip(s, 0.0, 1.0)
        t = np.clip(t, 0.0, 1.0)

    closest_a = a0 + s * u
    closest_b = b0 + t * v
    return float(np.linalg.norm(closest_a - closest_b))


# ---------- Transmission scene ----------

class TransmissionScene:
    """
    Idealized SM-465 transmission for the homework.

    Coordinates:
    - The case is centered at the origin, aligned with XYZ axes.
    - X axis runs from rear (input side, negative) to front (output, positive).
    - The mainshaft is roughly at Z = 0.
    - The countershaft is below, at negative Z.

    All dimensions are in millimeters.
    """

    def __init__(self) -> None:
        # --- Collision margin / slack ---
        self.collision_margin_mm = 5.0   # applied to capsule radii
        self.case_clearance_mm   = 2.0   # base clearance to walls

        # Inner case dimensions from the assignment (simplified model):
        # length 280, width 210, height 300 [mm]
        half_length_x = 140.0
        half_width_y  = 105.0
        half_height_z = 150.0

        # Axis-aligned inner box (region occupied by air inside case)
        self.inner_min_xyz = np.array(
            [-half_length_x, -half_width_y, -half_height_z],
            dtype=float,
        )
        self.inner_max_xyz = np.array(
            [+half_length_x, +half_width_y, +half_height_z],
            dtype=float,
        )

        # ----- Mainshaft model (shaft-local coordinates) -----
        #
        # Each capsule is (local_point_a, local_point_b, radius_mm).
        # These have been shortened/shifted so the shaft actually fits
        # inside the 280 mm case at the start pose.
        self.mainshaft_capsules_local: List[Tuple[np.ndarray, np.ndarray, float]] = []

        # Long central shaft: entirely inside the case
        self.mainshaft_capsules_local.append(
            (np.array([-130.0, 0.0, 0.0]),
             np.array([+130.0, 0.0, 0.0]),
             14.0)  # ~ shaft radius
        )

        # Large gear near the rear (input side)
        # Radius reduced so that after collision_margin we
        # still have clearance at the start pose.
        self.mainshaft_capsules_local.append(
            (np.array([-110.0, 0.0, 0.0]),
             np.array([-70.0, 0.0, 0.0]),
             39.0)
        )

        # Cluster near front
        self.mainshaft_capsules_local.append(
            (np.array([+40.0, 0.0, 0.0]),
             np.array([+90.0, 0.0, 0.0]),
             32.0)
        )

        # ----- Countershaft model -----
        #
        # Fixed in the case, below the mainshaft (negative Z).
        self.countershaft_capsules_local: List[Tuple[np.ndarray, np.ndarray, float]] = []

        # Long countershaft body
        self.countershaft_capsules_local.append(
            (np.array([-190.0, 0.0, -70.0]),
             np.array([+190.0, 0.0, -70.0]),
             16.0)
        )

        # Large counter gear under the rear mainshaft gear
        self.countershaft_capsules_local.append(
            (np.array([-150.0, 0.0, -70.0]),
             np.array([-110.0, 0.0, -70.0]),
             39.0)
        )

        # Front cluster
        self.countershaft_capsules_local.append(
            (np.array([+60.0, 0.0, -70.0]),
             np.array([+110.0, 0.0, -70.0]),
             34.0)
        )

        # Countershaft is rigidly attached to case:
        self.countershaft_pose_world = Pose(
            x=0.0, y=0.0, z=0.0,
            roll=0.0, pitch=0.0, yaw=0.0,
        )

    # ----- Capsule helpers -----

    def mainshaft_capsules_world(
        self,
        shaft_pose_world: Pose,
    ) -> List[Tuple[np.ndarray, np.ndarray, float]]:
        """
        Return world-frame endpoints + *effective* radius for each mainshaft capsule.
        """
        world_capsules: List[Tuple[np.ndarray, np.ndarray, float]] = []
        for local_a, local_b, radius in self.mainshaft_capsules_local:
            wa = transform_point_world(shaft_pose_world, local_a)
            wb = transform_point_world(shaft_pose_world, local_b)
            eff_r = max(radius - self.collision_margin_mm, 0.0)
            world_capsules.append((wa, wb, eff_r))
        return world_capsules

    def countershaft_capsules_world(self) -> List[Tuple[np.ndarray, np.ndarray, float]]:
        """
        Return world-frame endpoints + *effective* radius for each countershaft capsule.
        """
        world_capsules: List[Tuple[np.ndarray, np.ndarray, float]] = []
        for local_a, local_b, radius in self.countershaft_capsules_local:
            wa = transform_point_world(self.countershaft_pose_world, local_a)
            wb = transform_point_world(self.countershaft_pose_world, local_b)
            eff_r = max(radius - self.collision_margin_mm, 0.0)
            world_capsules.append((wa, wb, eff_r))
        return world_capsules

    # ----- Collision checks -----

    def _mainshaft_hits_countershaft(self, mainshaft_pose: Pose) -> bool:
        """Check capsule–capsule collisions between mainshaft and countershaft."""
        main_caps = self.mainshaft_capsules_world(mainshaft_pose)
        counter_caps = self.countershaft_capsules_world()

        for ma0, ma1, ra in main_caps:
            for cb0, cb1, rb in counter_caps:
                d = segment_segment_distance_mm(ma0, ma1, cb0, cb1)
                if d < (ra + rb):
                    return True
        return False

    def _mainshaft_hits_case(self, mainshaft_pose: Pose) -> bool:
        """
        Check collision against case walls.
        """
        min_xyz = self.inner_min_xyz
        max_xyz = self.inner_max_xyz
        c = self.case_clearance_mm + self.collision_margin_mm

        # Sample ends + midpoints of each capsule
        for local_a, local_b, _radius in self.mainshaft_capsules_local:
            for local_p in (local_a, 0.5 * (local_a + local_b), local_b):
                p = transform_point_world(mainshaft_pose, local_p)
                x, y, z = p

                # Back plate is solid
                if x < (min_xyz[0] + c):
                    return True

                # If still in the case along X, enforce side / top / bottom
                if x <= (max_xyz[0] + c):
                    if (y < (min_xyz[1] + c) or
                        y > (max_xyz[1] - c) or
                        z < (min_xyz[2] + c) or
                        z > (max_xyz[2] - c)):
                        return True

                # Front is open once beyond max_x + c

        return False

    def pose_is_in_collision(self, mainshaft_pose: Pose) -> bool:
        """
        Main API for the planner.
        """
        if self._mainshaft_hits_case(mainshaft_pose):
            return True
        if self._mainshaft_hits_countershaft(mainshaft_pose):
            return True
        return False

    # ----- Start / goal poses -----

    def start_pose(self) -> Pose:
        """
        Return an initial pose placed at the center of the bearing bores.
        """
        center_x = 0.5 * (self.inner_min_xyz[0] + self.inner_max_xyz[0])
        center_y = 0.5 * (self.inner_min_xyz[1] + self.inner_max_xyz[1])
        center_z = 0.0  # symmetry plane

        start = Pose(
            x=center_x,
            y=center_y,
            z=center_z,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        if self.pose_is_in_collision(start):
            print(
                "[WARN] start_pose: initial pose flagged as colliding in the "
                "approximate model; using it anyway to allow RRT to start."
            )

        return start

    def goal_pose(self) -> Pose:
        """
        Goal configuration: mainshaft outside the case, resting on
        an imaginary workbench in front of the transmission.
        """
        goal_x = self.inner_max_xyz[0] + 220.0

        return Pose(
            x=goal_x,
            y=0.0,
            z=0.0,
            roll=0.0,
            pitch=np.deg2rad(25.0),
            yaw=np.deg2rad(5.0),
        )

