#!/usr/bin/env python3
"""
Goal-biased RRT in 6-DOF (x, y, z, roll, pitch, yaw).
Simple nearest-neighbor (by XYZ only), fixed step, and discrete edge collision checks.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, List, Optional
import math
import numpy as np


@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def as_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.roll, self.pitch, self.yaw], dtype=float)

    def position_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    @staticmethod
    def from_vector(vector: np.ndarray) -> "Pose":
        vector = np.asarray(vector, dtype=float).flatten()
        return Pose(float(vector[0]), float(vector[1]), float(vector[2]),
                    float(vector[3]), float(vector[4]), float(vector[5]))


def wrap_angle_radians(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RRT:
    def __init__(
        self,
        collision_function: Callable[[Pose], bool],
        start_pose: Pose,
        goal_pose: Pose,
        random_seed: int = 550,
        max_iterations: int = 20000,
        goal_sample_probability: float = 0.10,
        translation_step_mm: float = 12.0,
        rotation_step_rad: float = math.radians(4.0),
        edge_sample_count: int = 8,
        sampler_min_vector: Optional[np.ndarray] = None,
        sampler_max_vector: Optional[np.ndarray] = None,
    ):
        self.collision_function = collision_function
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.random_generator = np.random.default_rng(random_seed)
        self.max_iterations = int(max_iterations)
        self.goal_sample_probability = float(goal_sample_probability)
        self.translation_step_mm = float(translation_step_mm)
        self.rotation_step_rad = float(rotation_step_rad)
        self.edge_sample_count = int(edge_sample_count)

        # Default sampling box (safe, a bit larger than the case)
        self.sampler_min_vector = (
            np.array([-220.0, -220.0, -220.0, -math.pi, -math.pi/2.0, -math.pi])
            if sampler_min_vector is None else np.array(sampler_min_vector, dtype=float)
        )
        self.sampler_max_vector = (
            np.array([ 420.0,  220.0,  220.0,  math.pi,  math.pi/2.0,  math.pi])
            if sampler_max_vector is None else np.array(sampler_max_vector, dtype=float)
        )

        self.node_list: List[Pose] = [start_pose]
        self.parent_index_list: List[Optional[int]] = [None]
        self.node_xyz_list: List[np.ndarray] = [start_pose.position_vector()]

    # ---- RRT helpers -------------------------------------------------------

    def sample_configuration(self) -> Pose:
        if self.random_generator.random() < self.goal_sample_probability:
            return self.goal_pose
        random_vector = self.random_generator.uniform(self.sampler_min_vector, self.sampler_max_vector)
        return Pose.from_vector(random_vector)

    def nearest_node_index(self, query_pose: Pose) -> int:
        query_xyz = query_pose.position_vector()
        distances_sq = [float(np.sum((xyz - query_xyz) ** 2)) for xyz in self.node_xyz_list]
        return int(np.argmin(distances_sq))

    def steer_towards(self, from_pose: Pose, to_pose: Pose) -> Pose:
        """Take one step from from_pose toward to_pose."""
        from_xyz = from_pose.position_vector()
        to_xyz = to_pose.position_vector()

        direction_xyz = to_xyz - from_xyz
        distance = float(np.linalg.norm(direction_xyz)) + 1e-9
        if distance > self.translation_step_mm:
            direction_xyz = direction_xyz * (self.translation_step_mm / distance)

        # Limit rotation deltas per step (independently in RPY)
        delta_roll  = wrap_angle_radians(to_pose.roll  - from_pose.roll)
        delta_pitch = wrap_angle_radians(to_pose.pitch - from_pose.pitch)
        delta_yaw   = wrap_angle_radians(to_pose.yaw   - from_pose.yaw)

        delta_roll  = np.clip(delta_roll,  -self.rotation_step_rad, self.rotation_step_rad)
        delta_pitch = np.clip(delta_pitch, -self.rotation_step_rad, self.rotation_step_rad)
        delta_yaw   = np.clip(delta_yaw,   -self.rotation_step_rad, self.rotation_step_rad)

        stepped_vector = from_pose.as_vector().copy()
        stepped_vector[:3] += direction_xyz
        stepped_vector[3:] += np.array([delta_roll, delta_pitch, delta_yaw], dtype=float)
        return Pose.from_vector(stepped_vector)

    def edge_is_collision_free(self, pose_a: Pose, pose_b: Pose) -> bool:
        """Discretize the edge and test collisions."""
        for t in np.linspace(0.0, 1.0, self.edge_sample_count):
            vector_t = (1.0 - t) * pose_a.as_vector() + t * pose_b.as_vector()
            if self.collision_function(Pose.from_vector(vector_t)):
                return False
        return True

    def reached_goal_region(self, pose: Pose, xyz_tolerance_mm: float = 20.0) -> bool:
        return (abs(pose.x - self.goal_pose.x) < xyz_tolerance_mm and
                abs(pose.y - self.goal_pose.y) < xyz_tolerance_mm and
                abs(pose.z - self.goal_pose.z) < xyz_tolerance_mm)

    def extract_path(self, goal_index: int) -> List[Pose]:
        path: List[Pose] = []
        node_index: Optional[int] = goal_index
        while node_index is not None:
            path.append(self.node_list[node_index])
            node_index = self.parent_index_list[node_index]
        path.reverse()
        return path

    # ---- Main solve --------------------------------------------------------

    def solve(self) -> List[Pose]:
        for _ in range(self.max_iterations):
            sample_pose = self.sample_configuration()
            nearest_index = self.nearest_node_index(sample_pose)
            new_pose = self.steer_towards(self.node_list[nearest_index], sample_pose)

            if not self.edge_is_collision_free(self.node_list[nearest_index], new_pose):
                continue

            self.node_list.append(new_pose)
            self.parent_index_list.append(nearest_index)
            self.node_xyz_list.append(new_pose.position_vector())

            if self.reached_goal_region(new_pose):
                return self.extract_path(len(self.node_list) - 1)

        return []  # failed to find a path in the given budget

