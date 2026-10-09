"""Loaders for the extracted data in ``results/data`` and shared geometry."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from transforms import quat_to_rotmat, transform_matrix, wrap_angle


@dataclass
class Scans:
    stamp_ns: np.ndarray     # (N,) int64
    bag_time_ns: np.ndarray  # (N,) int64
    ranges: np.ndarray       # (N, B) float32
    angles: np.ndarray       # (B,) rad, laser frame
    meta: dict

    @property
    def t(self) -> np.ndarray:
        """Header stamps in seconds (float64; ~0.2 us resolution at 1.5e9 s)."""
        return self.stamp_ns * 1e-9

    def valid_mask(self) -> np.ndarray:
        r = self.ranges
        return np.isfinite(r) & (r >= self.meta["range_min"]) & (r <= self.meta["range_max"])


@dataclass
class Extrinsic:
    """Static transform parent -> child (maps child coordinates into parent)."""
    parent: str
    child: str
    matrix: np.ndarray  # 4x4

    def se2(self) -> np.ndarray:
        """Planar projection (x, y, yaw); yaw from the child x-axis projected onto the xy-plane."""
        x_axis = self.matrix[:3, 0]
        return np.array([self.matrix[0, 3], self.matrix[1, 3], np.arctan2(x_axis[1], x_axis[0])])


def load_scans(data_dir: Path) -> Scans:
    with np.load(data_dir / "scans.npz") as z:
        arrays = {k: z[k] for k in z.files}
    meta = json.loads((data_dir / "scan_meta.json").read_text(encoding="utf-8"))
    return Scans(arrays["stamp_ns"], arrays["bag_time_ns"], arrays["ranges"],
                 arrays["angles_rad"], meta)


def load_poses(data_dir: Path, name: str) -> pd.DataFrame:
    """name: 'odom' or 'gt'. Adds column t (seconds)."""
    df = pd.read_csv(data_dir / f"{name}.csv")
    df["t"] = df["stamp_ns"].to_numpy() * 1e-9
    return df


def load_tf_static(data_dir: Path) -> list[dict]:
    return json.loads((data_dir / "tf_static.json").read_text(encoding="utf-8"))


def lookup_static(tf_static: list[dict], parent: str, child: str) -> Extrinsic:
    """Compose static transforms along the tree from parent down to child."""
    edges = {tr["child"]: tr for tr in tf_static}
    chain = []
    frame = child
    while frame != parent:
        if frame not in edges:
            raise KeyError(f"no static TF path {parent} -> {child}")
        chain.append(edges[frame])
        frame = edges[frame]["parent"]
    m = np.eye(4)
    for tr in reversed(chain):
        m = m @ transform_matrix(tr["translation_xyz"], tr["rotation_xyzw"])
    return Extrinsic(parent, child, m)


def scan_points_laser(ranges: np.ndarray, angles: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Valid beams of one scan as 3D points (M, 3) in the laser frame (z = 0)."""
    r = ranges[valid].astype(float)
    a = angles[valid]
    return np.column_stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)])


def points_to_frame(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def poses_to_world_2d(points_xy: np.ndarray, pose: np.ndarray) -> np.ndarray:
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return np.column_stack([pose[0] + c * points_xy[:, 0] - s * points_xy[:, 1],
                            pose[1] + s * points_xy[:, 0] + c * points_xy[:, 1]])


def gt_pose_for_scans(scans: Scans, gt: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Exact-timestamp association scan -> GT (no interpolation).

    Returns (gt_index per scan or -1, boolean mask of scans with GT).
    """
    lookup = {int(s): i for i, s in enumerate(gt["stamp_ns"].to_numpy())}
    idx = np.array([lookup.get(int(s), -1) for s in scans.stamp_ns])
    return idx, idx >= 0


def gt_se2(gt: pd.DataFrame) -> np.ndarray:
    """GT base_link poses projected to (x, y, yaw)."""
    r = quat_to_rotmat(gt[["qx", "qy", "qz", "qw"]].to_numpy())
    yaw = np.arctan2(r[:, 1, 0], r[:, 0, 0])
    return np.column_stack([gt["x"], gt["y"], wrap_angle(yaw)])


def gt_base_and_laser_poses(gt: pd.DataFrame, ext_se2: np.ndarray,
                            interpretation: str) -> tuple[np.ndarray, np.ndarray]:
    """Planar base_link and laser poses from /gt under a frame interpretation.

    interpretation 'base_link': /gt child is base_link (as labelled in the bag).
    interpretation 'laser':     /gt child is the laser frame (see check_data.py).
    """
    from transforms import se2_compose, se2_inverse
    raw = gt_se2(gt)
    ext = np.broadcast_to(ext_se2, raw.shape)
    if interpretation == "base_link":
        return raw, se2_compose(raw, ext)
    if interpretation == "laser":
        return se2_compose(raw, np.broadcast_to(se2_inverse(ext_se2), raw.shape)), raw
    raise ValueError(f"unknown GT interpretation {interpretation!r}")


def odom_se2(odom: pd.DataFrame) -> np.ndarray:
    return np.column_stack([odom["x"], odom["y"], wrap_angle(odom["yaw"].to_numpy())])
