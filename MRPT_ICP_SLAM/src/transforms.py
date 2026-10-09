"""Rotation and SE(2) helpers.

Conventions (ROS REP-103 / REP-105):
    * Quaternions are stored as (x, y, z, w), Hamilton convention.
    * Euler angles are intrinsic Z-Y-X (yaw, pitch, roll), i.e.
      R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    * A 2D pose (x, y, yaw) is the transform T_parent_child: it maps points
      given in the child frame into the parent frame.
"""

from __future__ import annotations

import numpy as np


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def quat_to_rotmat(q):
    """(x, y, z, w) -> 3x3 rotation matrix (supports shape (..., 4))."""
    x, y, z, w = np.moveaxis(quat_normalize(q), -1, 0)
    r = np.empty(np.shape(x) + (3, 3))
    r[..., 0, 0] = 1 - 2 * (y * y + z * z)
    r[..., 0, 1] = 2 * (x * y - z * w)
    r[..., 0, 2] = 2 * (x * z + y * w)
    r[..., 1, 0] = 2 * (x * y + z * w)
    r[..., 1, 1] = 1 - 2 * (x * x + z * z)
    r[..., 1, 2] = 2 * (y * z - x * w)
    r[..., 2, 0] = 2 * (x * z - y * w)
    r[..., 2, 1] = 2 * (y * z + x * w)
    r[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return r


def quat_to_rpy(q):
    """(x, y, z, w) -> (roll, pitch, yaw) in rad, Z-Y-X convention."""
    x, y, z, w = np.moveaxis(quat_normalize(q), -1, 0)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=-1)


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.stack(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        axis=-1,
    )


def transform_matrix(translation, quat):
    """4x4 homogeneous matrix from translation (3,) and quaternion (x,y,z,w)."""
    t = np.eye(4)
    t[:3, :3] = quat_to_rotmat(quat)
    t[:3, 3] = translation
    return t


# --------------------------------------------------------------------- SE(2)

def se2_matrix(pose):
    """(x, y, yaw) -> 3x3 homogeneous matrix (supports shape (..., 3))."""
    pose = np.asarray(pose, dtype=float)
    c, s = np.cos(pose[..., 2]), np.sin(pose[..., 2])
    m = np.zeros(pose.shape[:-1] + (3, 3))
    m[..., 0, 0], m[..., 0, 1], m[..., 0, 2] = c, -s, pose[..., 0]
    m[..., 1, 0], m[..., 1, 1], m[..., 1, 2] = s, c, pose[..., 1]
    m[..., 2, 2] = 1.0
    return m


def se2_compose(a, b):
    """a (+) b : pose b expressed in frame a, returned in a's parent frame."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    x = a[..., 0] + c * b[..., 0] - s * b[..., 1]
    y = a[..., 1] + s * b[..., 0] + c * b[..., 1]
    return np.stack([x, y, wrap_angle(a[..., 2] + b[..., 2])], axis=-1)


def se2_inverse(a):
    a = np.asarray(a, dtype=float)
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    x = -c * a[..., 0] - s * a[..., 1]
    y = s * a[..., 0] - c * a[..., 1]
    return np.stack([x, y, wrap_angle(-a[..., 2])], axis=-1)


def se2_relative(a, b):
    """Pose of b relative to a: a^-1 (+) b."""
    return se2_compose(se2_inverse(a), b)


def interpolate_se2(t_src, poses_src, t_query):
    """Linear interpolation of x, y and unwrapped yaw.

    Returns (poses, valid_mask); queries outside [t_src[0], t_src[-1]] are
    marked invalid (no extrapolation) and set to NaN.
    """
    t_src = np.asarray(t_src, dtype=float)
    t_query = np.asarray(t_query, dtype=float)
    poses_src = np.asarray(poses_src, dtype=float)
    yaw = np.unwrap(poses_src[:, 2])
    out = np.column_stack(
        [
            np.interp(t_query, t_src, poses_src[:, 0]),
            np.interp(t_query, t_src, poses_src[:, 1]),
            wrap_angle(np.interp(t_query, t_src, yaw)),
        ]
    )
    valid = (t_query >= t_src[0]) & (t_query <= t_src[-1])
    out[~valid] = np.nan
    return out, valid


def align_rigid_2d(src_xy, dst_xy):
    """Least-squares rigid alignment (Umeyama/Kabsch, no scale) in 2D.

    Finds (R, t) minimising sum ||dst - (R src + t)||^2.
    Returns the alignment as SE(2) pose (tx, ty, theta).
    """
    src = np.asarray(src_xy, dtype=float)
    dst = np.asarray(dst_xy, dtype=float)
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    d = np.eye(2)
    d[1, 1] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    t = mu_d - r @ mu_s
    return np.array([t[0], t[1], np.arctan2(r[1, 0], r[0, 0])])


def path_length(xy):
    xy = np.asarray(xy, dtype=float)
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def cumulative_path_length(xy):
    xy = np.asarray(xy, dtype=float)
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
