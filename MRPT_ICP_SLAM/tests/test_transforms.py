"""Unit tests for src/transforms.py and src/dataset.py geometry helpers.

Run:  python -m unittest discover -s tests -v
"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dataset import Extrinsic, lookup_static, gt_base_and_laser_poses  # noqa: E402
from transforms import (  # noqa: E402
    align_rigid_2d, interpolate_se2, quat_to_rotmat, quat_to_rpy, rpy_to_quat,
    se2_compose, se2_inverse, se2_matrix, se2_relative, wrap_angle,
)


class TestQuaternions(unittest.TestCase):
    def test_yaw_90deg(self):
        q = np.array([0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])  # +90 deg about z
        r = quat_to_rotmat(q)
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(quat_to_rpy(q), [0, 0, np.pi / 2], atol=1e-12)

    def test_rpy_roundtrip(self):
        rng = np.random.default_rng(0)
        rpy = rng.uniform([-1, -1.2, -3], [1, 1.2, 3], size=(200, 3))
        q = rpy_to_quat(rpy[:, 0], rpy[:, 1], rpy[:, 2])
        np.testing.assert_allclose(quat_to_rpy(q), rpy, atol=1e-10)

    def test_rotation_orthonormal(self):
        q = np.array([0.0079, 0.0183, 0.0008, 0.9998])  # laser mount from the bag
        r = quat_to_rotmat(q)
        np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(r), 1.0, places=12)

    def test_matches_zyx_composition(self):
        roll, pitch, yaw = 0.1, -0.2, 0.3
        rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        np.testing.assert_allclose(quat_to_rotmat(rpy_to_quat(roll, pitch, yaw)), rz @ ry @ rx, atol=1e-12)


class TestSE2(unittest.TestCase):
    def test_compose_matches_matrices(self):
        a, b = np.array([1.0, 2.0, 0.7]), np.array([-0.5, 0.3, -2.0])
        m = se2_matrix(a) @ se2_matrix(b)
        c = se2_compose(a, b)
        np.testing.assert_allclose(se2_matrix(c), m, atol=1e-12)

    def test_inverse_and_relative(self):
        a = np.array([3.0, -1.0, 2.5])
        np.testing.assert_allclose(se2_compose(a, se2_inverse(a)), [0, 0, 0], atol=1e-12)
        b = np.array([4.0, 0.0, -3.0])
        np.testing.assert_allclose(se2_compose(a, se2_relative(a, b)), b, atol=1e-12)

    def test_wrap(self):
        np.testing.assert_allclose(wrap_angle([np.pi + 0.1, -np.pi - 0.1]), [-np.pi + 0.1, np.pi - 0.1])

    def test_interpolate_across_pi(self):
        t = np.array([0.0, 1.0])
        poses = np.array([[0, 0, np.pi - 0.1], [2, 0, -np.pi + 0.1]])
        out, valid = interpolate_se2(t, poses, np.array([0.5, 2.0]))
        self.assertTrue(valid[0])
        self.assertFalse(valid[1])
        self.assertAlmostEqual(abs(out[0, 2]), np.pi, places=9)  # shortest way, not through 0
        self.assertAlmostEqual(out[0, 0], 1.0)

    def test_align_rigid_recovers_transform(self):
        rng = np.random.default_rng(1)
        src = rng.uniform(-10, 10, size=(100, 2))
        true = np.array([2.0, -3.0, 0.8])
        dst = se2_compose(np.broadcast_to(true, (100, 3)), np.column_stack([src, np.zeros(100)]))[:, :2]
        np.testing.assert_allclose(align_rigid_2d(src, dst), true, atol=1e-9)


class TestStaticTF(unittest.TestCase):
    def test_chain_lookup(self):
        tf = [
            {"parent": "base", "child": "a", "translation_xyz": [1, 0, 0],
             "rotation_xyzw": [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)]},
            {"parent": "a", "child": "b", "translation_xyz": [1, 0, 0], "rotation_xyzw": [0, 0, 0, 1]},
        ]
        ext = lookup_static(tf, "base", "b")
        np.testing.assert_allclose(ext.matrix[:3, 3], [1, 1, 0], atol=1e-12)
        np.testing.assert_allclose(ext.se2(), [1, 1, np.pi / 2], atol=1e-12)

    def test_gt_interpretations_are_consistent(self):
        import pandas as pd
        ext = np.array([0.14, -0.10, 0.002])
        gt = pd.DataFrame({"x": [5.0], "y": [1.0], "z": [0.0], "qx": [0.0], "qy": [0.0],
                           "qz": [np.sin(0.6)], "qw": [np.cos(0.6)]})
        base_l, laser_l = gt_base_and_laser_poses(gt, ext, "laser")
        base_b, laser_b = gt_base_and_laser_poses(gt, ext, "base_link")
        np.testing.assert_allclose(se2_compose(base_l, ext), laser_l, atol=1e-12)
        np.testing.assert_allclose(se2_compose(base_b, ext), laser_b, atol=1e-12)
        np.testing.assert_allclose(laser_l, base_b, atol=1e-12)  # raw /gt pose in both cases


if __name__ == "__main__":
    unittest.main()
