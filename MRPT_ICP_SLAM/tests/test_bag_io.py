"""Integration tests for the rosbags workaround and the streaming bag reader.

Reads only the first few messages of the real bag (skipped if absent).
Run:  python -m unittest discover -s tests -v
"""

import pathlib
import sys
import unittest
from itertools import islice
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import bag_io  # noqa: E402  (applies the patch on import)
from project_config import load_config, project_path  # noqa: E402
from rosbags_compat import apply_rosbags_windows_patch, patch_active  # noqa: E402

BAG = project_path(load_config()["bag_path"])


class TestCompat(unittest.TestCase):
    def test_normalize_msgtype_works(self):
        import rosbags.typesys.msg as m
        self.assertEqual(m.normalize_msgtype("sensor_msgs/LaserScan"), "sensor_msgs/msg/LaserScan")
        self.assertEqual(m.normalize_msgtype("tf2_msgs/msg/TFMessage"), "tf2_msgs/msg/TFMessage")

    def test_patch_only_when_needed_and_idempotent(self):
        needed = True
        try:
            pathlib.PosixPath("a/b")
            needed = False
        except NotImplementedError:
            pass
        self.assertEqual(patch_active(), needed)
        self.assertEqual(apply_rosbags_windows_patch(), needed)


@unittest.skipUnless(BAG.is_file(), f"bag not found: {BAG}")
class TestBagReader(unittest.TestCase):
    def test_topics_and_types(self):
        with bag_io.BagReader(BAG) as bag:
            topics = {t.topic: t for t in bag.topics()}
        self.assertEqual(topics["/scan"].msgtype, bag_io.LASER_SCAN)
        self.assertEqual(topics["/odom"].msgtype, bag_io.ODOMETRY)
        self.assertEqual(topics["/gt"].msgtype, bag_io.TF_MESSAGE)
        self.assertEqual(topics["/tf_static"].msgtype, bag_io.TF_MESSAGE)

    def test_first_messages(self):
        with bag_io.BagReader(BAG) as bag:
            items = list(islice(bag.iter_messages(["/scan", "/odom", "/gt"]), 30))
        scans = [m for t, m in items if t == "/scan"]
        self.assertTrue(scans)
        s = scans[0]
        self.assertEqual(s.frame_id, "laser")
        self.assertEqual(len(s.ranges), 720)
        self.assertEqual(s.ranges.dtype, np.float32)
        self.assertAlmostEqual(np.degrees(s.angles()[-1]), 89.75, places=3)
        stamps = [m.stamp_ns for _, m in items]
        self.assertTrue(all(isinstance(v, int) for v in stamps))

    def test_unknown_topic_raises(self):
        with bag_io.BagReader(BAG) as bag:
            with self.assertRaises(KeyError):
                next(bag.iter_messages(["/does_not_exist"]))

    def test_unsupported_type_raises(self):
        with bag_io.BagReader(BAG) as bag:
            with self.assertRaises(TypeError):
                next(bag.iter_messages(["/d400/color/image_raw"]))


if __name__ == "__main__":
    unittest.main()
