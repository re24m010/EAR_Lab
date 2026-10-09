"""Streaming, read-only access to ROS1 bag files on Windows without ROS.

Built on ``rosbags`` (pure Python). Message definitions are taken from the
bag itself via ``AnyReader``; this is required because e.g. ``tf2_msgs`` is
not part of the default ROS1 typestore, and it guarantees that exactly the
recorded message layouts are used.

Memory: messages are read chunk by chunk and converted one at a time into
small plain dataclasses with numpy arrays. Large topics (images) are never
deserialized unless explicitly requested.

Timestamps are kept as integer nanoseconds (``int``) to avoid any float
rounding; ``stamp_ns`` is the message header stamp, ``bag_time_ns`` the time
at which the message was written into the bag.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rosbags_compat import apply_rosbags_windows_patch

apply_rosbags_windows_patch()

from rosbags.highlevel import AnyReader  # noqa: E402  (must follow the patch)
from rosbags.typesys import Stores, get_typestore  # noqa: E402

LASER_SCAN = "sensor_msgs/msg/LaserScan"
ODOMETRY = "nav_msgs/msg/Odometry"
TF_MESSAGE = "tf2_msgs/msg/TFMessage"


@dataclass(frozen=True)
class TopicInfo:
    topic: str
    msgtype: str
    count: int


@dataclass(frozen=True)
class LaserScan:
    stamp_ns: int
    bag_time_ns: int
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: np.ndarray        # float32, shape (N,)
    intensities: np.ndarray   # float32, shape (N,) or (0,)

    def angles(self) -> np.ndarray:
        """Beam angles in the laser frame (rad), angle_min + i * increment."""
        return self.angle_min + np.arange(len(self.ranges)) * self.angle_increment


@dataclass(frozen=True)
class Odometry:
    stamp_ns: int
    bag_time_ns: int
    frame_id: str
    child_frame_id: str
    position: np.ndarray        # (3,)
    orientation: np.ndarray     # (4,) x, y, z, w
    linear_velocity: np.ndarray   # (3,) in child frame (ROS convention)
    angular_velocity: np.ndarray  # (3,)
    pose_covariance: np.ndarray   # (36,)
    twist_covariance: np.ndarray  # (36,)


@dataclass(frozen=True)
class Transform:
    stamp_ns: int
    bag_time_ns: int
    parent_frame: str
    child_frame: str
    translation: np.ndarray   # (3,)
    rotation: np.ndarray      # (4,) x, y, z, w


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _vec3(v) -> np.ndarray:
    return np.array([v.x, v.y, v.z], dtype=float)


def _quat(q) -> np.ndarray:
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def _convert_scan(msg, bag_time_ns: int) -> LaserScan:
    return LaserScan(
        stamp_ns=_stamp_ns(msg.header.stamp),
        bag_time_ns=bag_time_ns,
        frame_id=msg.header.frame_id,
        angle_min=float(msg.angle_min),
        angle_max=float(msg.angle_max),
        angle_increment=float(msg.angle_increment),
        time_increment=float(msg.time_increment),
        scan_time=float(msg.scan_time),
        range_min=float(msg.range_min),
        range_max=float(msg.range_max),
        # copy: deserialized arrays may be views into the chunk buffer
        ranges=np.array(msg.ranges, dtype=np.float32, copy=True),
        intensities=np.array(msg.intensities, dtype=np.float32, copy=True),
    )


def _convert_odom(msg, bag_time_ns: int) -> Odometry:
    return Odometry(
        stamp_ns=_stamp_ns(msg.header.stamp),
        bag_time_ns=bag_time_ns,
        frame_id=msg.header.frame_id,
        child_frame_id=msg.child_frame_id,
        position=_vec3(msg.pose.pose.position),
        orientation=_quat(msg.pose.pose.orientation),
        linear_velocity=_vec3(msg.twist.twist.linear),
        angular_velocity=_vec3(msg.twist.twist.angular),
        pose_covariance=np.array(msg.pose.covariance, dtype=float),
        twist_covariance=np.array(msg.twist.covariance, dtype=float),
    )


def _convert_tf(msg, bag_time_ns: int) -> list[Transform]:
    return [
        Transform(
            stamp_ns=_stamp_ns(tr.header.stamp),
            bag_time_ns=bag_time_ns,
            parent_frame=tr.header.frame_id,
            child_frame=tr.child_frame_id,
            translation=_vec3(tr.transform.translation),
            rotation=_quat(tr.transform.rotation),
        )
        for tr in msg.transforms
    ]


_CONVERTERS = {
    LASER_SCAN: lambda m, t: [_convert_scan(m, t)],
    ODOMETRY: lambda m, t: [_convert_odom(m, t)],
    TF_MESSAGE: _convert_tf,
}


class BagReader:
    """Read-only, streaming ROS1 bag reader.

    Usage::

        with BagReader(path) as bag:
            for topic, item in bag.iter_messages(["/scan", "/odom"]):
                ...
    """

    def __init__(self, path: Path | str):
        self.path = Path(path)
        if not self.path.is_file():
            raise FileNotFoundError(self.path)
        self._reader = AnyReader(
            [self.path], default_typestore=get_typestore(Stores.ROS1_NOETIC)
        )

    def __enter__(self) -> "BagReader":
        self._reader.open()
        return self

    def __exit__(self, *exc) -> None:
        self._reader.close()

    @property
    def start_time_ns(self) -> int:
        return int(self._reader.start_time)

    @property
    def end_time_ns(self) -> int:
        return int(self._reader.end_time)

    @property
    def message_count(self) -> int:
        return int(self._reader.message_count)

    def topics(self) -> list[TopicInfo]:
        return sorted(
            (TopicInfo(c.topic, c.msgtype, int(c.msgcount)) for c in self._reader.connections),
            key=lambda t: t.topic,
        )

    def iter_messages(self, topics: Iterable[str]) -> Iterator[tuple[str, object]]:
        """Yield (topic, converted message) in bag-time order.

        TF messages are split into one ``Transform`` per contained transform.
        Only LaserScan, Odometry and TFMessage are supported.
        """
        wanted = set(topics)
        conns = [c for c in self._reader.connections if c.topic in wanted]
        missing = wanted - {c.topic for c in conns}
        if missing:
            raise KeyError(f"topics not in bag: {sorted(missing)}")
        for c in conns:
            if c.msgtype not in _CONVERTERS:
                raise TypeError(f"unsupported message type {c.msgtype} on {c.topic}")
        for conn, bag_time_ns, raw in self._reader.messages(connections=conns):
            msg = self._reader.deserialize(raw, conn.msgtype)
            for item in _CONVERTERS[conn.msgtype](msg, int(bag_time_ns)):
                yield conn.topic, item
