"""Isolated Windows compatibility workaround for ``rosbags``.

Problem (observed with rosbags 0.11.6, Python 3.12, Windows 11):
    ``rosbags.typesys.msg.normalize_msgtype`` instantiates ``pathlib.PosixPath``
    to split ROS message type names such as ``"sensor_msgs/LaserScan"``.
    On Windows ``PosixPath(...)`` raises
    ``NotImplementedError: cannot instantiate 'PosixPath' on your system``,
    so opening any ROS1 bag fails in ``Reader.open()``.

Fix:
    The names are only manipulated as strings (``.parent``, ``.name``, ``/``),
    the file system is never touched. ``pathlib.PurePosixPath`` offers exactly
    these operations with identical "/"-semantics on every OS. We therefore
    replace the module-level name ``PosixPath`` inside ``rosbags.typesys.msg``
    by ``PurePosixPath`` at runtime.

Properties:
    * The installed library files are NOT modified.
    * The patch is applied only if ``PosixPath`` cannot be instantiated.
    * It is idempotent and verified with a self-test after application.
"""

from __future__ import annotations

import pathlib
import warnings
from importlib.metadata import version

TESTED_ROSBAGS_VERSIONS = {"0.11.6"}

_patch_applied = False


def _posixpath_usable() -> bool:
    try:
        pathlib.PosixPath("a/b")
    except NotImplementedError:
        return False
    return True


def apply_rosbags_windows_patch() -> bool:
    """Apply the workaround if needed. Returns True if the patch is active."""
    global _patch_applied
    if _patch_applied:
        return True
    if _posixpath_usable():
        return False

    rosbags_version = version("rosbags")
    if rosbags_version not in TESTED_ROSBAGS_VERSIONS:
        warnings.warn(
            f"rosbags {rosbags_version} is untested with this workaround "
            f"(tested: {sorted(TESTED_ROSBAGS_VERSIONS)}).",
            stacklevel=2,
        )

    import rosbags.typesys.msg as rosbags_msg

    if getattr(rosbags_msg, "PosixPath", None) is pathlib.PosixPath:
        rosbags_msg.PosixPath = pathlib.PurePosixPath

    # Self-test: the patched function must give the documented ROS2-style name.
    result = rosbags_msg.normalize_msgtype("sensor_msgs/LaserScan")
    if result != "sensor_msgs/msg/LaserScan":
        raise RuntimeError(f"rosbags patch self-test failed: {result!r}")

    _patch_applied = True
    return True


def patch_active() -> bool:
    return _patch_applied
