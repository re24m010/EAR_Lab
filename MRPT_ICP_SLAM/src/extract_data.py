"""Phase C.2: extract laser scans, odometry, ground truth and extrinsics.

Reads the bag once (streaming) and writes to ``results/data``:

    scans.npz          stamp_ns, bag_time_ns, ranges (float32, N x beams), angles_rad
    scan_meta.json     LaserScan header fields (verified constant over all scans)
    odom.csv           wheel odometry, one row per /odom message
    gt.csv             ground truth (gt_map -> base_link), one row per transform
    tf_static.json     static sensor extrinsics
    odom_tum.txt       TUM format: t tx ty tz qx qy qz qw  (t in s)
    gt_tum.txt         TUM format
    dataset_info.json  provenance: file size/mtime/SHA-256, versions, topics, runtime

The bag is opened read-only; size and mtime are checked before and after.

Usage:  python src/extract_data.py [--config config/dataset.json] [--skip-hash]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
import time
from importlib.metadata import version
from pathlib import Path

import numpy as np
import pandas as pd

from bag_io import BagReader, LaserScan, Odometry, Transform
from project_config import DEFAULT_CONFIG, load_config, project_path
from rosbags_compat import patch_active
from transforms import quat_to_rpy

SCAN_META_FIELDS = (
    "frame_id", "angle_min", "angle_max", "angle_increment", "time_increment",
    "scan_time", "range_min", "range_max",
)


def sha256_file(path: Path, block: int = 16 * 1024 * 1024) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(block):
            h.update(chunk)
    return h.hexdigest()


def file_state(path: Path) -> dict:
    st = path.stat()
    return {"size_bytes": st.st_size, "mtime_ns": st.st_mtime_ns}


def ns_to_tum_time(ns: np.ndarray) -> list[str]:
    """Exact decimal seconds with 9 digits, no float rounding."""
    return [f"{int(v) // 1_000_000_000}.{int(v) % 1_000_000_000:09d}" for v in ns]


def write_tum(path: Path, stamp_ns, xyz, quat) -> None:
    times = ns_to_tum_time(stamp_ns)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for t, p, q in zip(times, xyz, quat):
            f.write(f"{t} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} "
                    f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


def pose_table(stamp_ns, bag_ns, xyz, quat, extra: dict | None = None) -> pd.DataFrame:
    xyz = np.asarray(xyz)
    quat = np.asarray(quat)
    rpy = quat_to_rpy(quat)
    df = pd.DataFrame({
        "stamp_ns": np.asarray(stamp_ns, dtype=np.int64),
        "bag_time_ns": np.asarray(bag_ns, dtype=np.int64),
        "x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2],
        "qx": quat[:, 0], "qy": quat[:, 1], "qz": quat[:, 2], "qw": quat[:, 3],
        "roll": rpy[:, 0], "pitch": rpy[:, 1], "yaw": rpy[:, 2],
    })
    for k, v in (extra or {}).items():
        df[k] = v
    return df


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--skip-hash", action="store_true", help="skip SHA-256 of the bag")
    args = ap.parse_args()

    cfg = load_config(args.config)
    bag_path = project_path(cfg["bag_path"])
    out_dir = project_path(cfg["output"]["data_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)
    topics = cfg["topics"]

    state_before = file_state(bag_path)
    t0 = time.perf_counter()
    digest = None if args.skip_hash else sha256_file(bag_path)
    t_hash = time.perf_counter() - t0
    print(f"bag: {bag_path}  size={state_before['size_bytes']} B"
          + ("" if digest is None else f"  sha256={digest} ({t_hash:.1f} s)"))

    t1 = time.perf_counter()
    with BagReader(bag_path) as bag:
        topic_list = bag.topics()
        counts = {t.topic: t.count for t in topic_list}
        bag_start, bag_end, n_total = bag.start_time_ns, bag.end_time_ns, bag.message_count

        n_scan = counts[topics["scan"]]
        scan_stamp = np.zeros(n_scan, dtype=np.int64)
        scan_bag = np.zeros(n_scan, dtype=np.int64)
        ranges = None
        scan_meta_values: dict[str, set] = {k: set() for k in SCAN_META_FIELDS}
        beam_counts, intensity_counts = set(), set()
        i_scan = 0

        odom_rows: list[Odometry] = []
        gt_rows: list[Transform] = []
        tf_static: list[Transform] = []

        for topic, item in bag.iter_messages(topics.values()):
            if topic == topics["scan"]:
                assert isinstance(item, LaserScan)
                if ranges is None:
                    ranges = np.full((n_scan, len(item.ranges)), np.nan, dtype=np.float32)
                if len(item.ranges) != ranges.shape[1]:
                    raise ValueError(f"scan {i_scan}: {len(item.ranges)} beams, "
                                     f"expected {ranges.shape[1]}")
                ranges[i_scan] = item.ranges
                scan_stamp[i_scan], scan_bag[i_scan] = item.stamp_ns, item.bag_time_ns
                for k in SCAN_META_FIELDS:
                    scan_meta_values[k].add(getattr(item, k))
                beam_counts.add(len(item.ranges))
                intensity_counts.add(len(item.intensities))
                if i_scan == 0:
                    angles = item.angles()
                i_scan += 1
            elif topic == topics["odom"]:
                odom_rows.append(item)
            elif topic == topics["ground_truth"]:
                gt_rows.append(item)
            elif topic == topics["tf_static"]:
                tf_static.append(item)
    t_read = time.perf_counter() - t1
    if i_scan != n_scan:
        raise RuntimeError(f"read {i_scan} scans, index says {n_scan}")

    # ---------------------------------------------------------------- scans
    np.savez_compressed(out_dir / "scans.npz", stamp_ns=scan_stamp, bag_time_ns=scan_bag,
                        ranges=ranges, angles_rad=angles)
    constant = all(len(v) == 1 for v in scan_meta_values.values())
    meta = {k: next(iter(v)) if len(v) == 1 else sorted(v) for k, v in scan_meta_values.items()}
    n_beams = next(iter(beam_counts))
    inc = meta["angle_increment"]
    meta.update({
        "header_fields_constant_over_all_scans": constant,
        "num_scans": n_scan,
        "num_beams": sorted(beam_counts) if len(beam_counts) > 1 else n_beams,
        "num_intensities": sorted(intensity_counts),
        "angle_min_deg": float(np.degrees(meta["angle_min"])),
        "angle_max_deg": float(np.degrees(meta["angle_max"])),
        "angle_increment_deg": float(np.degrees(inc)),
        "fov_first_to_last_beam_deg": float(np.degrees((n_beams - 1) * inc)),
        "fov_covered_deg": float(np.degrees(n_beams * inc)),
        "angle_max_consistent": bool(np.isclose(meta["angle_min"] + (n_beams - 1) * inc,
                                                meta["angle_max"], atol=1e-6)),
        "beam_center_offset_deg": float(np.degrees(meta["angle_min"]
                                                   + (n_beams - 1) * inc / 2)),
    })
    (out_dir / "scan_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")

    # ------------------------------------------------------------- odometry
    odom_frames = sorted({(o.frame_id, o.child_frame_id) for o in odom_rows})
    odom_df = pose_table(
        [o.stamp_ns for o in odom_rows], [o.bag_time_ns for o in odom_rows],
        [o.position for o in odom_rows], [o.orientation for o in odom_rows],
        extra={
            "vx": [o.linear_velocity[0] for o in odom_rows],
            "vy": [o.linear_velocity[1] for o in odom_rows],
            "vz": [o.linear_velocity[2] for o in odom_rows],
            "wx": [o.angular_velocity[0] for o in odom_rows],
            "wy": [o.angular_velocity[1] for o in odom_rows],
            "wz": [o.angular_velocity[2] for o in odom_rows],
            "pose_cov_all_zero": [not np.any(o.pose_covariance) for o in odom_rows],
            "twist_cov_all_zero": [not np.any(o.twist_covariance) for o in odom_rows],
        },
    )
    odom_df.to_csv(out_dir / "odom.csv", index=False)
    write_tum(out_dir / "odom_tum.txt", odom_df.stamp_ns,
              odom_df[["x", "y", "z"]].to_numpy(), odom_df[["qx", "qy", "qz", "qw"]].to_numpy())

    # ---------------------------------------------------------- ground truth
    gt_frames = sorted({(g.parent_frame, g.child_frame) for g in gt_rows})
    gt_df = pose_table(
        [g.stamp_ns for g in gt_rows], [g.bag_time_ns for g in gt_rows],
        [g.translation for g in gt_rows], [g.rotation for g in gt_rows],
    )
    gt_df.to_csv(out_dir / "gt.csv", index=False)
    write_tum(out_dir / "gt_tum.txt", gt_df.stamp_ns,
              gt_df[["x", "y", "z"]].to_numpy(), gt_df[["qx", "qy", "qz", "qw"]].to_numpy())

    # ------------------------------------------------------------ tf_static
    tf_list = []
    for tr in tf_static:
        rpy = quat_to_rpy(tr.rotation)
        tf_list.append({
            "parent": tr.parent_frame, "child": tr.child_frame, "stamp_ns": tr.stamp_ns,
            "translation_xyz": tr.translation.tolist(), "rotation_xyzw": tr.rotation.tolist(),
            "rpy_deg": np.degrees(rpy).tolist(),
        })
    (out_dir / "tf_static.json").write_text(json.dumps(tf_list, indent=2), encoding="utf-8")

    # ------------------------------------------------------------ provenance
    state_after = file_state(bag_path)
    info = {
        "sequence": cfg["sequence"],
        "bag_path": str(bag_path),
        "bag_file_before": state_before,
        "bag_file_after": state_after,
        "bag_unchanged": state_before == state_after,
        "sha256": digest,
        "bag_start_ns": bag_start, "bag_end_ns": bag_end,
        "bag_duration_s": (bag_end - bag_start) * 1e-9,
        "bag_message_count": n_total,
        "topics": [{"topic": t.topic, "msgtype": t.msgtype, "count": t.count} for t in topic_list],
        "extracted": {
            "scans": n_scan, "odom": len(odom_df), "gt_transforms": len(gt_df),
            "tf_static_transforms": len(tf_list),
            "odom_frames": odom_frames, "gt_frames": gt_frames,
        },
        "runtime_s": {"sha256": None if digest is None else t_hash, "bag_read": t_read},
        "software": {
            "python": sys.version.split()[0], "platform": platform.platform(),
            "rosbags": version("rosbags"), "numpy": np.__version__, "pandas": pd.__version__,
            "rosbags_windows_patch_active": patch_active(),
        },
    }
    (out_dir / "dataset_info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")

    print(f"read {n_total} bag messages in {t_read:.1f} s")
    print(f"scans: {n_scan} x {n_beams} beams, header constant: {constant}")
    print(f"odom: {len(odom_df)} {odom_frames}")
    print(f"gt: {len(gt_df)} {gt_frames}")
    print(f"tf_static: {len(tf_list)} transforms")
    print(f"bag unchanged (size+mtime): {info['bag_unchanged']}")
    print(f"outputs in {out_dir}")
    return 0 if info["bag_unchanged"] else 1


if __name__ == "__main__":
    sys.exit(main())
