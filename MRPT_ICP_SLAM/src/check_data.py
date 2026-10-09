"""Phase C.2: verify timestamps, frames, quaternions and data properties.

Input:  results/data (from extract_data.py)
Output: results/checks/data_checks.json   all numbers
        results/checks/data_checks.md     human-readable summary
        results/checks/revisit.npz        arrays for the revisit plot
        results/data/odom_aligned.csv     odometry interpolated to GT stamps, aligned two ways

Every value written here is computed from the data; interpretation is left to
documentation/data_preparation.md, where observations and conclusions are
kept apart.

Usage:  python src/check_data.py [--config config/dataset.json]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from dataset import (
    Scans, gt_base_and_laser_poses, gt_pose_for_scans, gt_se2, load_poses, load_scans,
    load_tf_static, lookup_static, odom_se2, poses_to_world_2d, scan_points_laser,
)
from project_config import DEFAULT_CONFIG, load_config, project_path
from transforms import (
    align_rigid_2d, cumulative_path_length, interpolate_se2, path_length, quat_to_rpy,
    se2_compose, se2_inverse, se2_relative, wrap_angle,
)


def _f(x) -> float:
    return float(np.round(x, 6))


def stats(a) -> dict:
    a = np.asarray(a, dtype=float)
    return {"min": _f(a.min()), "max": _f(a.max()), "mean": _f(a.mean()),
            "median": _f(np.median(a)), "std": _f(a.std())}


# ------------------------------------------------------------------ timing

def stream_timing(stamp_ns: np.ndarray, bag_ns: np.ndarray, gap_factor: float) -> dict:
    d_ns = np.diff(stamp_ns)
    med = np.median(d_ns)
    gaps = np.flatnonzero(d_ns > gap_factor * med)
    return {
        "count": int(len(stamp_ns)),
        "first_stamp_ns": int(stamp_ns[0]), "last_stamp_ns": int(stamp_ns[-1]),
        "duration_s": _f((stamp_ns[-1] - stamp_ns[0]) * 1e-9),
        "rate_hz": _f((len(stamp_ns) - 1) / ((stamp_ns[-1] - stamp_ns[0]) * 1e-9)),
        "dt_ms": stats(d_ns * 1e-6),
        "non_monotonic": int(np.sum(d_ns <= 0)),
        "gaps": [{"after_index": int(i), "dt_ms": _f(d_ns[i] * 1e-6)} for i in gaps],
        "bag_time_equals_stamp_exactly": int(np.sum(bag_ns == stamp_ns)),
        "bag_minus_stamp_ms": stats((bag_ns - stamp_ns) * 1e-6),
    }


# ----------------------------------------------------------------- scans

def scan_checks(scans: Scans, odom: pd.DataFrame) -> dict:
    r, m = scans.ranges, scans.meta
    nan, posinf, neginf = np.isnan(r), np.isposinf(r), np.isneginf(r)
    fin = np.isfinite(r)
    below = fin & (r < m["range_min"])
    above = fin & (r > m["range_max"])
    valid = scans.valid_mask()
    total = r.size
    per_scan = valid.mean(axis=1)
    per_beam = valid.mean(axis=0)
    n = r.shape[1]
    acq = (n - 1) * m["time_increment"]
    v_max = float(np.max(np.abs(odom["vx"])))
    w_max = float(np.max(np.abs(odom["wz"])))
    worst_beams = np.argsort(per_beam)[:10]
    return {
        "num_scans": int(r.shape[0]), "num_beams": int(n),
        "fov_deg": _f(np.degrees((n - 1) * m["angle_increment"])),
        "angle_min_deg": _f(m["angle_min_deg"]), "angle_max_deg": _f(m["angle_max_deg"]),
        "angle_increment_deg": _f(m["angle_increment_deg"]),
        "beam_center_offset_deg": _f(m["beam_center_offset_deg"]),
        "range_categories_fraction": {
            "valid": _f(valid.sum() / total), "nan": _f(nan.sum() / total),
            "+inf": _f(posinf.sum() / total), "-inf": _f(neginf.sum() / total),
            "below_range_min": _f(below.sum() / total), "above_range_max": _f(above.sum() / total),
        },
        "valid_fraction_per_scan": stats(per_scan),
        "scans_with_valid_fraction_below_0.8": int(np.sum(per_scan < 0.8)),
        "valid_range_m": {"min": _f(r[valid].min()), "max": _f(r[valid].max()),
                          "median": _f(np.median(r[valid])),
                          "p99": _f(np.percentile(r[valid], 99))},
        "least_valid_beams": [{"beam": int(b), "angle_deg": _f(np.degrees(scans.angles[b])),
                               "valid_fraction": _f(per_beam[b])} for b in worst_beams],
        "timing_model": {
            "scan_time_s": m["scan_time"], "time_increment_s": m["time_increment"],
            "steps_per_revolution": _f(m["scan_time"] / m["time_increment"]),
            "deg_per_step_from_timing": _f(360.0 / (m["scan_time"] / m["time_increment"])),
            "acquisition_time_of_720_beams_ms": _f(acq * 1e3),
        },
        "motion_during_one_scan": {
            "max_odom_speed_mps": _f(v_max), "max_odom_yaw_rate_radps": _f(w_max),
            "max_translation_cm": _f(v_max * acq * 100), "max_rotation_deg": _f(np.degrees(w_max * acq)),
        },
    }


# ----------------------------------------------------- frames / quaternions

def frame_checks(tf_static: list[dict], scans: Scans, base: str, laser: str) -> dict:
    ext = lookup_static(tf_static, base, laser)
    rot = ext.matrix[:3, :3]
    tree = {}
    for tr in tf_static:
        tree.setdefault(tr["parent"], []).append(tr["child"])
    roll, pitch, yaw = np.degrees(quat_to_rpy(
        next(tr["rotation_xyzw"] for tr in tf_static if tr["child"] == laser)))
    # Elevation of every beam direction in base_link (scan plane tilt).
    dirs = np.column_stack([np.cos(scans.angles), np.sin(scans.angles), np.zeros(len(scans.angles))])
    elev = np.degrees(np.arcsin((dirs @ rot.T)[:, 2]))
    i_low = int(np.argmin(elev))
    return {
        "tf_tree": tree,
        "base_to_laser": {
            "translation_m": [_f(v) for v in ext.matrix[:3, 3]],
            "rpy_deg": [_f(roll), _f(pitch), _f(yaw)],
            "se2_x_y_yawdeg": [_f(ext.se2()[0]), _f(ext.se2()[1]), _f(np.degrees(ext.se2()[2]))],
            "laser_x_axis_in_base": [_f(v) for v in rot[:, 0]],
            "laser_z_axis_in_base": [_f(v) for v in rot[:, 2]],
            "laser_upright": bool(rot[2, 2] > 0),
            "rotation_orthonormal_error": _f(np.abs(rot.T @ rot - np.eye(3)).max()),
        },
        "scan_plane_tilt": {
            "beam_elevation_deg": stats(elev),
            "lowest_beam_angle_deg": _f(np.degrees(scans.angles[i_low])),
            "lowest_beam_elevation_deg": _f(elev[i_low]),
            "floor_hit_distance_if_base_link_on_floor_m":
                _f(ext.matrix[2, 3] / np.tan(np.radians(-elev[i_low]))) if elev[i_low] < 0 else None,
        },
    }


def quaternion_checks(odom: pd.DataFrame, gt: pd.DataFrame, tf_static: list[dict]) -> dict:
    def norms(df):
        n = np.linalg.norm(df[["qx", "qy", "qz", "qw"]].to_numpy(), axis=1)
        return {"min": _f(n.min()), "max": _f(n.max()), "max_abs_dev_from_1": _f(np.abs(n - 1).max())}
    tf_n = [np.linalg.norm(tr["rotation_xyzw"]) for tr in tf_static]
    return {
        "odom_norm": norms(odom), "gt_norm": norms(gt),
        "tf_static_norm_max_abs_dev": _f(np.max(np.abs(np.array(tf_n) - 1))),
        "odom_roll_pitch_deg": {"roll": stats(np.degrees(odom["roll"])),
                                "pitch": stats(np.degrees(odom["pitch"]))},
        "odom_z_m": stats(odom["z"]),
        "gt_roll_pitch_deg": {"roll": stats(np.degrees(gt["roll"])),
                              "pitch": stats(np.degrees(gt["pitch"]))},
        "gt_z_m": stats(gt["z"]),
    }


def twist_consistency(odom: pd.DataFrame) -> dict:
    """Compare reported twist with finite differences of the reported pose."""
    t = odom["t"].to_numpy()
    p = odom_se2(odom)
    yaw = np.unwrap(p[:, 2])
    dx, dy = np.gradient(p[:, 0], t), np.gradient(p[:, 1], t)
    v_fwd = np.cos(yaw) * dx + np.sin(yaw) * dy      # velocity in base_link x
    v_lat = -np.sin(yaw) * dx + np.cos(yaw) * dy
    w = np.gradient(yaw, t)
    return {
        "vx_twist_vs_pose_derivative": {"corr": _f(np.corrcoef(odom["vx"], v_fwd)[0, 1]),
                                        "median_abs_diff_mps": _f(np.median(np.abs(odom["vx"] - v_fwd)))},
        "wz_twist_vs_pose_derivative": {"corr": _f(np.corrcoef(odom["wz"], w)[0, 1]),
                                        "median_abs_diff_radps": _f(np.median(np.abs(odom["wz"] - w)))},
        "lateral_velocity_from_pose_mps": stats(v_lat),
        "twist_vy_all_zero": bool(np.all(odom["vy"] == 0)),
        "pose_covariance_all_zero": bool(odom["pose_cov_all_zero"].all()),
        "twist_covariance_all_zero": bool(odom["twist_cov_all_zero"].all()),
    }


def pose_jump_check(df: pd.DataFrame, poses: np.ndarray, max_speed: float, t0: float) -> dict:
    """Consecutive pose steps whose implied speed exceeds a physical limit."""
    t = df["t"].to_numpy()
    dt = np.diff(t)
    step = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    v = step / dt
    bad = np.flatnonzero(v > max_speed)
    still = step == 0
    extra = {}
    if "vx" in df:  # odometry: pose not updated although the twist reports motion
        moving = np.abs(df["vx"].to_numpy()[1:]) > 0.3
        extra["zero_motion_steps_while_vx_above_0.3"] = int((still & moving).sum())
        extra["double_steps_above_1.8x_median"] = int(np.sum(step > 1.8 * np.median(step[~still])))
    return {
        **extra,
        "max_plausible_speed_mps": max_speed,
        "implied_speed_mps": {"median": _f(np.median(v)), "p99": _f(np.percentile(v, 99)), "max": _f(v.max())},
        "steps_above_limit": int(len(bad)),
        "events": [{"t_rel_s": _f(t[i] - t0), "step_cm": _f(step[i] * 100), "dt_ms": _f(dt[i] * 1e3),
                    "implied_speed_mps": _f(v[i])} for i in bad],
        "zero_motion_steps": int(still.sum()),
    }


# ------------------------------------------------- time association / lag

def scan_gt_association(scans: Scans, gt: pd.DataFrame) -> dict:
    idx, has = gt_pose_for_scans(scans, gt)
    missing = np.flatnonzero(~has)
    gt_set = set(gt["stamp_ns"].astype(np.int64))
    scan_set = set(scans.stamp_ns.astype(np.int64))
    return {
        "scans": int(len(scans.stamp_ns)), "gt_poses": int(len(gt)),
        "scans_with_identical_gt_stamp": int(has.sum()),
        "gt_stamps_not_equal_to_any_scan_stamp": len(gt_set - scan_set),
        "scan_indices_without_gt": missing.tolist(),
        "scan_times_without_gt_rel_s": [_f((scans.stamp_ns[i] - scans.stamp_ns[0]) * 1e-9) for i in missing],
    }


def odom_phase_to_scans(odom: pd.DataFrame, scans: Scans) -> dict:
    s = scans.stamp_ns
    o = odom["stamp_ns"].to_numpy()
    j = np.clip(np.searchsorted(s, o), 1, len(s) - 1)
    nearest = np.where(np.abs(o - s[j - 1]) < np.abs(o - s[j]), s[j - 1], s[j])
    d = (o - nearest) * 1e-6
    return {"odom_minus_nearest_scan_ms": stats(d),
            "odom_stamps_identical_to_a_scan": int(np.sum(d == 0))}


def estimate_lag(t_ref, sig_ref, t_q, sig_q, max_lag_s, step_s=0.001) -> dict:
    """Find tau maximising corr(sig_q(t), sig_ref(t + tau))."""
    lags = np.arange(-max_lag_s, max_lag_s + step_s / 2, step_s)
    inside = (t_q - max_lag_s >= t_ref[0]) & (t_q + max_lag_s <= t_ref[-1])
    corr = np.array([np.corrcoef(sig_q[inside], np.interp(t_q[inside] + lag, t_ref, sig_ref))[0, 1]
                     for lag in lags])
    k = int(np.argmax(corr))
    return {"best_lag_ms": _f(lags[k] * 1e3), "corr_at_best": _f(corr[k]),
            "corr_at_zero": _f(corr[np.argmin(np.abs(lags))]), "_lags": lags, "_corr": corr}


def odom_gt_lag(odom: pd.DataFrame, gt: pd.DataFrame, max_lag_ms: float) -> dict:
    tg = gt["t"].to_numpy()
    pg = gt_se2(gt)
    to = odom["t"].to_numpy()
    w_gt = np.gradient(np.unwrap(pg[:, 2]), tg)
    v_gt = np.hypot(np.gradient(pg[:, 0], tg), np.gradient(pg[:, 1], tg))
    yaw_rate = estimate_lag(tg, w_gt, to, odom["wz"].to_numpy(), max_lag_ms * 1e-3)
    speed = estimate_lag(tg, v_gt, to, np.abs(odom["vx"].to_numpy()), max_lag_ms * 1e-3)
    return {"yaw_rate": yaw_rate, "speed": speed}


# ------------------------------------------------------- odom alignment

def align_odometry(odom: pd.DataFrame, gt: pd.DataFrame, pg: np.ndarray) -> tuple[pd.DataFrame, dict]:
    """pg: GT base_link poses (x, y, yaw), one per row of gt."""
    tg = gt["t"].to_numpy()
    po, valid = interpolate_se2(odom["t"].to_numpy(), odom_se2(odom), tg)
    i0 = int(np.flatnonzero(valid)[0])
    g, o = pg[valid], po[valid]

    t_init = se2_compose(pg[i0], se2_inverse(po[i0]))
    o_init = se2_compose(np.broadcast_to(t_init, o.shape), o)
    t_ls = align_rigid_2d(o[:, :2], g[:, :2])
    o_ls = se2_compose(np.broadcast_to(t_ls, o.shape), o)

    def errors(a):
        e = np.linalg.norm(a[:, :2] - g[:, :2], axis=1)
        ey = np.degrees(np.abs(wrap_angle(a[:, 2] - g[:, 2])))
        return {"pos_rmse_m": _f(np.sqrt(np.mean(e ** 2))), "pos_max_m": _f(e.max()),
                "pos_final_m": _f(e[-1]), "yaw_abs_mean_deg": _f(ey.mean()),
                "yaw_final_deg": _f(ey[-1])}

    sub = slice(None, None, 20)  # 0.5 s subsampling reduces noise-inflated length
    out = pd.DataFrame({
        "stamp_ns": gt["stamp_ns"].to_numpy()[valid],
        "gt_x": g[:, 0], "gt_y": g[:, 1], "gt_yaw": g[:, 2],
        "odom_init_x": o_init[:, 0], "odom_init_y": o_init[:, 1], "odom_init_yaw": o_init[:, 2],
        "odom_se2_x": o_ls[:, 0], "odom_se2_y": o_ls[:, 1], "odom_se2_yaw": o_ls[:, 2],
    })
    info = {
        "note": "Odometry characterisation only - NOT a SLAM result.",
        "gt_poses_used": int(valid.sum()), "gt_poses_outside_odom_time_range": int((~valid).sum()),
        "initial_pose_alignment": {"transform_x_y_yawdeg": [_f(t_init[0]), _f(t_init[1]), _f(np.degrees(t_init[2]))],
                                   **errors(o_init)},
        "se2_least_squares_alignment": {"transform_x_y_yawdeg": [_f(t_ls[0]), _f(t_ls[1]), _f(np.degrees(t_ls[2]))],
                                        **errors(o_ls)},
        "path_length_m": {"gt_full_rate": _f(path_length(g[:, :2])), "odom_full_rate": _f(path_length(o[:, :2])),
                          "gt_0.5s": _f(path_length(g[sub, :2])), "odom_0.5s": _f(path_length(o[sub, :2]))},
        "odom_to_gt_length_ratio_0.5s": _f(path_length(o[sub, :2]) / path_length(g[sub, :2])),
    }
    return out, info


# -------------------------------------------------------------- revisits

def revisit_analysis(gt: pd.DataFrame, min_dt: float, min_ds: float, radii: list[float]) -> tuple[dict, dict]:
    p = gt_se2(gt)
    t = gt["t"].to_numpy()
    s = cumulative_path_length(p[:, :2])
    d = np.linalg.norm(p[:, None, :2] - p[None, :, :2], axis=2)
    eligible = (np.abs(t[:, None] - t[None, :]) >= min_dt) & (np.abs(s[:, None] - s[None, :]) >= min_ds)
    d_masked = np.where(eligible, d, np.inf)
    nearest = d_masked.min(axis=1)
    partner = d_masked.argmin(axis=1)
    i = int(np.argmin(nearest))
    j = int(partner[i])
    heading_diff = np.degrees(np.abs(wrap_angle(p[i, 2] - p[j, 2])))
    yaw_u = np.unwrap(p[:, 2])
    out = {
        "criterion": f"pairs with |dt| >= {min_dt} s and |ds| >= {min_ds} m along the path",
        "start_end_distance_m": _f(np.linalg.norm(p[-1, :2] - p[0, :2])),
        "gt_path_length_m": _f(s[-1]),
        "bbox_m": {"x": [_f(p[:, 0].min()), _f(p[:, 0].max())], "y": [_f(p[:, 1].min()), _f(p[:, 1].max())]},
        "net_heading_change_deg": _f(np.degrees(yaw_u[-1] - yaw_u[0])),
        "total_abs_heading_change_deg": _f(np.degrees(np.abs(np.diff(yaw_u)).sum())),
        "closest_revisit": {"distance_m": _f(nearest[i]), "i": i, "j": j,
                            "t_i_rel_s": _f(t[i] - t[0]), "t_j_rel_s": _f(t[j] - t[0]),
                            "heading_difference_deg": _f(heading_diff)},
        "fraction_of_poses_with_revisit_within_radius":
            {str(r): _f(np.mean(nearest <= r)) for r in radii},
    }
    arrays = {"t_rel": t - t[0], "nearest_m": nearest, "partner": partner, "xy": p[:, :2], "s": s}
    return out, arrays


def reobservation_analysis(scans: Scans, gt: pd.DataFrame, laser_poses: np.ndarray, stride: int,
                           res: float = 0.25, min_gap_s: float = 10.0) -> dict:
    """Proxy for laser-level revisits: grid cells hit again after >= min_gap_s without hits."""
    idx, has = gt_pose_for_scans(scans, gt)
    valid = scans.valid_mask()
    cells_all, times_all = [], []
    for k in np.flatnonzero(has)[::stride]:
        pts = scan_points_laser(scans.ranges[k], scans.angles, valid[k])[:, :2]
        w = poses_to_world_2d(pts, laser_poses[idx[k]])
        c = np.unique(np.floor(w / res).astype(np.int64) @ np.array([1, 1_000_003]))
        cells_all.append(c)
        times_all.append(np.full(len(c), scans.t[k] - scans.t[0]))
    cells = np.concatenate(cells_all)
    times = np.concatenate(times_all)
    order = np.lexsort((times, cells))
    cells, times = cells[order], times[order]
    same = cells[1:] == cells[:-1]
    gaps = np.where(same, np.diff(times), 0.0)
    uniq, start = np.unique(cells, return_index=True)
    max_gap = np.maximum.reduceat(np.concatenate([gaps, [0.0]]), start)
    reobs = max_gap >= min_gap_s
    return {"grid_resolution_m": res, "min_gap_s": min_gap_s, "scan_stride": stride,
            "occupied_cells": int(len(uniq)),
            "cells_reobserved_after_gap": int(reobs.sum()),
            "fraction_reobserved_after_gap": _f(reobs.mean())}


# ------------------------------------------- extrinsic / convention check

def registration_sharpness(scans: Scans, gt: pd.DataFrame, extrinsic_se2, stride: int, res: float) -> dict:
    """Count occupied cells of GT-registered scans for several hypotheses.

    Fewer occupied cells = sharper accumulated map = hypothesis more
    consistent with how the GT poses were produced.
    """
    idx, has = gt_pose_for_scans(scans, gt)
    pg = gt_se2(gt)
    valid = scans.valid_mask()
    hypotheses = {
        "A_extrinsic_from_tf_static": (extrinsic_se2, 1.0),
        "B_no_extrinsic_laser_at_base": (np.zeros(3), 1.0),
        "C_extrinsic_mirrored_beams": (extrinsic_se2, -1.0),
        "D_extrinsic_yaw_plus_180deg": (extrinsic_se2 + np.array([0, 0, np.pi]), 1.0),
    }
    result = {}
    for name, (ext, sign) in hypotheses.items():
        cells = []
        for k in np.flatnonzero(has)[::stride]:
            pts = scan_points_laser(scans.ranges[k], sign * scans.angles, valid[k])[:, :2]
            w = poses_to_world_2d(pts, se2_compose(pg[idx[k]], ext))
            cells.append(np.floor(w / res).astype(np.int64) @ np.array([1, 1_000_003]))
        result[name] = int(len(np.unique(np.concatenate(cells))))
    ref = result["A_extrinsic_from_tf_static"]
    return {"grid_resolution_m": res, "scan_stride": stride, "occupied_cells": result,
            "ratio_to_A": {k: _f(v / ref) for k, v in result.items()}}


def extrinsic_grid_search(scans: Scans, gt: pd.DataFrame, extrinsic_se2, xy_range, yaws_deg,
                          stride: int, res: float) -> dict:
    """Which planar laser offset w.r.t. the /gt child frame gives the sharpest map?

    If /gt were base_link, the optimum should lie near the tf_static extrinsic;
    if /gt is the laser frame, near (0, 0, 0).
    """
    idx, has = gt_pose_for_scans(scans, gt)
    pg = gt_se2(gt)
    valid = scans.valid_mask()
    ks = np.flatnonzero(has)[::stride]
    pts = [scan_points_laser(scans.ranges[k], scans.angles, valid[k])[:, :2] for k in ks]

    def n_cells(ext):
        c = [np.floor(poses_to_world_2d(p, se2_compose(pg[idx[k]], ext)) / res).astype(np.int64)
             @ np.array([1, 1_000_003]) for k, p in zip(ks, pts)]
        return int(len(np.unique(np.concatenate(c))))

    lo, hi, step = xy_range
    grid = np.round(np.arange(lo, hi + step / 2, step), 6)
    results = [(n_cells(np.array([x, y, np.radians(yw)])), float(x), float(y), float(yw))
               for yw in yaws_deg for x in grid for y in grid]
    results.sort()
    return {
        "grid_resolution_m": res, "scan_stride": stride,
        "search": {"xy_m": list(xy_range), "yaw_deg": list(yaws_deg), "evaluations": len(results)},
        "best": [{"cells": c, "x": _f(x), "y": _f(y), "yaw_deg": _f(yw)} for c, x, y, yw in results[:5]],
        "cells_at_tf_static_extrinsic": n_cells(extrinsic_se2),
        "cells_at_zero_offset": n_cells(np.zeros(3)),
    }


def gt_frame_odometry_test(odom: pd.DataFrame, gt: pd.DataFrame, extrinsic_se2,
                           windows_s, turn_deg: float) -> dict:
    """Compare odometry (base_link) relative motion with GT under both frame hypotheses.

    A laser mounted off the rotation axis moves on an arc while the robot
    turns, so the hypotheses differ mainly in turning segments.
    """
    tg = gt["t"].to_numpy()
    po, valid = interpolate_se2(odom["t"].to_numpy(), odom_se2(odom), tg)
    hyp = {h: gt_base_and_laser_poses(gt, extrinsic_se2, h)[0] for h in ("base_link", "laser")}
    rate = 1.0 / np.median(np.diff(tg))
    out = {}
    for w in windows_s:
        n = int(round(w * rate))
        i = np.flatnonzero(valid[:-n] & valid[n:])
        j = i + n
        # only pairs whose index gap really spans w seconds (skip GT gaps)
        ok = np.abs((tg[j] - tg[i]) - w) < 0.5 / rate
        i, j = i[ok], j[ok]
        ro = se2_relative(po[i], po[j])
        turning = np.abs(wrap_angle(hyp["base_link"][j, 2] - hyp["base_link"][i, 2])) > np.radians(turn_deg)
        res = {"pairs": int(len(i)), "turning_pairs": int(turning.sum())}
        for h, p in hyp.items():
            e = np.linalg.norm(ro[:, :2] - se2_relative(p[i], p[j])[:, :2], axis=1)
            res[f"gt_as_{h}"] = {"rmse_all_m": _f(np.sqrt(np.mean(e ** 2))),
                                 "rmse_turning_m": _f(np.sqrt(np.mean(e[turning] ** 2))),
                                 "rmse_straight_m": _f(np.sqrt(np.mean(e[~turning] ** 2)))}
        out[f"{w:g}s"] = res
    return out


# ------------------------------------------------------------------ main

def to_markdown(c: dict) -> str:
    sc, ass, rv, al = c["scans"], c["scan_gt_association"], c["revisits"], c["odom_alignment"]
    fr, lag, rs = c["frames"], c["odom_gt_lag"], c["registration_sharpness"]
    lines = [
        "# Data checks: " + c["sequence"], "",
        "Generated by `src/check_data.py`. All numbers computed from the bag extraction.", "",
        "## Streams",
        "| stream | count | rate [Hz] | dt median [ms] | dt min/max [ms] | gaps | bag time == stamp |",
        "|---|---|---|---|---|---|---|",
    ]
    for k, v in c["timing"].items():
        lines.append(f"| {k} | {v['count']} | {v['rate_hz']:.2f} | {v['dt_ms']['median']:.2f} | "
                     f"{v['dt_ms']['min']:.2f} / {v['dt_ms']['max']:.2f} | {len(v['gaps'])} | "
                     f"{v['bag_time_equals_stamp_exactly']}/{v['count']} |")
    rc = sc["range_categories_fraction"]
    lines += [
        "", "## Laser scan",
        f"- {sc['num_scans']} scans x {sc['num_beams']} beams, {sc['angle_min_deg']:.2f} deg to "
        f"{sc['angle_max_deg']:.2f} deg, increment {sc['angle_increment_deg']:.4f} deg "
        f"(first-to-last beam {sc['fov_deg']:.2f} deg)",
        f"- beam centre offset {sc['beam_center_offset_deg']:.4f} deg",
        f"- ranges: valid {rc['valid']:.3f}, +inf {rc['+inf']:.3f}, nan {rc['nan']:.3f}, "
        f"< range_min {rc['below_range_min']:.3f}, > range_max {rc['above_range_max']:.3f}",
        f"- valid fraction per scan: min {sc['valid_fraction_per_scan']['min']:.3f}, "
        f"median {sc['valid_fraction_per_scan']['median']:.3f}",
        f"- timing: {sc['timing_model']['steps_per_revolution']:.1f} steps/rev -> "
        f"{sc['timing_model']['deg_per_step_from_timing']:.4f} deg/step; 720 beams take "
        f"{sc['timing_model']['acquisition_time_of_720_beams_ms']:.2f} ms",
        f"- max motion during one scan: {sc['motion_during_one_scan']['max_translation_cm']:.2f} cm, "
        f"{sc['motion_during_one_scan']['max_rotation_deg']:.3f} deg",
        "", "## Frames",
        f"- base_link -> laser: t = {fr['base_to_laser']['translation_m']} m, "
        f"rpy = {fr['base_to_laser']['rpy_deg']} deg, upright = {fr['base_to_laser']['laser_upright']}",
        f"- scan plane: lowest beam elevation {fr['scan_plane_tilt']['lowest_beam_elevation_deg']:.3f} deg "
        f"at beam angle {fr['scan_plane_tilt']['lowest_beam_angle_deg']:.2f} deg",
        "", "## Pose plausibility (implied speed between consecutive poses)",
    ]
    for k, v in c["pose_jumps"].items():
        ev = ", ".join(f"{e['t_rel_s']:.2f} s ({e['step_cm']:.1f} cm / {e['dt_ms']:.0f} ms)" for e in v["events"])
        lines.append(f"- {k}: median {v['implied_speed_mps']['median']} m/s, max {v['implied_speed_mps']['max']} m/s; "
                     f"{v['steps_above_limit']} steps > {v['max_plausible_speed_mps']} m/s"
                     + (f": {ev}" if ev else "") + f"; zero-motion steps: {v['zero_motion_steps']}")
    lines += [
        "", "## Scan / GT association",
        f"- {ass['scans_with_identical_gt_stamp']}/{ass['scans']} scans have a GT pose with an identical stamp; "
        f"GT stamps not equal to any scan stamp: {ass['gt_stamps_not_equal_to_any_scan_stamp']}",
        f"- scans without GT: indices {ass['scan_indices_without_gt']}",
        "", "## Odometry vs GT time lag (correlation)",
        f"- yaw rate: best lag {lag['yaw_rate']['best_lag_ms']} ms (r = {lag['yaw_rate']['corr_at_best']}, "
        f"r(0) = {lag['yaw_rate']['corr_at_zero']})",
        f"- speed: best lag {lag['speed']['best_lag_ms']} ms (r = {lag['speed']['corr_at_best']}, "
        f"r(0) = {lag['speed']['corr_at_zero']})",
        "", f"## Odometry vs GT base_link (GT interpreted as `{al['gt_interpretation']}`; "
        "characterisation, not SLAM)",
        f"- initial-pose alignment: RMSE {al['initial_pose_alignment']['pos_rmse_m']} m, "
        f"final {al['initial_pose_alignment']['pos_final_m']} m",
        f"- SE(2) least squares: RMSE {al['se2_least_squares_alignment']['pos_rmse_m']} m",
        f"- length ratio odom/GT (0.5 s sampling): {al['odom_to_gt_length_ratio_0.5s']}",
        "", "## Revisits",
        f"- {rv['criterion']}",
        f"- start-end distance {rv['start_end_distance_m']} m, net heading change "
        f"{rv['net_heading_change_deg']} deg",
        f"- closest revisit: {rv['closest_revisit']['distance_m']} m "
        f"(t = {rv['closest_revisit']['t_i_rel_s']} s vs {rv['closest_revisit']['t_j_rel_s']} s, "
        f"heading difference {rv['closest_revisit']['heading_difference_deg']} deg)",
        f"- fraction of poses with a revisit within radius: {rv['fraction_of_poses_with_revisit_within_radius']}",
        f"- laser re-observation proxy: {c['reobservation']}",
        "", "## GT child frame: which frame do the /gt poses describe?",
        "Registration sharpness (occupied 5 cm cells of GT-registered scans, lower = sharper):",
        f"- {rs['occupied_cells']}", f"- ratio to A: {rs['ratio_to_A']}",
        f"- extrinsic grid search best: {c['extrinsic_grid_search']['best'][:3]}",
        f"- cells at tf_static extrinsic {c['extrinsic_grid_search']['cells_at_tf_static_extrinsic']}, "
        f"at zero offset {c['extrinsic_grid_search']['cells_at_zero_offset']}",
        "Odometry relative-motion test (RMSE of relative translation odom vs GT):",
    ]
    for w, r in c["gt_frame_odometry_test"].items():
        lines.append(f"- {w} ({r['pairs']} pairs, {r['turning_pairs']} turning): "
                     f"GT as base_link {r['gt_as_base_link']}, GT as laser {r['gt_as_laser']}")
    lines += [f"- interpretation used downstream: `{c['gt_child_frame_interpretation_used']}`", ""]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    args = ap.parse_args()
    cfg = load_config(args.config)
    ck = cfg["checks"]
    data_dir = project_path(cfg["output"]["data_dir"])
    out_dir = project_path(cfg["output"]["checks_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    scans = load_scans(data_dir)
    odom = load_poses(data_dir, "odom")
    gt = load_poses(data_dir, "gt")
    tf_static = load_tf_static(data_dir)
    ext = lookup_static(tf_static, cfg["frames"]["base"], cfg["frames"]["laser"]).se2()

    checks = {"sequence": cfg["sequence"], "config": cfg["_config_path"]}
    checks["timing"] = {
        "scan": stream_timing(scans.stamp_ns, scans.bag_time_ns, ck["gap_factor"]),
        "odom": stream_timing(odom["stamp_ns"].to_numpy(), odom["bag_time_ns"].to_numpy(), ck["gap_factor"]),
        "gt": stream_timing(gt["stamp_ns"].to_numpy(), gt["bag_time_ns"].to_numpy(), ck["gap_factor"]),
    }
    checks["scans"] = scan_checks(scans, odom)
    checks["frames"] = frame_checks(tf_static, scans, cfg["frames"]["base"], cfg["frames"]["laser"])
    checks["quaternions"] = quaternion_checks(odom, gt, tf_static)
    checks["odom_twist"] = twist_consistency(odom)
    t0 = scans.t[0]
    checks["pose_jumps"] = {
        "gt": pose_jump_check(gt, gt_se2(gt), ck["max_plausible_speed_mps"], t0),
        "odom": pose_jump_check(odom, odom_se2(odom), ck["max_plausible_speed_mps"], t0),
    }
    checks["scan_gt_association"] = scan_gt_association(scans, gt)
    checks["odom_phase_to_scans"] = odom_phase_to_scans(odom, scans)
    lag = odom_gt_lag(odom, gt, ck["odom_gt_lag_search_ms"])
    lag_arrays = {f"{k}_{a}": v.pop(f"_{a}") for k, v in lag.items() for a in ("lags", "corr")}
    checks["odom_gt_lag"] = lag

    # GT frame interpretation tests (independent of the configured choice)
    checks["registration_sharpness"] = registration_sharpness(
        scans, gt, ext, ck["consistency_scan_stride"], ck["consistency_grid_resolution_m"])
    checks["extrinsic_grid_search"] = extrinsic_grid_search(
        scans, gt, ext, ck["extrinsic_search_xy_m"], ck["extrinsic_search_yaw_deg"],
        ck["extrinsic_search_scan_stride"], ck["consistency_grid_resolution_m"])
    checks["gt_frame_odometry_test"] = gt_frame_odometry_test(
        odom, gt, ext, ck["relative_motion_windows_s"], ck["turning_threshold_deg"])

    interp = cfg["ground_truth"]["child_frame_interpretation"]
    gt_base, gt_laser = gt_base_and_laser_poses(gt, ext, interp)
    checks["gt_child_frame_interpretation_used"] = interp

    aligned, checks["odom_alignment"] = align_odometry(odom, gt, gt_base)
    checks["odom_alignment"]["gt_interpretation"] = interp
    _, checks["odom_alignment_if_gt_is_base_link"] = align_odometry(
        odom, gt, gt_base_and_laser_poses(gt, ext, "base_link")[0])
    aligned.to_csv(data_dir / "odom_aligned.csv", index=False)

    checks["revisits"], rv_arrays = revisit_analysis(
        gt, ck["revisit_min_time_separation_s"], ck["revisit_min_path_separation_m"], ck["revisit_radii_m"])
    checks["reobservation"] = reobservation_analysis(scans, gt, gt_laser, ck["consistency_scan_stride"])

    np.savez_compressed(out_dir / "revisit.npz", **rv_arrays, **lag_arrays)
    (out_dir / "data_checks.json").write_text(json.dumps(checks, indent=2), encoding="utf-8")
    md = to_markdown(checks)
    (out_dir / "data_checks.md").write_text(md, encoding="utf-8")
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
