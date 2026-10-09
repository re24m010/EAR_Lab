"""Phase C.2: technical visualisations of the extracted data.

Input:  results/data, results/checks (run extract_data.py and check_data.py first)
Output: plots/data_preparation/*.png

Usage:  python src/plot_data.py [--config config/dataset.json]
"""

from __future__ import annotations

import argparse
import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from dataset import (  # noqa: E402
    gt_base_and_laser_poses, gt_pose_for_scans, gt_se2, load_poses, load_scans,
    load_tf_static, lookup_static, poses_to_world_2d, scan_points_laser,
)
from project_config import DEFAULT_CONFIG, load_config, project_path  # noqa: E402
from transforms import se2_compose  # noqa: E402

# Reference palette (dataviz skill, light mode), fixed slot order.
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white", "savefig.facecolor": "white",
    "font.family": ["Segoe UI", "DejaVu Sans", "sans-serif"], "font.size": 9,
    "text.color": INK, "axes.labelcolor": INK2, "axes.titlecolor": INK, "axes.titlesize": 10,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 0.6, "grid.linestyle": "-",
    "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
    "axes.spines.top": False, "axes.spines.right": False,
    "legend.frameon": False, "lines.linewidth": 1.5, "savefig.dpi": 200,
})


def save(fig, path):
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {path.name}")


def world_points(scans, idx_list, laser_poses, gt_idx):
    valid = scans.valid_mask()
    out = []
    for k in idx_list:
        pts = scan_points_laser(scans.ranges[k], scans.angles, valid[k])[:, :2]
        out.append(poses_to_world_2d(pts, laser_poses[gt_idx[k]]))
    return np.concatenate(out)


def density_image(ax, pts, extent, res=0.05):
    (x0, x1), (y0, y1) = extent
    h, _, _ = np.histogram2d(pts[:, 0], pts[:, 1], bins=[np.arange(x0, x1 + res, res),
                                                          np.arange(y0, y1 + res, res)])
    ax.imshow(np.log1p(h.T), origin="lower", extent=(x0, x1, y0, y1), cmap="Greys",
              interpolation="nearest", vmin=0, vmax=np.log1p(np.percentile(h[h > 0], 95)))


# ---------------------------------------------------------------- figures

def fig_scan_examples(scans, checks, out):
    valid = scans.valid_mask()
    per_scan = valid.mean(axis=1)
    t_rel = scans.t - scans.t[0]
    picks = sorted({0, int(np.argmin(per_scan)), len(per_scan) // 2, int(np.argmax(t_rel > 45.0))})
    fig, axes = plt.subplots(2, 2, figsize=(8.0, 6.6))
    lim = 20.0
    for ax, k in zip(axes.ravel(), picks):
        p = scan_points_laser(scans.ranges[k], scans.angles, valid[k])
        ax.plot([0, lim * np.cos(scans.angles[0])], [0, lim * np.sin(scans.angles[0])], color=AXIS, lw=0.8)
        ax.plot([0, lim * np.cos(scans.angles[-1])], [0, lim * np.sin(scans.angles[-1])], color=AXIS, lw=0.8)
        ax.scatter(p[:, 0], p[:, 1], s=1.5, color=BLUE, linewidths=0, rasterized=True)
        ax.plot(0, 0, marker=">", ms=8, color=INK)
        ax.set_title(f"Scan {k}  (t = {t_rel[k]:.2f} s)  -  {valid[k].sum()}/{scans.ranges.shape[1]} gültig")
        ax.set_aspect("equal")
        ax.set_xlim(-2, lim)
        ax.set_ylim(-lim / 1.6, lim / 1.6)
        ax.set_xlabel("x_laser [m]")
        ax.set_ylabel("y_laser [m]")
    sc = checks["scans"]
    fig.suptitle(f"2D-Laserscans im Laser-Frame: {sc['num_beams']} Strahlen, "
                 f"{sc['angle_min_deg']:.2f}° bis {sc['angle_max_deg']:.2f}° "
                 f"(Schritt {sc['angle_increment_deg']:.2f}°)", color=INK)
    fig.text(0.5, 0.005, "Graue Linien: erster und letzter Strahl. Pfeil: Sensorursprung, x nach vorn. "
             "Ausschnitt bis 20 m.", ha="center", color=INK2, fontsize=8)
    fig.tight_layout(rect=(0, 0.02, 1, 0.97))
    save(fig, out / "scan_examples.png")


def fig_scan_profile(scans, out):
    valid = scans.valid_mask()
    ang = np.degrees(scans.angles)
    k = len(scans.ranges) // 2
    r = np.where(valid[k], scans.ranges[k], np.nan)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8.0, 5.0), sharex=True)
    a1.plot(ang, r, color=BLUE, lw=1.0)
    a1.set_ylabel("Distanz [m]")
    a1.set_title(f"Entfernungsprofil von Scan {k} (Lücken = ungültige Strahlen, inf)")
    a2.plot(ang, valid.mean(axis=0) * 100, color=BLUE, lw=1.0)
    a2.set_ylabel("gültig über alle Scans [%]")
    a2.set_xlabel("Strahlwinkel im Laser-Frame [°]  (0° = vorn, + = links)")
    a2.set_title("Anteil gültiger Messungen je Strahl")
    a2.set_xlim(-92, 92)
    a2.set_xticks(np.arange(-90, 91, 30))
    fig.tight_layout()
    save(fig, out / "scan_range_profile.png")


def fig_scan_quality(scans, out):
    valid = scans.valid_mask()
    t_rel = scans.t - scans.t[0]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.0, 3.2), gridspec_kw={"width_ratios": [1.6, 1]})
    a1.plot(t_rel, valid.mean(axis=1) * 100, color=BLUE, lw=1.0)
    a1.set_xlabel("Zeit seit Sequenzstart [s]")
    a1.set_ylabel("gültige Strahlen [%]")
    a1.set_title("Gültige Strahlen pro Scan")
    a2.hist(scans.ranges[valid], bins=np.arange(0, 40.5, 0.5), color=BLUE, edgecolor="white", linewidth=0.5)
    a2.set_xlabel("Distanz [m]")
    a2.set_ylabel("Anzahl Messungen")
    a2.set_title("Verteilung gültiger Distanzen")
    fig.tight_layout()
    save(fig, out / "scan_quality.png")


def fig_timestamps(scans, odom, gt, out):
    streams = [("/scan", np.diff(scans.stamp_ns)), ("/odom", np.diff(odom["stamp_ns"].to_numpy())),
               ("/gt", np.diff(gt["stamp_ns"].to_numpy()))]
    fig, axes = plt.subplots(1, 3, figsize=(9.0, 2.8))
    for ax, (name, d) in zip(axes, streams):
        ax.hist(d * 1e-6, bins=40, color=BLUE, edgecolor="white", linewidth=0.5)
        ax.set_yscale("log")
        ax.set_title(f"{name}: Δt (n = {len(d)})")
        ax.set_xlabel("Δt [ms]")
    axes[0].set_ylabel("Anzahl (log)")
    fig.tight_layout()
    save(fig, out / "timestamp_intervals.png")


def fig_trajectory(aligned, checks, out):
    al = checks["odom_alignment"]
    t = aligned["stamp_ns"].to_numpy() * 1e-9
    t -= t[0]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10.0, 5.6), gridspec_kw={"width_ratios": [1, 1.25]})
    a1.plot(aligned.gt_x, aligned.gt_y, color=BLUE, lw=2.0,
            label=f"Ground Truth, base_link (/gt als {al['gt_interpretation']}-Pose interpretiert)")
    a1.plot(aligned.odom_init_x, aligned.odom_init_y, color=ORANGE, lw=1.5,
            label="Radodometrie, Startpose ausgerichtet")
    a1.plot(aligned.odom_se2_x, aligned.odom_se2_y, color=AQUA, lw=1.5,
            label="Radodometrie, SE(2)-Least-Squares")
    a1.plot(aligned.gt_x.iloc[0], aligned.gt_y.iloc[0], "o", ms=8, color=INK, mec="white", mew=1.5)
    a1.plot(aligned.gt_x.iloc[-1], aligned.gt_y.iloc[-1], "s", ms=8, color=INK, mec="white", mew=1.5)
    a1.annotate("Start", (aligned.gt_x.iloc[0], aligned.gt_y.iloc[0]), xytext=(8, 0),
                textcoords="offset points", color=INK2, va="center")
    a1.annotate("Ende", (aligned.gt_x.iloc[-1], aligned.gt_y.iloc[-1]), xytext=(8, 0),
                textcoords="offset points", color=INK2, va="center")
    a1.set_aspect("equal")
    a1.set_xlabel("x [m] (gt_map)")
    a1.set_ylabel("y [m] (gt_map)")
    a1.set_title("Trajektorie in der Ebene")
    a1.legend(loc="upper left", bbox_to_anchor=(0, -0.12), fontsize=8)

    e_init = np.hypot(aligned.odom_init_x - aligned.gt_x, aligned.odom_init_y - aligned.gt_y)
    e_ls = np.hypot(aligned.odom_se2_x - aligned.gt_x, aligned.odom_se2_y - aligned.gt_y)
    a2.plot(t, e_init, color=ORANGE, label=f"Startpose ausgerichtet (RMSE {al['initial_pose_alignment']['pos_rmse_m']:.2f} m)")
    a2.plot(t, e_ls, color=AQUA, label=f"SE(2)-Least-Squares (RMSE {al['se2_least_squares_alignment']['pos_rmse_m']:.2f} m)")
    a2.set_xlabel("Zeit seit Sequenzstart [s]")
    a2.set_ylabel("Positionsabweichung zur GT [m]")
    a2.set_title("Abweichung der Radodometrie (keine SLAM-Ergebnisse)")
    a2.legend(loc="upper left", fontsize=8)
    a2.set_ylim(bottom=0)
    fig.tight_layout()
    save(fig, out / "trajectory_gt_vs_odometry.png")


def fig_gt_registered(scans, gt, tf_static, cfg, checks, out):
    ext = lookup_static(tf_static, cfg["frames"]["base"], cfg["frames"]["laser"]).se2()
    gt_idx, has = gt_pose_for_scans(scans, gt)
    ks = np.flatnonzero(has)[::cfg["checks"]["consistency_scan_stride"]]
    raw = gt_se2(gt)
    variants = {
        "A: /gt = base_link (wie deklariert) + Extrinsik": se2_compose(raw, np.broadcast_to(ext, raw.shape)),
        "B: /gt = Laser-Frame (ohne Extrinsik)": raw,
    }
    _, laser_poses = gt_base_and_laser_poses(gt, ext, cfg["ground_truth"]["child_frame_interpretation"])
    pts_all = world_points(scans, ks, laser_poses, gt_idx)
    # Full map under the configured interpretation.
    fig, ax = plt.subplots(figsize=(6.4, 8.0))
    margin = 8.0  # show structure up to 8 m beyond the driven path
    density_image(ax, pts_all, ((raw[:, 0].min() - margin, raw[:, 0].max() + margin),
                                (raw[:, 1].min() - margin, raw[:, 1].max() + margin)))
    ax.plot(raw[:, 0], raw[:, 1], color=BLUE, lw=1.2, label="GT-Trajektorie")
    ax.set_aspect("equal")
    ax.grid(False)
    ax.set_xlabel("x [m] (gt_map)")
    ax.set_ylabel("y [m] (gt_map)")
    ax.set_title("Konsistenzprüfung: Scans mit GT-Posen registriert\n"
                 f"(Interpretation: /gt = {cfg['ground_truth']['child_frame_interpretation']}; "
                 "kein SLAM-Ergebnis)")
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    save(fig, out / "gt_registered_scans_map.png")

    # Zoom on the turning segment, both hypotheses side by side.
    t_rel = gt["t"].to_numpy() - gt["t"].iloc[0]
    w = np.abs(np.gradient(np.unwrap(raw[:, 2]), t_rel))
    c = raw[int(np.argmax(np.convolve(w, np.ones(40) / 40, mode="same"))), :2]
    half = 6.0
    zoom = ((c[0] - half, c[0] + half), (c[1] - half, c[1] + half))
    rs = checks["registration_sharpness"]["occupied_cells"]
    cells = [rs["A_extrinsic_from_tf_static"], rs["B_no_extrinsic_laser_at_base"]]
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 5.0))
    for ax, (name, lp), n in zip(axes, variants.items(), cells):
        density_image(ax, world_points(scans, ks, lp, gt_idx), zoom, res=0.03)
        ax.set_title(f"{name}\n{n} belegte 5-cm-Zellen (gesamte Sequenz)", fontsize=9)
        ax.set_aspect("equal")
        ax.grid(False)
        ax.set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    fig.suptitle("Ausschnitt mit der stärksten Drehung: welche Frame-Interpretation ergibt die schärfere Karte?",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save(fig, out / "gt_frame_hypotheses_zoom.png")


def fig_revisits(gt, checks, out):
    cfg = load_config()
    cfg_checks = cfg["checks"]
    rv = np.load(project_path(cfg["output"]["checks_dir"]) / "revisit.npz")
    near = rv["nearest_m"]
    xy = rv["xy"]
    t = rv["t_rel"]
    revisit = near <= 0.5
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10.0, 5.2), gridspec_kw={"width_ratios": [1, 1.4]})
    a1.plot(xy[:, 0], xy[:, 1], color=AXIS, lw=2.5, label="GT-Trajektorie")
    a1.scatter(xy[revisit, 0], xy[revisit, 1], s=10, color=ORANGE, linewidths=0, zorder=3,
               label="Pose mit Revisit ≤ 0,5 m")
    a1.plot(xy[0, 0], xy[0, 1], "o", ms=8, color=INK, mec="white", mew=1.5)
    a1.plot(xy[-1, 0], xy[-1, 1], "s", ms=8, color=INK, mec="white", mew=1.5)
    a1.annotate("Start", xy[0], xytext=(8, 0), textcoords="offset points", color=INK2, va="center")
    a1.annotate("Ende", xy[-1], xytext=(8, 0), textcoords="offset points", color=INK2, va="center")
    a1.set_aspect("equal")
    a1.set_xlabel("x [m]")
    a1.set_ylabel("y [m]")
    a1.set_title("Wiederbefahrene Abschnitte")
    a1.legend(loc="upper left", bbox_to_anchor=(0, -0.12), fontsize=8)
    a2.plot(t, np.minimum(near, 30), color=BLUE, lw=1.2)
    a2.axhline(0.5, color=ORANGE, lw=1.0)
    a2.annotate("0,5 m", (t[-1], 0.5), xytext=(4, 0), textcoords="offset points", color=INK2, va="center")
    a2.set_yscale("log")
    a2.set_xlabel("Zeit seit Sequenzstart [s]")
    a2.set_ylabel("Abstand zur nächsten früheren/späteren Pose [m]")
    a2.set_title("Abstand zur nächsten zeitlich entfernten Pose\n"
                 f"(Δt ≥ {cfg_checks['revisit_min_time_separation_s']:g} s und Δs ≥ "
                 f"{cfg_checks['revisit_min_path_separation_m']:g} m entlang des Pfades)", fontsize=9)
    fig.tight_layout()
    save(fig, out / "revisit_analysis.png")


def fig_lag(checks, out):
    rv = np.load(project_path(load_config()["output"]["checks_dir"]) / "revisit.npz")
    lag = checks["odom_gt_lag"]["yaw_rate"]
    fig, ax = plt.subplots(figsize=(6.0, 3.0))
    ax.plot(rv["yaw_rate_lags"] * 1e3, rv["yaw_rate_corr"], color=BLUE)
    ax.axvline(lag["best_lag_ms"], color=AXIS, lw=1.0)
    ax.annotate(f"Maximum bei {lag['best_lag_ms']:.0f} ms (r = {lag['corr_at_best']:.4f})",
                (lag["best_lag_ms"], lag["corr_at_best"]), xytext=(10, -14), textcoords="offset points",
                color=INK2, fontsize=8)
    ax.set_xlabel("Zeitversatz τ [ms]:  ω_odom(t) vs. ω_GT(t + τ)")
    ax.set_ylabel("Korrelation r")
    ax.set_title("Zeitversatz Radodometrie ↔ GT (Gierrate)")
    fig.tight_layout()
    save(fig, out / "odom_gt_time_lag.png")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    args = ap.parse_args()
    cfg = load_config(args.config)
    data_dir = project_path(cfg["output"]["data_dir"])
    out = project_path(cfg["output"]["plots_dir"])
    out.mkdir(parents=True, exist_ok=True)
    checks = json.loads((project_path(cfg["output"]["checks_dir"]) / "data_checks.json").read_text(encoding="utf-8"))

    import pandas as pd
    scans = load_scans(data_dir)
    odom = load_poses(data_dir, "odom")
    gt = load_poses(data_dir, "gt")
    tf_static = load_tf_static(data_dir)
    aligned = pd.read_csv(data_dir / "odom_aligned.csv")

    fig_scan_examples(scans, checks, out)
    fig_scan_profile(scans, out)
    fig_scan_quality(scans, out)
    fig_timestamps(scans, odom, gt, out)
    fig_trajectory(aligned, checks, out)
    fig_gt_registered(scans, gt, tf_static, cfg, checks, out)
    fig_revisits(gt, checks, out)
    fig_lag(checks, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
