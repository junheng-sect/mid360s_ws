#!/usr/bin/env python3
"""Compare timestamp-aligned replay results in the prior map; no ground truth is assumed."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def read_csv(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.array([int(row[key]) if key == "stamp_ns" else float(row[key]) for row in rows])
            for key in rows[0]} if rows else {}


def xyz(data):
    return np.column_stack([data[k] for k in ("x", "y", "z")])


def quantile(values):
    return {key: float(np.percentile(values, rank)) for key, rank in
            (("p50", 50), ("p95", 95), ("p99", 99), ("max", 100))} if len(values) else None


def summarize(folder, anchor):
    raw = read_csv(folder / "input.csv")
    pose = read_csv(folder / "pose.csv")
    resources = read_csv(folder / "resources.csv")
    diag = [json.loads(line) for line in (folder / "diagnostics.jsonl").read_text().splitlines()]
    tracking = {d["stamp_ns"]: d for d in diag if d["state"] == "TRACKING"}
    terminal = next((d for d in diag if d["state"] in ("STALE", "FAILED")), None)
    sensor_stamps = raw["stamp_ns"]
    output_stamps = pose.get("stamp_ns", np.array([], dtype=np.int64))
    idx = np.searchsorted(sensor_stamps, output_stamps - 8_000_000, side="right") - 1
    idx = np.clip(idx, 0, len(sensor_stamps) - 1)
    keep = ((output_stamps >= anchor) & (output_stamps >= sensor_stamps[idx]) &
            (output_stamps - sensor_stamps[idx] < 130_000_000))
    expected = int((sensor_stamps >= anchor).sum())
    latency = (pose.get("wall_s", np.array([])) - raw["wall_s"][idx]) * 1000
    resource_wall = resources["wall_s"]
    cpu_total = resources["cpu_s"][-1] - resources["cpu_s"][0]
    result = dict(
        raw_scans=len(sensor_stamps),
        publication_stamp_monotonic=bool(np.all(np.diff(output_stamps) > 0)), odom_count=len(output_stamps),
        expected_scans_after_anchor=expected,
        published_scans_after_anchor=len(np.unique(idx[keep & (sensor_stamps[idx] >= anchor)])),
        first_valid_s=float((output_stamps[0] - sensor_stamps[0]) / 1e9) if len(output_stamps) else None,
        last_valid_s=float((output_stamps[-1] - sensor_stamps[0]) / 1e9) if len(output_stamps) else None,
        end_to_end_ms=quantile(latency[keep]),
        cpu_one_core_pct=100 * cpu_total / (resource_wall[-1] - resource_wall[0]),
        peak_rss_mib=float(resources["rss_mib"].max()),
    )
    if diag:
        frames = list(tracking.values())
        result.update(
            observed_tracking_scans=len(frames),
            scan_processing_ms=quantile([float(d["processing_ms"]) for d in frames]),
            queue_to_process_ms=quantile([float(d["receive_to_process_ms"]) for d in frames]),
            callback_to_publish_ms=quantile([float(d["receive_to_process_ms"]) + float(d["processing_ms"])
                                             for d in frames]),
            lidar_interval_ms=quantile([float(d["lidar_interval_ms"]) for d in frames]),
            residual_m=quantile([float(d["residual_mean_m"]) for d in frames]),
            match_fraction=quantile([float(d["match_fraction"]) for d in frames]),
            correction_m=quantile([float(d["update_m"]) for d in frames]),
            correction_deg=quantile([np.degrees(float(d["update_rad"])) for d in frames]),
            max_queue_depth=max((int(d["queue_depth"]) for d in frames), default=None),
            fixed_map_point_counts=sorted({int(d["map_points"]) for d in frames}),
            terminal={k: terminal[k] for k in ("state", "reason", "stamp_ns", "wall_s")} if terminal else None,
        )
    return result, pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--ywl", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    seed = json.loads(args.seed.read_text())
    anchor = seed["anchor_ns"]
    baseline, base_pose = summarize(args.baseline, anchor)
    ywl, ywl_pose = summarize(args.ywl, anchor)
    report = dict(anchor_ns=anchor, map_sha256=seed["map_sha256"], baseline=baseline, ywl=ywl)
    if ywl_pose and base_pose:
        stamps = ywl_pose["stamp_ns"]
        base_stamps = base_pose["stamp_ns"]
        left = np.clip(np.searchsorted(base_stamps, stamps) - 1, 0, len(base_stamps) - 1)
        right = np.clip(left + 1, 0, len(base_stamps) - 1)
        idx = np.where(abs(base_stamps[left] - stamps) <= abs(base_stamps[right] - stamps), left, right)
        keep = np.abs(stamps - base_stamps[idx]) <= 2_000_000
        transform = seed["map_from_camera"]
        base_rotation = Rotation.from_quat(transform["q_xyzw"])
        base_xyz = base_rotation.apply(xyz(base_pose)[idx[keep]]) + transform["translation"]
        ywl_xyz = xyz(ywl_pose)[keep]
        q = lambda rows: Rotation.from_quat(np.column_stack([rows[k] for k in ("qx", "qy", "qz", "qw")]))
        if keep.any():
            relative = (base_rotation * q(base_pose)[idx[keep]]).inv() * q(ywl_pose)[keep]
            from_seed_s = (stamps[keep] - anchor) / 1e9
            report.update(common_scans=int(keep.sum()),
                          position_difference_m=quantile(np.linalg.norm(ywl_xyz - base_xyz, axis=1)),
                          orientation_difference_deg=quantile(np.degrees(relative.magnitude())))
            if from_seed_s[-1] - from_seed_s[0] >= 20:
                start = from_seed_s <= from_seed_s[0] + 0.6
                end = from_seed_s >= from_seed_s[-1] - 3
                report["endpoint_displacement_m"] = {
                    "baseline": float(np.linalg.norm(base_xyz[end].mean(0) - base_xyz[start].mean(0))),
                    "ywl": float(np.linalg.norm(ywl_xyz[end].mean(0) - ywl_xyz[start].mean(0)))}
            with args.out.with_name("paired_poses.csv").open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(("stamp_ns", "baseline_x", "baseline_y", "baseline_z", "ywl_x", "ywl_y", "ywl_z"))
                writer.writerows((n, *b, *y) for n, b, y in zip(stamps[keep], base_xyz, ywl_xyz))
    args.out.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
