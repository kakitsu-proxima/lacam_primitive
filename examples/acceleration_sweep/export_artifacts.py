#!/usr/bin/env python3
"""Export representative anytime trajectories and GIFs for every scenario."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        if not rows:
            return
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def choose_profiles(details: dict[str, Any]) -> dict[str, str]:
    """Choose one reproducible representative profile per scenario."""
    groups: dict[str, list[dict[str, Any]]] = {}
    for row in details["summary"]:
        if row["first_ms_median"] is not None:
            groups.setdefault(row["scenario"], []).append(row)
    selected: dict[str, str] = {}
    for scenario, rows in groups.items():
        exact = [
            row
            for row in rows
            if row["first_within_10ms"] and row["best_known_by_50ms"]
        ]
        if exact:
            choice = min(exact, key=lambda row: float(row["first_ms_median"]))
        else:
            quick = [row for row in rows if row["first_within_10ms"]]
            if quick:
                def quick_score(row: dict[str, Any]) -> tuple[float, float]:
                    at_50 = row["best_50_cost_median"]
                    known = row["best_known_cost"]
                    gap = (
                        float(at_50) - float(known)
                        if at_50 is not None and known is not None
                        else math.inf
                    )
                    return gap, float(row["first_ms_median"])

                choice = min(quick, key=quick_score)
            else:
                choice = min(rows, key=lambda row: float(row["first_ms_median"]))
        selected[scenario] = str(choice["profile"])
    return selected


def representative_run(
    details: dict[str, Any], scenario: str, profile: str
) -> tuple[int | None, dict[str, Any]]:
    if profile == "reference":
        row = next(
            row for row in details["reference_runs"] if row["scenario"] == scenario
        )
        return None, row
    rows = [
        row
        for row in details["runs"]
        if row["scenario"] == scenario
        and row["profile"] == profile
        and row["success"]
    ]
    if not rows:
        raise RuntimeError(f"no successful benchmark run for {scenario}/{profile}")
    first_values = sorted(float(row["first_ms"]) for row in rows)
    median = first_values[len(first_values) // 2]
    row = min(rows, key=lambda item: abs(float(item["first_ms"]) - median))
    return int(row["repetition"]), row


def solution_source(
    repo: Path, scenario: str, profile: str, repetition: int | None
) -> Path:
    root = repo / "outputs" / "acceleration_sweep" / "solutions"
    if profile == "reference":
        return root / "reference" / f"{scenario}.yaml"
    return root / profile / f"{scenario}_r{repetition}.yaml"


def waypoint_rows(
    variant: dict[str, Any], macro_dt: float, heading_bins: int
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for plan in variant["plans"]:
        agent = int(plan["agent"])
        primitive_ids = [int(value) for value in plan["primitive_ids"]]
        for step, state in enumerate(plan["states_m"]):
            heading = int(state[2])
            rows.append(
                {
                    "agent": agent,
                    "step": step,
                    "time_s": step * macro_dt,
                    "x_m": float(state[0]),
                    "y_m": float(state[1]),
                    "heading_bin": heading,
                    "yaw_rad": heading * 2.0 * math.pi / heading_bins,
                    "yaw_deg": heading * 360.0 / heading_bins,
                    "primitive_id_to_next": (
                        primitive_ids[step] if step < len(primitive_ids) else ""
                    ),
                }
            )
    return rows


def sampled_rows(
    problem: dict[str, Any],
    solution: dict[str, Any],
    variant: dict[str, Any],
    sample_dt_s: float,
    viz: Any,
) -> list[dict[str, Any]]:
    grid = problem["grid"]
    cell_size = float(grid["cell_size"])
    origin_x, origin_y = (float(value) for value in grid.get("origin", [0.0, 0.0]))
    heading_bins = int(grid["heading_bins"])
    macro_dt = float(grid["macro_dt"])
    primitive_catalog = viz._build_primitive_catalog(
        problem,
        multiple_rotation_amounts_enabled=solution.get("runtime", {}).get(
            "multiple_rotation_amounts_enabled"
        ),
    )
    max_steps = max(len(plan["states_m"]) - 1 for plan in variant["plans"])
    duration = max_steps * macro_dt
    sample_count = max(1, int(math.ceil(duration / sample_dt_s)))
    sample_times = np.linspace(0.0, duration, sample_count + 1)
    rows: list[dict[str, Any]] = []
    for plan in variant["plans"]:
        cell_states = [
            [
                (float(state[0]) - origin_x) / cell_size,
                (float(state[1]) - origin_y) / cell_size,
                int(state[2]),
            ]
            for state in plan["states_m"]
        ]
        sampled = viz._interpolate_plan(
            cell_states,
            primitive_ids=plan["primitive_ids"],
            primitive_catalog=primitive_catalog,
            heading_bins=heading_bins,
            macro_dt=macro_dt,
            sample_times=sample_times,
        )
        x = origin_x + cell_size * sampled[:, 0]
        y = origin_y + cell_size * sampled[:, 1]
        yaw = np.unwrap(sampled[:, 2])
        edge_order = 2 if len(sample_times) >= 3 else 1
        vx = np.gradient(x, sample_times, edge_order=edge_order)
        vy = np.gradient(y, sample_times, edge_order=edge_order)
        omega = np.gradient(yaw, sample_times, edge_order=edge_order)
        ax = np.gradient(vx, sample_times, edge_order=edge_order)
        ay = np.gradient(vy, sample_times, edge_order=edge_order)
        alpha = np.gradient(omega, sample_times, edge_order=edge_order)
        for index, time_s in enumerate(sample_times):
            rows.append(
                {
                    "agent": int(plan["agent"]),
                    "sample": index,
                    "time_s": float(time_s),
                    "x_m": float(x[index]),
                    "y_m": float(y[index]),
                    "yaw_rad_unwrapped": float(yaw[index]),
                    "yaw_deg_unwrapped": float(np.degrees(yaw[index])),
                    "fd_vx_mps": float(vx[index]),
                    "fd_vy_mps": float(vy[index]),
                    "fd_speed_mps": float(math.hypot(vx[index], vy[index])),
                    "fd_omega_radps": float(omega[index]),
                    "fd_ax_mps2": float(ax[index]),
                    "fd_ay_mps2": float(ay[index]),
                    "fd_acceleration_mps2": float(math.hypot(ax[index], ay[index])),
                    "fd_alpha_radps2": float(alpha[index]),
                }
            )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--sample-dt-s", type=float, default=0.05)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--playback-speed", type=float, default=1.0)
    parser.add_argument("--pixels-per-meter", type=float, default=240.0)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.sample_dt_s <= 0.0:
        raise SystemExit("--sample-dt-s must be positive")

    repo = args.repo.resolve()
    suite = Path(__file__).resolve().parent
    artifact_root = (args.output_dir or suite / "artifacts").resolve()
    sys.path.insert(0, str(repo / "python"))
    import lacam_primitive_viz as viz

    with (suite / "benchmark_details.json").open(encoding="utf-8") as stream:
        details = json.load(stream)
    preferred_profiles = choose_profiles(details)

    manifest: list[dict[str, Any]] = []
    pose_manifest: list[dict[str, Any]] = []
    for scenario, profile in sorted(preferred_profiles.items()):
        repetition, benchmark_row = representative_run(details, scenario, profile)
        source = solution_source(repo, scenario, profile, repetition)
        problem_file = suite / f"{scenario}.yaml"
        if not source.is_file():
            raise FileNotFoundError(source)
        with problem_file.open(encoding="utf-8") as stream:
            problem = yaml.safe_load(stream)
        pose_reference = problem["grid"]["pose_reference"]
        reference_cell_size = float(pose_reference["cell_size"])
        reference_origin_x, reference_origin_y = (
            float(value) for value in pose_reference.get("origin", [0.0, 0.0])
        )
        reference_heading_bins = int(pose_reference["heading_bins"])
        for agent, item in enumerate(problem["agents"]):
            start = item["start_ref"]
            goal = item["goal_ref"]
            pose_manifest.append(
                {
                    "scenario": scenario,
                    "agent": agent,
                    "moves": start != goal,
                    "start_x_m": round(
                        reference_origin_x + reference_cell_size * float(start[0]), 12
                    ),
                    "start_y_m": round(
                        reference_origin_y + reference_cell_size * float(start[1]), 12
                    ),
                    "start_heading_deg": 360.0
                    * int(start[2])
                    / reference_heading_bins,
                    "goal_x_m": round(
                        reference_origin_x + reference_cell_size * float(goal[0]), 12
                    ),
                    "goal_y_m": round(
                        reference_origin_y + reference_cell_size * float(goal[1]), 12
                    ),
                    "goal_heading_deg": 360.0
                    * int(goal[2])
                    / reference_heading_bins,
                }
            )
        with source.open(encoding="utf-8") as stream:
            solution = yaml.safe_load(stream)
        if not solution.get("success"):
            raise RuntimeError(f"selected solution is unsuccessful: {source}")

        destination = artifact_root / scenario
        destination.mkdir(parents=True, exist_ok=True)
        copied_problem = destination / "problem.yaml"
        copied_solution = destination / "solution.yaml"
        shutil.copy2(problem_file, copied_problem)
        shutil.copy2(source, copied_solution)
        improvements = solution.get("improvements", [])
        if not improvements or not all("plans" in item for item in improvements):
            raise RuntimeError(f"solution lacks saved incumbent plans: {source}")

        scenario_manifest: list[dict[str, Any]] = []
        for index, variant in enumerate(improvements):
            cost_text = str(variant["cost"]).replace(".", "p")
            prefix = f"solution_{index:02d}_cost_{cost_text}"
            waypoint_file = destination / f"{prefix}_waypoints.csv"
            samples_file = destination / f"{prefix}_samples.csv"
            smooth_gif = destination / f"{prefix}_smooth.gif"
            waypoint_gif = destination / f"{prefix}_waypoints.gif"
            write_csv(
                waypoint_file,
                waypoint_rows(
                    variant,
                    float(problem["grid"]["macro_dt"]),
                    int(problem["grid"]["heading_bins"]),
                ),
            )
            write_csv(
                samples_file,
                sampled_rows(problem, solution, variant, args.sample_dt_s, viz),
            )
            viz.save_gif(
                copied_problem,
                copied_solution,
                smooth_gif,
                fps=args.fps,
                playback_speed=args.playback_speed,
                pixels_per_meter=args.pixels_per_meter,
                waypoint_only=False,
                solution_index=index,
            )
            viz.save_gif(
                copied_problem,
                copied_solution,
                waypoint_gif,
                fps=args.fps,
                playback_speed=args.playback_speed,
                pixels_per_meter=args.pixels_per_meter,
                waypoint_only=True,
                solution_index=index,
            )
            item = {
                "scenario": scenario,
                "profile": profile,
                "benchmark_repetition": repetition,
                "solution_index": index,
                "cost": float(variant["cost"]),
                "found_at_search_ms": float(variant["elapsed_ms"]),
                "weight": float(variant["weight"]),
                "waypoints_csv": waypoint_file.relative_to(artifact_root).as_posix(),
                "samples_csv": samples_file.relative_to(artifact_root).as_posix(),
                "smooth_gif": smooth_gif.relative_to(artifact_root).as_posix(),
                "waypoints_gif": waypoint_gif.relative_to(artifact_root).as_posix(),
            }
            scenario_manifest.append(item)
            manifest.append(item)
            print(
                f"{scenario} solution {index}: cost={variant['cost']} "
                f"at {variant['elapsed_ms']} ms",
                flush=True,
            )
        with (destination / "manifest.json").open("w", encoding="utf-8") as stream:
            json.dump(
                {
                    "scenario": scenario,
                    "profile": profile,
                    "benchmark_repetition": repetition,
                    "benchmark_metrics": benchmark_row,
                    "sample_dt_s": args.sample_dt_s,
                    "solutions": scenario_manifest,
                },
                stream,
                ensure_ascii=False,
                indent=2,
            )

    write_csv(artifact_root / "manifest.csv", manifest)
    write_csv(artifact_root / "scenario_poses.csv", pose_manifest)
    with (artifact_root / "manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
