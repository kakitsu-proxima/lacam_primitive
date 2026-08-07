from __future__ import annotations

import math
import subprocess
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

COLORS = (
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
)


def run_planner(
    repo_root: str | Path,
    problem_file: str | Path,
    output_file: str | Path,
    *,
    time_limit_ms: float | None = None,
    transition_cache: bool | None = None,
    candidate_cache: bool | None = None,
    ara_star: bool | None = None,
) -> Path:
    repo_root = Path(repo_root).resolve()
    executable = repo_root / "build" / "lacam_primitive"
    if not executable.is_file():
        raise FileNotFoundError(
            f"{executable} does not exist. Run: cmake -S . -B build && cmake --build build"
        )

    output_file = Path(output_file).resolve()
    output_file.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(executable),
        "--input",
        str(Path(problem_file).resolve()),
        "--output",
        str(output_file),
    ]
    if time_limit_ms is not None:
        command.extend(["--time-limit-ms", str(float(time_limit_ms))])
    if transition_cache is not None:
        command.extend(["--transition-cache", "on" if transition_cache else "off"])
    if candidate_cache is not None:
        command.extend(["--candidate-cache", "on" if candidate_cache else "off"])
    if ara_star is not None:
        command.extend(["--ara-star", "on" if ara_star else "off"])

    completed = subprocess.run(
        command,
        cwd=repo_root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    print(completed.stdout)
    if completed.returncode != 0:
        raise RuntimeError(
            "planner failed\n"
            f"exit code: {completed.returncode}\n"
            f"command: {' '.join(command)}\n"
            f"output:\n{completed.stdout}"
        )
    return output_file


def _font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default()


def _shortest_heading_delta(start: float, goal: float) -> float:
    return math.remainder(goal - start, 2.0 * math.pi)


def _build_primitive_catalog(problem: dict[str, Any]) -> list[dict[str, Any]]:
    """
    Reconstruct primitive IDs in exactly the same order as the C++ PrimitiveTable.

    The C++ order is:

      wait
      east/west/north/south for each sorted translation distance
      rotate_ccw / rotate_cw for each sorted pivot offset
    """
    primitive_cfg = problem["primitives"]

    translation_cells = sorted(
        set(int(v) for v in primitive_cfg["translation_cells"])
    )

    rotation_bins = int(primitive_cfg["rotation_bins"])

    pivot_offsets = sorted(
        set(
            int(v)
            for v in primitive_cfg.get(
                "rotation_pivot_offsets_cells",
                [0],
            )
        )
    )

    include_wait = bool(primitive_cfg.get("include_wait", True))

    catalog: list[dict[str, Any]] = []

    if include_wait:
        catalog.append(
            {
                "name": "wait",
                "type": "wait",
                "dx": 0,
                "dy": 0,
                "d_heading": 0,
                "pivot_offset_cells": 0,
            }
        )

    for distance in translation_cells:
        catalog.append(
            {
                "name": f"east_{distance}",
                "type": "translation",
                "dx": distance,
                "dy": 0,
                "d_heading": 0,
                "pivot_offset_cells": 0,
            }
        )

        catalog.append(
            {
                "name": f"west_{distance}",
                "type": "translation",
                "dx": -distance,
                "dy": 0,
                "d_heading": 0,
                "pivot_offset_cells": 0,
            }
        )

        catalog.append(
            {
                "name": f"north_{distance}",
                "type": "translation",
                "dx": 0,
                "dy": distance,
                "d_heading": 0,
                "pivot_offset_cells": 0,
            }
        )

        catalog.append(
            {
                "name": f"south_{distance}",
                "type": "translation",
                "dx": 0,
                "dy": -distance,
                "d_heading": 0,
                "pivot_offset_cells": 0,
            }
        )

    for pivot_offset in pivot_offsets:
        suffix = (
            "center"
            if pivot_offset == 0
            else (
                f"front_{pivot_offset}"
                if pivot_offset > 0
                else f"rear_{-pivot_offset}"
            )
        )

        catalog.append(
            {
                "name": f"rotate_ccw_{suffix}",
                "type": "pivot_rotation",
                "dx": 0,
                "dy": 0,
                "d_heading": rotation_bins,
                "pivot_offset_cells": pivot_offset,
            }
        )

        catalog.append(
            {
                "name": f"rotate_cw_{suffix}",
                "type": "pivot_rotation",
                "dx": 0,
                "dy": 0,
                "d_heading": -rotation_bins,
                "pivot_offset_cells": pivot_offset,
            }
        )

    return catalog


def _sample_primitive_segment(
    start: np.ndarray,
    goal: np.ndarray,
    primitive: dict[str, Any],
    alpha: float,
    heading_bins: int,
) -> np.ndarray:
    """
    Sample one primitive at alpha in [0, 1].

    start and goal are:
        [x_cell, y_cell, heading_bin]

    Return:
        [x_cell_float, y_cell_float, yaw_rad]
    """
    heading_step = 2.0 * math.pi / heading_bins

    start_yaw = float(start[2]) * heading_step

    primitive_type = primitive["type"]

    # ------------------------------------------------------------
    # Pivot rotation
    # ------------------------------------------------------------
    if primitive_type == "pivot_rotation":
        d_heading = int(primitive["d_heading"])
        yaw_delta = float(d_heading) * heading_step

        yaw = start_yaw + alpha * yaw_delta

        pivot_offset = float(
            primitive["pivot_offset_cells"]
        )

        # Pivot is fixed in world coordinates during this primitive.
        #
        # Body +x axis:
        #   [cos(start_yaw), sin(start_yaw)]
        #
        # Absolute pivot position:
        pivot_x = (
            float(start[0])
            + pivot_offset * math.cos(start_yaw)
        )

        pivot_y = (
            float(start[1])
            + pivot_offset * math.sin(start_yaw)
        )

        # Rectangle center moves on a circular arc around the pivot.
        x = (
            pivot_x
            - pivot_offset * math.cos(yaw)
        )

        y = (
            pivot_y
            - pivot_offset * math.sin(yaw)
        )

        return np.array(
            [x, y, yaw],
            dtype=float,
        )

    # ------------------------------------------------------------
    # Wait or global-axis translation
    # ------------------------------------------------------------

    goal_yaw = float(goal[2]) * heading_step

    yaw = (
        start_yaw
        + alpha
        * _shortest_heading_delta(
            start_yaw,
            goal_yaw,
        )
    )

    x = (
        (1.0 - alpha) * float(start[0])
        + alpha * float(goal[0])
    )

    y = (
        (1.0 - alpha) * float(start[1])
        + alpha * float(goal[1])
    )

    return np.array(
        [x, y, yaw],
        dtype=float,
    )

def _active_primitive_at_time(
    plan: dict[str, Any],
    time_value: float,
    macro_dt: float,
    primitive_catalog: list[dict[str, Any]],
) -> tuple[int, dict[str, Any]] | None:
    primitive_ids = plan["primitive_ids"]

    if not primitive_ids:
        return None

    max_time = (
        len(primitive_ids) * macro_dt
    )

    if time_value >= max_time:
        return None

    segment = int(
        math.floor(
            max(time_value, 0.0)
            / macro_dt
        )
    )

    segment = min(
        segment,
        len(primitive_ids) - 1,
    )

    primitive_id = int(
        primitive_ids[segment]
    )

    return (
        segment,
        primitive_catalog[
            primitive_id
        ],
    )


def _interpolate_plan(
    states: list[list[int]],
    primitive_ids: list[int],
    primitive_catalog: list[dict[str, Any]],
    heading_bins: int,
    macro_dt: float,
    sample_times: np.ndarray,
) -> np.ndarray:
    values = np.asarray(states, dtype=float)

    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError(
            "states must be "
            "[[x_cell, y_cell, heading_bin], ...]"
        )

    if len(values) == 0:
        raise ValueError(
            "states must not be empty"
        )

    if len(primitive_ids) != max(0, len(values) - 1):
        raise ValueError(
            "primitive_ids length must equal "
            "len(states) - 1"
        )

    output = np.empty(
        (len(sample_times), 3),
        dtype=float,
    )

    heading_step = (
        2.0 * math.pi / heading_bins
    )

    max_time = (
        (len(values) - 1) * macro_dt
    )

    # Single-state plan.
    if len(values) == 1:
        yaw = values[0, 2] * heading_step

        output[:, 0] = values[0, 0]
        output[:, 1] = values[0, 1]
        output[:, 2] = yaw

        return output

    for index, time_value in enumerate(
        sample_times
    ):
        clamped = min(
            max(float(time_value), 0.0),
            max_time,
        )

        # Force the final sample exactly onto the final state.
        if clamped >= max_time - 1e-12:
            final = values[-1]

            output[index, 0] = final[0]
            output[index, 1] = final[1]
            output[index, 2] = (
                final[2] * heading_step
            )

            continue

        segment = int(
            math.floor(
                clamped / macro_dt
            )
        )

        segment = max(
            0,
            min(
                segment,
                len(values) - 2,
            ),
        )

        segment_start_time = (
            segment * macro_dt
        )

        alpha = (
            clamped - segment_start_time
        ) / macro_dt

        alpha = min(
            max(alpha, 0.0),
            1.0,
        )

        primitive_id = int(
            primitive_ids[segment]
        )

        if (
            primitive_id < 0
            or primitive_id
            >= len(primitive_catalog)
        ):
            raise ValueError(
                f"invalid primitive id "
                f"{primitive_id}; "
                f"catalog size="
                f"{len(primitive_catalog)}"
            )

        primitive = (
            primitive_catalog[
                primitive_id
            ]
        )

        output[index] = (
            _sample_primitive_segment(
                start=values[segment],
                goal=values[segment + 1],
                primitive=primitive,
                alpha=alpha,
                heading_bins=heading_bins,
            )
        )

    return output


def save_gif(
    problem_file: str | Path,
    solution_file: str | Path,
    output_file: str | Path,
    *,
    fps: int = 20,
    playback_speed: float = 1.0,
    pixels_per_cell: int = 24,
    start_hold_seconds: float = 0.6,
    goal_hold_seconds: float = 1.2,
    waypoint_only: bool = False,
) -> Path:
    if fps <= 0 or playback_speed <= 0 or pixels_per_cell <= 0:
        raise ValueError("fps, playback_speed, and pixels_per_cell must be positive")

    with Path(problem_file).open(encoding="utf-8") as stream:
        problem: dict[str, Any] = yaml.safe_load(stream)
    with Path(solution_file).open(encoding="utf-8") as stream:
        solution: dict[str, Any] = yaml.safe_load(stream)

    if not solution.get("success"):
        raise ValueError("solution file does not contain a successful plan")

    grid = problem["grid"]
    robot = problem["robot"]
    width = int(grid["width_cells"])
    height = int(grid["height_cells"])
    cell_size = float(grid["cell_size"])
    heading_bins = int(grid["heading_bins"])
    macro_dt = float(grid["macro_dt"])
    robot_length_cells = float(robot["size"][0]) / cell_size
    robot_width_cells = float(robot["size"][1]) / cell_size

    plans = solution["plans"]
    max_steps = max(len(plan["states"]) - 1 for plan in plans)
    max_duration = max_steps * macro_dt
    if waypoint_only:
        # One frame per planner waypoint: 0, macro_dt, 2*macro_dt, ...
        sample_times = np.arange(max_steps + 1, dtype=float) * macro_dt
    elif max_duration <= 0.0:
        sample_times = np.array([0.0], dtype=float)
    else:
        # Smoothly interpolate between planner waypoints. linspace is used so
        # the final displayed simulation time is exactly max_duration.
        target_sim_dt = playback_speed / fps
        frame_count = max(1, int(math.ceil(max_duration / target_sim_dt)))
        sample_times = np.linspace(0.0, max_duration, frame_count + 1)
    primitive_catalog = (_build_primitive_catalog(problem))

    sampled = [
        _interpolate_plan(plan["states"], 
                          primitive_ids=plan["primitive_ids"],
                          primitive_catalog=primitive_catalog,
                          heading_bins=heading_bins,
                          macro_dt=macro_dt,
                          sample_times=sample_times
                          )
        for plan in plans
    ]

    margin_left, margin_top, margin_right, margin_bottom = 58, 58, 24, 36
    plot_width = width * pixels_per_cell
    plot_height = height * pixels_per_cell
    canvas_size = (
        margin_left + plot_width + margin_right,
        margin_top + plot_height + margin_bottom,
    )

    def cell_to_pixel(x: float, y: float) -> np.ndarray:
        return np.array(
            [
                margin_left + (x + 0.5) * pixels_per_cell,
                margin_top + (height - y - 0.5) * pixels_per_cell,
            ],
            dtype=float,
        )

    def corners(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
        center = cell_to_pixel(x, y)
        half_length = 0.5 * robot_length_cells * pixels_per_cell
        half_width = 0.5 * robot_width_cells * pixels_per_cell
        longitudinal = np.array([math.cos(yaw), -math.sin(yaw)])
        lateral = np.array([math.sin(yaw), math.cos(yaw)])
        return [
            tuple(center + a * half_length * longitudinal + b * half_width * lateral)
            for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))
        ]

    background = Image.new("RGBA", canvas_size, "white")
    draw = ImageDraw.Draw(background)
    draw.rectangle(
        [margin_left, margin_top, margin_left + plot_width, margin_top + plot_height],
        fill="#f7f7f5",
        outline="#333333",
        width=2,
    )

    grid_color = "#353131"
    grid_width = 1

    for x in range(width + 1):
        px = margin_left + x * pixels_per_cell
        draw.line((px, margin_top, px, margin_top + plot_height), fill=grid_color, width=grid_width)
    for y in range(height + 1):
        py = margin_top + y * pixels_per_cell
        draw.line((margin_left, py, margin_left + plot_width, py), fill=grid_color, width=grid_width)

    for obstacle in problem.get("obstacles", []):
        x, y, w, h = (int(value) for value in obstacle["rect"])
        left = margin_left + x * pixels_per_cell
        right = margin_left + (x + w) * pixels_per_cell
        top = margin_top + (height - y - h) * pixels_per_cell
        bottom = margin_top + (height - y) * pixels_per_cell
        draw.rectangle((left, top, right, bottom), fill="#6c7075", outline="#222222")

    for index, plan in enumerate(plans):
        color = COLORS[index % len(COLORS)]

        trajectory = sampled[index]

        points = [
            tuple(cell_to_pixel(pose[0], pose[1])
                  )
                  for pose in trajectory
                  ]
        if len(points) >= 2:
            draw.line(points, fill=color + "99", width=3)

        start = plan["states"][0]
        goal = plan["states"][-1]
        start_yaw = start[2] * 2.0 * math.pi / heading_bins
        goal_yaw = goal[2] * 2.0 * math.pi / heading_bins
        draw.polygon(corners(start[0], start[1], start_yaw), fill=color + "33", outline=color)
        draw.polygon(corners(goal[0], goal[1], goal_yaw), fill="#ffffff55", outline=color, width=3)

    title_font = _font(18, bold=True)
    label_font = _font(14, bold=True)
    frames: list[Image.Image] = []
    for frame_index, time_value in enumerate(sample_times):
        frame = background.copy()
        frame_draw = ImageDraw.Draw(frame)
        frame_draw.text(
            (canvas_size[0] / 2, 28),
            (
                f"LaCAM + PIBT   sim t={time_value:.2f} s   "
                f"dt={macro_dt:.2f} s   "
                f"step={time_value / macro_dt:.2f}/{max_steps}   "
                f"cost={solution['cost']:.3g}"
            ),
            fill="#111111",
            font=title_font,
            anchor="mm",
        )
        for agent_index, trajectory in enumerate(sampled):
            x, y, yaw = trajectory[frame_index]
            color = COLORS[agent_index % len(COLORS)]
            polygon = corners(x, y, yaw)
            frame_draw.polygon(polygon, fill=color, outline="white", width=2)
            center = cell_to_pixel(x, y)
            heading = center + 0.38 * robot_length_cells * pixels_per_cell * np.array(
                [math.cos(yaw), -math.sin(yaw)]
            )
            frame_draw.line((tuple(center), tuple(heading)), fill="white", width=3)
            frame_draw.text(tuple(center), str(agent_index), fill="white", font=label_font, anchor="mm")
        frames.append(frame.convert("P", palette=Image.Palette.ADAPTIVE))

    if len(sample_times) >= 2:
        real_frame_seconds = (sample_times[1] - sample_times[0]) / playback_speed
        duration_ms = max(10, round(1000 * real_frame_seconds))
    else:
        duration_ms = max(10, round(1000 / fps))
    durations = [duration_ms] * len(frames)
    durations[0] += round(start_hold_seconds * 1000)
    durations[-1] += round(goal_hold_seconds * 1000)
    output_file = Path(output_file).resolve()
    output_file.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output_file,
        save_all=True,
        append_images=frames[1:],
        duration=durations,
        loop=0,
        disposal=2,
    )
    return output_file


def plan_and_animate(
    repo_root: str | Path,
    problem_file: str | Path,
    output_directory: str | Path,
    *,
    time_limit_ms: float | None = None,
    transition_cache: bool | None = None,
    candidate_cache: bool | None = None,
    ara_star: bool | None = None,
    fps: int = 20,
    playback_speed: float = 1.0,
    pixels_per_cell: int = 24,
    waypoint_only: bool = False,
) -> dict[str, Path]:
    output_directory = Path(output_directory).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    solution_file = output_directory / "solution.yaml"
    gif_file = output_directory / "animation.gif"
    run_planner(
        repo_root,
        problem_file,
        solution_file,
        time_limit_ms=time_limit_ms,
        transition_cache=transition_cache,
        candidate_cache=candidate_cache,
        ara_star=ara_star,
    )
    save_gif(
        problem_file,
        solution_file,
        gif_file,
        fps=fps,
        playback_speed=playback_speed,
        pixels_per_cell=pixels_per_cell,
        waypoint_only=waypoint_only,
    )
    return {"solution": solution_file, "animation": gif_file}


def benchmark_modes(
    repo_root: str | Path,
    problem_file: str | Path,
    output_directory: str | Path,
    *,
    time_limit_ms: float = 1000.0,
    include_repeated_weighted: bool = True,
) -> list[dict[str, Any]]:
    """Run cache/search mode combinations and return solution timing rows."""
    output_directory = Path(output_directory).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)

    ara_modes = (True, False) if include_repeated_weighted else (True,)
    rows: list[dict[str, Any]] = []
    for transition_cache in (True, False):
        for candidate_cache in (True, False):
            for ara_star in ara_modes:
                label = (
                    f"transition_{'on' if transition_cache else 'off'}__"
                    f"candidate_{'on' if candidate_cache else 'off'}__"
                    f"ara_{'on' if ara_star else 'off'}"
                )
                solution_file = output_directory / f"{label}.yaml"
                run_planner(
                    repo_root,
                    problem_file,
                    solution_file,
                    time_limit_ms=time_limit_ms,
                    transition_cache=transition_cache,
                    candidate_cache=candidate_cache,
                    ara_star=ara_star,
                )
                with solution_file.open(encoding="utf-8") as stream:
                    solution = yaml.safe_load(stream)
                timing = solution.get("timing", {})
                runtime = solution.get("runtime", {})
                rows.append(
                    {
                        "transition_cache": transition_cache,
                        "candidate_cache": candidate_cache,
                        "ara_star": ara_star,
                        "success": bool(solution.get("success")),
                        "cost": solution.get("cost"),
                        "static_precompute_ms": timing.get("static_precompute_ms"),
                        "query_precompute_ms": timing.get("query_precompute_ms"),
                        "search_ms": timing.get("search_ms"),
                        "warm_request_ms": timing.get("warm_request_ms"),
                        "cold_total_ms": timing.get("cold_total_ms"),
                        "expanded_nodes": runtime.get("expanded_nodes"),
                    }
                )
    return rows
