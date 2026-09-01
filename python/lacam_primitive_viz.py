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
    reverse_bfs: bool | None = None,
    candidate_diversification: bool | None = None,
    aabb_broadphase: bool | None = None,
    conflict_cache: bool | None = None,
    lazy_successors: bool | None = None,
    progressive_widening: bool | None = None,
    initial_candidate_width: int | None = None,
    per_primitive_intervals: bool | None = None,
    dynamics_aware_pibt: bool | None = None,
    interval_dominance: bool | None = None,
    multiple_rotation_amounts: bool | None = None,
    acceleration_constraints: bool | None = None,
    collision_mode: str | None = None,
    max_boundary_travel_per_interval_m: float | None = None,
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
    for option, value in (
        ("--reverse-bfs", reverse_bfs),
        ("--candidate-diversification", candidate_diversification),
        ("--aabb-broadphase", aabb_broadphase),
        ("--conflict-cache", conflict_cache),
        ("--lazy-successors", lazy_successors),
        ("--progressive-widening", progressive_widening),
        ("--per-primitive-intervals", per_primitive_intervals),
        ("--dynamics-aware-pibt", dynamics_aware_pibt),
        ("--interval-dominance", interval_dominance),
        ("--multiple-rotation-amounts", multiple_rotation_amounts),
        ("--acceleration-constraints", acceleration_constraints),
    ):
        if value is not None:
            command.extend([option, "on" if value else "off"])
    if initial_candidate_width is not None:
        if initial_candidate_width <= 0:
            raise ValueError("initial_candidate_width must be positive")
        command.extend(
            ["--initial-candidate-width", str(int(initial_candidate_width))]
        )
    if collision_mode is not None:
        if collision_mode not in {"time_indexed", "whole_step"}:
            raise ValueError(
                "collision_mode must be 'time_indexed' or 'whole_step'"
            )
        command.extend(["--collision-mode", collision_mode])
    if max_boundary_travel_per_interval_m is not None:
        if max_boundary_travel_per_interval_m <= 0:
            raise ValueError(
                "max_boundary_travel_per_interval_m must be positive"
            )
        command.extend(
            [
                "--max-boundary-travel-per-interval-m",
                str(float(max_boundary_travel_per_interval_m)),
            ]
        )

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


def _exact_cells(value_m: float, cell_size: float, name: str) -> int:
    cells = float(value_m) / cell_size
    rounded = round(cells)
    if not math.isclose(cells, rounded, abs_tol=1e-8):
        raise ValueError(
            f"{name}={value_m} m is not representable on the "
            f"{cell_size} m integer lattice"
        )
    return int(rounded)


def _grid_dimensions(grid: dict[str, Any]) -> tuple[int, int]:
    if "width_cells" in grid and "height_cells" in grid:
        return int(grid["width_cells"]), int(grid["height_cells"])
    bounds = grid.get("bounds_m")
    if bounds is None:
        raise ValueError("grid needs width_cells/height_cells or bounds_m")
    cell_size = float(grid["cell_size"])
    return (
        _exact_cells(float(bounds[2]) - float(bounds[0]), cell_size, "grid width"),
        _exact_cells(float(bounds[3]) - float(bounds[1]), cell_size, "grid height"),
    )


def _build_primitive_catalog(
    problem: dict[str, Any],
    *,
    multiple_rotation_amounts_enabled: bool | None = None,
) -> list[dict[str, Any]]:
    """
    Reconstruct primitive IDs in exactly the same order as the C++ PrimitiveTable.

    The C++ order is:

      wait
      east/west/north/south for each sorted translation distance
      rotate_ccw / rotate_cw for each sorted rotation amount and pivot offset
    """
    primitive_cfg = problem["primitives"]

    cell_size = float(problem["grid"]["cell_size"])
    if "translation_distances_m" in primitive_cfg:
        translation_cells = sorted(
            set(
                _exact_cells(float(v), cell_size, "translation_distances_m[]")
                for v in primitive_cfg["translation_distances_m"]
            )
        )
    else:
        translation_cells = sorted(
            set(int(v) for v in primitive_cfg["translation_cells"])
        )

    rotation_value = primitive_cfg["rotation_bins"]
    if isinstance(rotation_value, list):
        rotation_bin_counts = sorted(set(int(v) for v in rotation_value))
    else:
        rotation_bin_counts = [int(rotation_value)]
    if multiple_rotation_amounts_enabled is None:
        multiple_rotation_amounts_enabled = bool(
            primitive_cfg.get("use_multiple_rotation_amounts", False)
        )
    if not multiple_rotation_amounts_enabled:
        rotation_bin_counts = [min(rotation_bin_counts)]

    if "rotation_pivot_offsets_m" in primitive_cfg:
        pivot_offsets = sorted(
            set(
                _exact_cells(float(v), cell_size, "rotation_pivot_offsets_m[]")
                for v in primitive_cfg["rotation_pivot_offsets_m"]
            )
        )
    else:
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

    multiple_rotation_amounts = len(rotation_bin_counts) > 1
    for rotation_bins in rotation_bin_counts:
        for pivot_offset in pivot_offsets:
            pivot_suffix = (
                "center"
                if pivot_offset == 0
                else (
                    f"front_{pivot_offset}"
                    if pivot_offset > 0
                    else f"rear_{-pivot_offset}"
                )
            )
            suffix = (
                f"{rotation_bins}_{pivot_suffix}"
                if multiple_rotation_amounts
                else pivot_suffix
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


def _select_solution_variant(
    solution: dict[str, Any],
    solution_index: int | None,
) -> dict[str, Any]:
    """Return the final solution or one recorded anytime incumbent."""
    if solution_index is None:
        return {
            "cost": solution["cost"],
            "plans": solution["plans"],
            "elapsed_ms": solution.get("elapsed_ms"),
            "weight": None,
        }

    improvements = solution.get("improvements", [])
    if not improvements:
        raise ValueError("solution file does not contain anytime improvements")
    index = solution_index if solution_index >= 0 else len(improvements) + solution_index
    if index < 0 or index >= len(improvements):
        raise IndexError(
            f"solution_index {solution_index} is outside "
            f"the {len(improvements)} recorded improvements"
        )
    variant = improvements[index]
    if "plans" not in variant:
        raise ValueError(
            "this solution file only records improvement costs; rerun the updated planner "
            "to visualize earlier incumbents"
        )
    return variant


def save_gif(
    problem_file: str | Path,
    solution_file: str | Path,
    output_file: str | Path,
    *,
    fps: int = 20,
    playback_speed: float = 1.0,
    pixels_per_meter: float = 240.0,
    pixels_per_cell: float | None = None,
    start_hold_seconds: float = 0.6,
    goal_hold_seconds: float = 1.2,
    waypoint_only: bool = False,
    solution_index: int | None = None,
) -> Path:
    if fps <= 0 or playback_speed <= 0 or pixels_per_meter <= 0:
        raise ValueError("fps, playback_speed, and pixels_per_meter must be positive")
    if pixels_per_cell is not None and pixels_per_cell <= 0:
        raise ValueError("pixels_per_cell must be positive when specified")

    with Path(problem_file).open(encoding="utf-8") as stream:
        problem: dict[str, Any] = yaml.safe_load(stream)
    with Path(solution_file).open(encoding="utf-8") as stream:
        solution: dict[str, Any] = yaml.safe_load(stream)

    if not solution.get("success"):
        raise ValueError("solution file does not contain a successful plan")

    variant = _select_solution_variant(solution, solution_index)

    grid = problem["grid"]
    robot = problem["robot"]
    width, height = _grid_dimensions(grid)
    cell_size = float(grid["cell_size"])
    origin_x, origin_y = (float(v) for v in grid.get("origin", [0.0, 0.0]))
    if "bounds_m" in grid:
        min_x_m, min_y_m, max_x_m, max_y_m = (
            float(v) for v in grid["bounds_m"]
        )
    else:
        min_x_m = origin_x - 0.5 * cell_size
        min_y_m = origin_y - 0.5 * cell_size
        max_x_m = origin_x + (width - 0.5) * cell_size
        max_y_m = origin_y + (height - 0.5) * cell_size
    heading_bins = int(grid["heading_bins"])
    macro_dt = float(grid["macro_dt"])
    robot_length_m = float(robot["size"][0])
    robot_width_m = float(robot["size"][1])

    plans = []
    for raw_plan in variant["plans"]:
        plan = dict(raw_plan)
        if raw_plan.get("states_m"):
            plan["states"] = [
                [
                    (float(state[0]) - origin_x) / cell_size,
                    (float(state[1]) - origin_y) / cell_size,
                    int(state[2]),
                ]
                for state in raw_plan["states_m"]
            ]
        plans.append(plan)
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
    primitive_catalog = _build_primitive_catalog(
        problem,
        multiple_rotation_amounts_enabled=solution.get("runtime", {}).get(
            "multiple_rotation_amounts_enabled"
        ),
    )

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

    # Physical-size scaling is the default: grids describing the same metric
    # workspace get the same canvas dimensions regardless of cell_size.
    # pixels_per_cell remains as an explicit backwards-compatible override.
    effective_pixels_per_meter = (
        float(pixels_per_cell) / cell_size
        if pixels_per_cell is not None
        else float(pixels_per_meter)
    )
    pixel_scale = cell_size * effective_pixels_per_meter
    margin_left, margin_top, margin_right, margin_bottom = 58, 58, 24, 36
    plot_width = max(1, round((max_x_m - min_x_m) * effective_pixels_per_meter))
    plot_height = max(1, round((max_y_m - min_y_m) * effective_pixels_per_meter))
    canvas_size = (
        margin_left + plot_width + margin_right,
        margin_top + plot_height + margin_bottom,
    )

    def cell_to_pixel(x: float, y: float) -> np.ndarray:
        world_x = origin_x + x * cell_size
        world_y = origin_y + y * cell_size
        return np.array(
            [
                margin_left + (world_x - min_x_m) * effective_pixels_per_meter,
                margin_top + (max_y_m - world_y) * effective_pixels_per_meter,
            ],
            dtype=float,
        )

    def corners(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
        center = cell_to_pixel(x, y)
        half_length = 0.5 * robot_length_m * effective_pixels_per_meter
        half_width = 0.5 * robot_width_m * effective_pixels_per_meter
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

    for x in range(width):
        px = cell_to_pixel(float(x), 0.0)[0]
        draw.line((px, margin_top, px, margin_top + plot_height), fill=grid_color, width=grid_width)
    for y in range(height):
        py = cell_to_pixel(0.0, float(y))[1]
        draw.line((margin_left, py, margin_left + plot_width, py), fill=grid_color, width=grid_width)

    for obstacle in problem.get("obstacles", []):
        if "rect_m" in obstacle:
            x_m, y_m, w_m, h_m = (float(value) for value in obstacle["rect_m"])
        else:
            x, y, w, h = (int(value) for value in obstacle["rect"])
            x_m = origin_x + (x - 0.5) * cell_size
            y_m = origin_y + (y - 0.5) * cell_size
            w_m = w * cell_size
            h_m = h * cell_size
        left = margin_left + (x_m - min_x_m) * effective_pixels_per_meter
        right = margin_left + (x_m + w_m - min_x_m) * effective_pixels_per_meter
        top = margin_top + (max_y_m - y_m - h_m) * effective_pixels_per_meter
        bottom = margin_top + (max_y_m - y_m) * effective_pixels_per_meter
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
                f"step={int(time_value / macro_dt + 1e-9)}/{max_steps}   "
                f"cost={variant['cost']:.3g}"
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
            heading = center + 0.38 * robot_length_m * effective_pixels_per_meter * np.array(
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
    reverse_bfs: bool | None = None,
    candidate_diversification: bool | None = None,
    aabb_broadphase: bool | None = None,
    conflict_cache: bool | None = None,
    lazy_successors: bool | None = None,
    progressive_widening: bool | None = None,
    initial_candidate_width: int | None = None,
    per_primitive_intervals: bool | None = None,
    dynamics_aware_pibt: bool | None = None,
    interval_dominance: bool | None = None,
    multiple_rotation_amounts: bool | None = None,
    acceleration_constraints: bool | None = None,
    collision_mode: str | None = None,
    max_boundary_travel_per_interval_m: float | None = None,
    fps: int = 20,
    playback_speed: float = 1.0,
    pixels_per_meter: float = 240.0,
    pixels_per_cell: float | None = None,
    waypoint_only: bool | None = None,
    animation_modes: tuple[str, ...] = ("smooth", "waypoints"),
    include_all_solutions: bool = True,
) -> dict[str, Any]:
    output_directory = Path(output_directory).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    solution_file = output_directory / "solution.yaml"
    run_planner(
        repo_root,
        problem_file,
        solution_file,
        time_limit_ms=time_limit_ms,
        transition_cache=transition_cache,
        candidate_cache=candidate_cache,
        ara_star=ara_star,
        reverse_bfs=reverse_bfs,
        candidate_diversification=candidate_diversification,
        aabb_broadphase=aabb_broadphase,
        conflict_cache=conflict_cache,
        lazy_successors=lazy_successors,
        progressive_widening=progressive_widening,
        initial_candidate_width=initial_candidate_width,
        per_primitive_intervals=per_primitive_intervals,
        dynamics_aware_pibt=dynamics_aware_pibt,
        interval_dominance=interval_dominance,
        multiple_rotation_amounts=multiple_rotation_amounts,
        acceleration_constraints=acceleration_constraints,
        collision_mode=collision_mode,
        max_boundary_travel_per_interval_m=(
            max_boundary_travel_per_interval_m
        ),
    )
    with solution_file.open(encoding="utf-8") as stream:
        solution: dict[str, Any] = yaml.safe_load(stream)

    if waypoint_only is not None:
        # Backwards compatibility with the old one-animation API.
        animation_modes = ("waypoints" if waypoint_only else "smooth",)
    invalid_modes = set(animation_modes) - {"smooth", "waypoints"}
    if invalid_modes:
        raise ValueError(f"unknown animation modes: {sorted(invalid_modes)}")

    improvements = solution.get("improvements", [])
    if include_all_solutions and improvements and all(
        "plans" in item for item in improvements
    ):
        solution_indices: list[int | None] = list(range(len(improvements)))
    else:
        solution_indices = [None]

    animations: list[dict[str, Any]] = []
    for sequence, solution_index in enumerate(solution_indices):
        variant = _select_solution_variant(solution, solution_index)
        cost_text = str(variant["cost"]).replace(".", "p")
        for mode in animation_modes:
            gif_file = output_directory / (
                f"solution_{sequence:02d}_cost_{cost_text}_{mode}.gif"
            )
            save_gif(
                problem_file,
                solution_file,
                gif_file,
                fps=fps,
                playback_speed=playback_speed,
                pixels_per_meter=pixels_per_meter,
                pixels_per_cell=pixels_per_cell,
                waypoint_only=mode == "waypoints",
                solution_index=solution_index,
            )
            animations.append(
                {
                    "solution_index": solution_index,
                    "cost": variant["cost"],
                    "weight": variant.get("weight"),
                    "elapsed_ms": variant.get("elapsed_ms"),
                    "mode": mode,
                    "path": gif_file,
                }
            )

    preferred = next(
        (
            item["path"]
            for item in reversed(animations)
            if item["mode"] == "smooth"
        ),
        animations[-1]["path"],
    )
    return {
        "solution": solution_file,
        "animation": preferred,
        "animations": animations,
    }


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
