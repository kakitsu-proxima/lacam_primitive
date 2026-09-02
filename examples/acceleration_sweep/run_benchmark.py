#!/usr/bin/env python3
"""Benchmark the three-agent acceleration-constrained scenario suite.

All planner invocations are sequential so that concurrent processes do not
distort the search-time comparison.  The reported search/improvement clocks
exclude static and query cache construction by design.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import statistics
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path


IMPROVEMENT_RE = re.compile(
    r"solution improved: cost=(?P<cost>[-+0-9.eE]+).*?search_ms=(?P<ms>[-+0-9.eE]+)"
)
SEARCH_RE = re.compile(r"^search: (?P<ms>[-+0-9.eE]+) ms$", re.MULTILINE)


@dataclass(frozen=True)
class Profile:
    name: str
    weight: float
    max_branching: int
    initial_branching: int
    alternatives: int = 4
    lazy_successors: bool = False


PROFILES = (
    Profile("narrow_w2_b4", 2.0, 4, 0),
    Profile("narrow_w3_b4", 3.0, 4, 0),
    Profile("narrow_w5_b4", 5.0, 4, 0),
    Profile("staged_w2_4to8", 2.0, 8, 4),
    Profile("staged_w3_4to8", 3.0, 8, 4),
    Profile("staged_w5_4to8", 5.0, 8, 4),
    Profile("staged_w3_4to12", 3.0, 12, 4),
    Profile("fast_w10_b3_a3", 10.0, 3, 0, 3),
    Profile("staged_w5_3to4_a3", 5.0, 4, 3, 3),
    Profile("staged_w10_3to4_a3", 10.0, 4, 3, 3),
    Profile("staged_w5_3to8_a3", 5.0, 8, 3, 3),
    Profile("staged_w10_3to8_a3", 10.0, 8, 3, 3),
    Profile("staged_w10_3to4_a3_lazy", 10.0, 4, 3, 3, True),
)


def run_once(
    executable: Path,
    scenario: Path,
    output: Path,
    profile: Profile,
    time_limit_ms: float,
) -> dict[str, object]:
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(executable),
        "--input",
        str(scenario),
        "--output",
        str(output),
        "--time-limit-ms",
        str(time_limit_ms),
        "--anytime",
        "on",
        "--initial-weight",
        str(profile.weight),
        "--max-branching",
        str(profile.max_branching),
        "--initial-solution-max-branching",
        str(profile.initial_branching),
        "--alternatives-per-agent",
        str(profile.alternatives),
        "--lazy-successors",
        "on" if profile.lazy_successors else "off",
    ]
    completed = subprocess.run(
        command,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    improvements = [
        {"cost": float(match.group("cost")), "elapsed_ms": float(match.group("ms"))}
        for match in IMPROVEMENT_RE.finditer(completed.stdout)
    ]
    search_match = SEARCH_RE.search(completed.stdout)
    best_50 = min(
        (item["cost"] for item in improvements if item["elapsed_ms"] <= 50.0),
        default=None,
    )
    return {
        "scenario": scenario.stem,
        "profile": profile.name,
        "returncode": completed.returncode,
        "success": bool(improvements),
        "first_ms": improvements[0]["elapsed_ms"] if improvements else None,
        "first_cost": improvements[0]["cost"] if improvements else None,
        "second_ms": improvements[1]["elapsed_ms"] if len(improvements) > 1 else None,
        "second_cost": improvements[1]["cost"] if len(improvements) > 1 else None,
        "best_50_cost": best_50,
        "final_cost": improvements[-1]["cost"] if improvements else None,
        "improvement_count": len(improvements),
        "search_ms": float(search_match.group("ms")) if search_match else None,
        "improvements": improvements,
    }


def median(values: list[float | None]) -> float | None:
    present = [value for value in values if value is not None]
    return statistics.median(present) if present else None


def summarise(rows: list[dict[str, object]], best_known: dict[str, float | None]) -> list[dict[str, object]]:
    groups: dict[tuple[str, str], list[dict[str, object]]] = {}
    for row in rows:
        groups.setdefault((str(row["scenario"]), str(row["profile"])), []).append(row)

    summary: list[dict[str, object]] = []
    for (scenario, profile), group in sorted(groups.items()):
        first_values = [row["first_ms"] for row in group]
        successful_first = [float(value) for value in first_values if value is not None]
        first_cost = median([row["first_cost"] for row in group])
        best_50_cost = median([row["best_50_cost"] for row in group])
        reference = best_known.get(scenario)
        summary.append(
            {
                "scenario": scenario,
                "profile": profile,
                "runs": len(group),
                "successes": sum(bool(row["success"]) for row in group),
                "first_ms_median": median(first_values),
                "first_ms_min": min(successful_first) if successful_first else None,
                "first_ms_max": max(successful_first) if successful_first else None,
                "first_cost_median": first_cost,
                "second_ms_median": median([row["second_ms"] for row in group]),
                "best_50_cost_median": best_50_cost,
                "best_known_cost": reference,
                "first_within_10ms": (
                    median(first_values) is not None and median(first_values) <= 10.0
                ),
                "best_known_by_50ms": (
                    best_50_cost is not None
                    and reference is not None
                    and abs(best_50_cost - reference) < 1e-9
                ),
            }
        )
    return summary


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--time-limit-ms", type=float, default=100.0)
    parser.add_argument("--reference-time-limit-ms", type=float, default=1000.0)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--report-prefix", default="benchmark")
    parser.add_argument(
        "--scenario",
        action="append",
        help="scenario stem or glob; repeat to select several (default: all)",
    )
    args = parser.parse_args()

    repo = args.repo.resolve()
    suite = Path(__file__).resolve().parent
    output_dir = (args.output_dir or repo / "outputs" / "acceleration_sweep").resolve()
    executable = repo / "build" / "lacam_primitive"
    patterns = args.scenario or ["[0-9][0-9]_*.yaml"]
    scenarios = sorted({path for pattern in patterns for path in suite.glob(pattern)})
    if not executable.is_file():
        raise SystemExit(f"planner executable not found: {executable}")
    if not scenarios:
        raise SystemExit(f"no scenarios found under: {suite}")

    # The narrow first pass is essential on the bottleneck cases: a direct
    # width-12 search can spend the entire budget near the root.
    reference_profile = Profile("reference_w3_4to12", 3.0, 12, 4)
    reference_rows: list[dict[str, object]] = []
    for scenario in scenarios:
        row = run_once(
            executable,
            scenario,
            output_dir / "solutions" / "reference" / f"{scenario.stem}.yaml",
            reference_profile,
            args.reference_time_limit_ms,
        )
        reference_rows.append(row)
        print(f"reference {scenario.stem}: {row['final_cost']}", flush=True)

    best_known: dict[str, float | None] = {
        str(row["scenario"]): row["final_cost"] for row in reference_rows
    }
    raw_rows: list[dict[str, object]] = []
    for profile in PROFILES:
        for scenario in scenarios:
            for repetition in range(1, args.repeats + 1):
                row = run_once(
                    executable,
                    scenario,
                    output_dir / "solutions" / profile.name / f"{scenario.stem}_r{repetition}.yaml",
                    profile,
                    args.time_limit_ms,
                )
                row["repetition"] = repetition
                raw_rows.append(row)
                if row["final_cost"] is not None:
                    old = best_known.get(scenario.stem)
                    value = float(row["final_cost"])
                    best_known[scenario.stem] = value if old is None else min(old, value)
                print(
                    f"{profile.name} {scenario.stem} r{repetition}: "
                    f"first={row['first_ms']} cost={row['final_cost']}",
                    flush=True,
                )

    summary = summarise(raw_rows, best_known)
    serialisable_raw = [
        {key: value for key, value in row.items() if key != "improvements"}
        for row in raw_rows
    ]
    write_csv(suite / f"{args.report_prefix}_raw.csv", serialisable_raw)
    write_csv(suite / f"{args.report_prefix}_summary.csv", summary)
    with (suite / f"{args.report_prefix}_details.json").open("w", encoding="utf-8") as stream:
        json.dump(
            {
                "criteria": {
                    "first_solution_ms": 10.0,
                    "quality_deadline_ms": 50.0,
                    "timing_excludes_precompute": True,
                },
                "profiles": [asdict(profile) for profile in PROFILES],
                "reference_profile": asdict(reference_profile),
                "reference_runs": reference_rows,
                "best_known": best_known,
                "runs": raw_rows,
                "summary": summary,
            },
            stream,
            ensure_ascii=False,
            indent=2,
        )


if __name__ == "__main__":
    main()
