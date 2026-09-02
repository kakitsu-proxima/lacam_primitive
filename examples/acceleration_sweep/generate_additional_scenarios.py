#!/usr/bin/env python3
"""Generate the deterministic 12--30 scenario extension."""

from __future__ import annotations

import copy
from pathlib import Path

import yaml


POSES = {
    "LB_E": [5, 2, 0],
    "BM_E": [13, 2, 0],
    "RB_W": [21, 2, 4],
    "LT_E": [5, 8, 0],
    "TM_W": [13, 8, 4],
    "RT_W": [21, 8, 4],
    "LV_N": [3, 5, 2],
    "CV_S": [10, 5, 6],
    "RV_N": [17, 5, 2],
    "XV_S": [23, 5, 6],
    "LD_NE": [5, 5, 1],
    "CD_NW": [12, 5, 3],
    "RD_SW": [20, 5, 5],
}


# Tuple entries are (start pose name, goal pose name).  Equal names denote a
# stationary agent; every scenario still contains exactly three agents.
SCENARIOS = {
    "12_one_agent_left_dock": (
        ("LV_N", "LB_E"), ("TM_W", "TM_W"), ("XV_S", "XV_S")
    ),
    "13_one_agent_right_dock": (
        ("RT_W", "XV_S"), ("CD_NW", "CD_NW"), ("LB_E", "LB_E")
    ),
    "14_one_agent_bottom_reverse": (
        ("BM_E", "RB_W"), ("LD_NE", "LD_NE"), ("RT_W", "RT_W")
    ),
    "15_one_agent_top_transfer": (
        ("XV_S", "TM_W"), ("BM_E", "BM_E"), ("LV_N", "LV_N")
    ),
    "16_one_agent_diagonal_goal": (
        ("RV_N", "CD_NW"), ("LB_E", "LB_E"), ("XV_S", "XV_S")
    ),
    "17_one_agent_diagonal_shift": (
        ("LD_NE", "CD_NW"), ("RB_W", "RB_W"), ("RT_W", "RT_W")
    ),
    "18_one_agent_top_lane": (
        ("LT_E", "TM_W"), ("LB_E", "LB_E"), ("RB_W", "RB_W")
    ),
    "19_two_agent_diverge": (
        ("LB_E", "LV_N"), ("XV_S", "BM_E"), ("TM_W", "TM_W")
    ),
    "20_two_agent_center_exchange": (
        ("BM_E", "RV_N"), ("TM_W", "CV_S"), ("LV_N", "LV_N")
    ),
    "21_two_agent_long_relocation": (
        ("LB_E", "LV_N"), ("RB_W", "XV_S"), ("TM_W", "TM_W")
    ),
    "22_two_agent_vertical_top": (
        ("CV_S", "LV_N"), ("RV_N", "TM_W"), ("XV_S", "XV_S")
    ),
    "23_two_agent_top_to_center": (
        ("LT_E", "TM_W"), ("XV_S", "LV_N"), ("BM_E", "BM_E")
    ),
    "24_two_agent_top_swap": (
        ("TM_W", "LT_E"), ("XV_S", "RT_W"), ("BM_E", "BM_E")
    ),
    "25_two_agent_cross_lane": (
        ("LB_E", "CV_S"), ("TM_W", "RT_W"), ("RB_W", "RB_W")
    ),
    "26_three_agent_corner_relay": (
        ("RB_W", "LT_E"), ("RT_W", "RB_W"), ("CD_NW", "LB_E")
    ),
    "27_three_agent_compact_cycle": (
        ("RB_W", "RT_W"), ("RT_W", "BM_E"), ("CV_S", "LV_N")
    ),
    "28_three_agent_center_swap": (
        ("TM_W", "LV_N"), ("LV_N", "TM_W"), ("XV_S", "BM_E")
    ),
    "29_three_agent_lane_mix": (
        ("LB_E", "RB_W"), ("LT_E", "CV_S"), ("RV_N", "RT_W")
    ),
    "30_three_agent_corner_cycle": (
        ("RB_W", "RT_W"), ("RT_W", "LT_E"), ("CV_S", "RB_W")
    ),
}


class PlannerDumper(yaml.SafeDumper):
    def increase_indent(self, flow: bool = False, indentless: bool = False):
        return super().increase_indent(flow, False)

    def ignore_aliases(self, data):
        return True


def represent_list(dumper: PlannerDumper, values: list):
    scalar_only = all(not isinstance(value, (dict, list)) for value in values)
    return dumper.represent_sequence(
        "tag:yaml.org,2002:seq", values, flow_style=scalar_only
    )


PlannerDumper.add_representer(list, represent_list)


def main() -> None:
    suite = Path(__file__).resolve().parent
    with (suite / "02_one_agent_short_turn.yaml").open(encoding="utf-8") as stream:
        template = yaml.safe_load(stream)
    for stem, moves in SCENARIOS.items():
        problem = copy.deepcopy(template)
        problem["agents"] = [
            {
                "start_ref": POSES[start],
                "goal_ref": POSES[goal],
            }
            for start, goal in moves
        ]
        output = suite / f"{stem}.yaml"
        with output.open("w", encoding="utf-8") as stream:
            yaml.dump(
                problem,
                stream,
                Dumper=PlannerDumper,
                sort_keys=False,
                width=1000,
            )
        print(output.name)


if __name__ == "__main__":
    main()
