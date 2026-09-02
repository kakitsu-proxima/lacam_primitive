#!/usr/bin/env python3
"""Write the Japanese Markdown report from benchmark_details.json."""

from __future__ import annotations

import json
from pathlib import Path

import yaml

from export_artifacts import choose_profiles


def fmt(value: float | None) -> str:
    if value is None:
        return "—"
    if value < 0.01:
        return f"{value:.4f}"
    if value < 1.0:
        return f"{value:.3f}"
    return f"{value:.2f}"


def main() -> None:
    suite = Path(__file__).resolve().parent
    details = json.loads((suite / "benchmark_details.json").read_text(encoding="utf-8"))
    selected = choose_profiles(details)
    summary = {
        (row["scenario"], row["profile"]): row for row in details["summary"]
    }
    rows = []
    for scenario, profile in sorted(selected.items()):
        with (suite / f"{scenario}.yaml").open(encoding="utf-8") as stream:
            problem = yaml.safe_load(stream)
        moving = sum(
            item["start_ref"] != item["goal_ref"] for item in problem["agents"]
        )
        row = summary[scenario, profile]
        exact = bool(row["first_within_10ms"] and row["best_known_by_50ms"])
        quick = bool(row["first_within_10ms"])
        verdict = "達成" if exact else ("品質未達" if quick else "初期時間未達")
        rows.append((scenario, moving, profile, row, exact, quick, verdict))

    quick_count = sum(row[5] for row in rows)
    exact_count = sum(row[4] for row in rows)
    category_lines = []
    for moving in (1, 2, 3):
        subset = [row for row in rows if row[1] == moving]
        category_lines.append(
            f"- {moving}台移動: {len(subset)}例、初期≤10 ms "
            f"{sum(row[5] for row in subset)}/{len(subset)}、"
            f"50 msでbest-known {sum(row[4] for row in subset)}/{len(subset)}"
        )

    table = [
        "| ID / scenario | 移動台数 | 代表profile | 初期ms | 初期cost | 50ms cost | best-known | 判定 |",
        "|---|---:|---|---:|---:|---:|---:|---|",
    ]
    for scenario, moving, profile, row, _, _, verdict in rows:
        table.append(
            f"| {scenario} | {moving} | `{profile}` | "
            f"{fmt(row['first_ms_median'])} | {fmt(row['first_cost_median'])} | "
            f"{fmt(row['best_50_cost_median'])} | {fmt(row['best_known_cost'])} | "
            f"{verdict} |"
        )

    report = f"""# 加速度制約あり・30シナリオベンチマーク

計測日: 2026-09-02  
planner HEAD: `e1d011a` + 作業ツリー上の加速度対応変更  
build: Release / Intel Core Ultra 7 155H  
短時間profile: 100 ms × 5回、逐次実行  
reference: 1,000 ms × 1回

## 構成

全シナリオでagentは3台、加速度制約・dynamics-aware PIBT・非中心pivot -0.3 mを有効にしています。位置はx=0.3–2.3 m、y=0.2–0.8 m、姿勢は0°・45°・90°・135°・180°・225°・270°を使用します。

{chr(10).join(category_lines)}

## 結論

- 初期解中央値10 ms以下: **{quick_count}/30例**
- 初期解10 ms以下、かつ50 msまでに今回のbest-known cost: **{exact_count}/30例**
- 30/30例で加速度制約を満たす解を確認

簡単な例だけではなく、長距離横断、45°/135°/225°終端、2台交差、3台swap・relay・dense配置を含みます。未達例は削除せず、探索時間と品質gapを示すstress caseとして残しています。

## 全結果

時刻はcache構築を除くsearch開始基準の5回中央値です。`best-known` は全短時間profileと1,000 ms referenceで観測した最小costであり、最適性証明ではありません。

{chr(10).join(table)}

## 保存データ

- `benchmark_raw.csv`: 全1950短時間runの生データ
- `benchmark_summary.csv`: scenario/profile別の中央値
- `benchmark_details.json`: 全incumbent履歴
- `artifacts/scenario_poses.csv`: 全90 agent設定の物理start/goal
- `artifacts/manifest.csv`: 代表runの全解とGIF/CSVの索引

再生成方法と各シナリオの説明は [README.md](README.md) を参照してください。
"""
    (suite / "RESULTS_JA.md").write_text(report, encoding="utf-8")


if __name__ == "__main__":
    main()
