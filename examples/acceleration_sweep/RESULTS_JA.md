# 加速度制約あり・30シナリオベンチマーク

計測日: 2026-09-02  
planner HEAD: `e1d011a` + 作業ツリー上の加速度対応変更  
build: Release / Intel Core Ultra 7 155H  
短時間profile: 100 ms × 5回、逐次実行  
reference: 1,000 ms × 1回

## 構成

全シナリオでagentは3台、加速度制約・dynamics-aware PIBT・非中心pivot -0.3 mを有効にしています。位置はx=0.3–2.3 m、y=0.2–0.8 m、姿勢は0°・45°・90°・135°・180°・225°・270°を使用します。

- 1台移動: 10例、初期≤10 ms 9/10、50 msでbest-known 7/10
- 2台移動: 10例、初期≤10 ms 10/10、50 msでbest-known 8/10
- 3台移動: 10例、初期≤10 ms 9/10、50 msでbest-known 5/10

## 結論

- 初期解中央値10 ms以下: **28/30例**
- 初期解10 ms以下、かつ50 msまでに今回のbest-known cost: **20/30例**
- 30/30例で加速度制約を満たす解を確認

簡単な例だけではなく、長距離横断、45°/135°/225°終端、2台交差、3台swap・relay・dense配置を含みます。未達例は削除せず、探索時間と品質gapを示すstress caseとして残しています。

## 全結果

時刻はcache構築を除くsearch開始基準の5回中央値です。`best-known` は全短時間profileと1,000 ms referenceで観測した最小costであり、最適性証明ではありません。

| ID / scenario | 移動台数 | 代表profile | 初期ms | 初期cost | 50ms cost | best-known | 判定 |
|---|---:|---|---:|---:|---:|---:|---|
| 01_one_agent_cross_workspace | 1 | `staged_w10_3to4_a3_lazy` | 14.91 | 6.50 | 6.50 | 6.00 | 初期時間未達 |
| 02_one_agent_short_turn | 1 | `staged_w10_3to4_a3_lazy` | 0.104 | 4.00 | 4.00 | 4.00 | 達成 |
| 03_one_agent_top_reorient | 1 | `staged_w3_4to12` | 0.581 | 3.50 | 3.00 | 3.00 | 達成 |
| 04_two_agent_relocation | 2 | `staged_w10_3to8_a3` | 0.444 | 4.50 | 3.50 | 3.50 | 達成 |
| 05_two_agent_crossing | 2 | `fast_w10_b3_a3` | 3.84 | 9.50 | 7.00 | 6.00 | 品質未達 |
| 06_two_agent_diagonal | 2 | `staged_w3_4to12` | 4.60 | 6.00 | 5.00 | 5.00 | 達成 |
| 07_three_agent_relay | 3 | `staged_w5_4to8` | 2.70 | 6.50 | 5.50 | 5.50 | 達成 |
| 08_three_agent_dense | 3 | `fast_w10_b3_a3` | 6.25 | 9.00 | 7.50 | 6.50 | 品質未達 |
| 09_three_agent_diagonal | 3 | `staged_w5_3to4_a3` | 10.13 | 6.50 | 6.50 | 6.00 | 初期時間未達 |
| 10_three_agent_swap_and_exit | 3 | `narrow_w5_b4` | 9.36 | 8.50 | 6.50 | 6.00 | 品質未達 |
| 11_three_agent_fast_cycle | 3 | `staged_w5_3to4_a3` | 1.89 | 7.00 | 6.00 | 6.00 | 達成 |
| 12_one_agent_left_dock | 1 | `staged_w10_3to4_a3_lazy` | 0.068 | 1.50 | 1.50 | 1.50 | 達成 |
| 13_one_agent_right_dock | 1 | `staged_w10_3to4_a3_lazy` | 0.067 | 1.50 | 1.50 | 1.50 | 達成 |
| 14_one_agent_bottom_reverse | 1 | `staged_w2_4to8` | 0.852 | 3.50 | 3.50 | 3.50 | 達成 |
| 15_one_agent_top_transfer | 1 | `staged_w10_3to4_a3_lazy` | 0.251 | 2.00 | 2.00 | 2.00 | 達成 |
| 16_one_agent_diagonal_goal | 1 | `staged_w5_3to4_a3` | 8.81 | 8.50 | 8.50 | 4.50 | 品質未達 |
| 17_one_agent_diagonal_shift | 1 | `staged_w10_3to4_a3_lazy` | 0.198 | 3.50 | 3.50 | 3.50 | 達成 |
| 18_one_agent_top_lane | 1 | `staged_w10_3to4_a3_lazy` | 0.286 | 4.00 | 4.00 | 3.50 | 品質未達 |
| 19_two_agent_diverge | 2 | `staged_w10_3to8_a3` | 0.447 | 4.50 | 3.50 | 3.50 | 達成 |
| 20_two_agent_center_exchange | 2 | `staged_w10_3to4_a3_lazy` | 0.464 | 2.50 | 2.00 | 2.00 | 達成 |
| 21_two_agent_long_relocation | 2 | `staged_w5_3to8_a3` | 0.367 | 3.00 | 2.50 | 2.50 | 達成 |
| 22_two_agent_vertical_top | 2 | `staged_w5_3to8_a3` | 0.467 | 5.00 | 4.00 | 4.00 | 達成 |
| 23_two_agent_top_to_center | 2 | `staged_w10_3to4_a3_lazy` | 1.93 | 10.00 | 8.00 | 7.50 | 品質未達 |
| 24_two_agent_top_swap | 2 | `staged_w5_3to8_a3` | 0.580 | 4.00 | 3.50 | 3.50 | 達成 |
| 25_two_agent_cross_lane | 2 | `staged_w10_3to4_a3_lazy` | 0.184 | 3.00 | 3.00 | 3.00 | 達成 |
| 26_three_agent_corner_relay | 3 | `staged_w10_3to4_a3_lazy` | 6.53 | 8.50 | 8.50 | 7.50 | 品質未達 |
| 27_three_agent_compact_cycle | 3 | `narrow_w5_b4` | 1.06 | 6.00 | 5.50 | 5.50 | 達成 |
| 28_three_agent_center_swap | 3 | `fast_w10_b3_a3` | 3.83 | 8.00 | 7.50 | 6.50 | 品質未達 |
| 29_three_agent_lane_mix | 3 | `staged_w10_3to4_a3_lazy` | 0.489 | 5.00 | 4.50 | 4.50 | 達成 |
| 30_three_agent_corner_cycle | 3 | `staged_w5_3to4_a3` | 0.900 | 5.50 | 5.00 | 5.00 | 達成 |

## 保存データ

- `benchmark_raw.csv`: 全1950短時間runの生データ
- `benchmark_summary.csv`: scenario/profile別の中央値
- `benchmark_details.json`: 全incumbent履歴
- `artifacts/scenario_poses.csv`: 全90 agent設定の物理start/goal
- `artifacts/manifest.csv`: 代表runの全解とGIF/CSVの索引

再生成方法と各シナリオの説明は [README.md](README.md) を参照してください。
