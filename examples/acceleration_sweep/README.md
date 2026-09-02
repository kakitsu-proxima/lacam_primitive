# 加速度制約あり・3台構成ベンチマーク

`examples/example.yaml` と同じ物理環境・車体・非中心 pivot を使い、常に3台を配置したまま、1台だけ、2台、3台すべてが動く30問題を比較するための設定集です。

## 共通条件

- 盤面: 2.6 m × 1.0 m、cell size 0.1 m、8 heading bins
- 1 step: 0.5 s
- 車体: 0.825 m × 0.275 m、collision padding 0.005 m
- 非中心 pivot offset: -0.3 m
- 最大並進加速度・車体点加速度: 19.6 m/s²
- 45°/90°回転 primitive、time-indexed collision check
- `use_acceleration_constraints`、`use_dynamics_aware_pibt`、`use_interval_dominance` はすべて有効

`start_ref` / `goal_ref` は0.1 m・8方向の `pose_reference` 上の物理 poseを表します。使用範囲はx=0.3–2.3 m、y=0.2–0.8 mで、東西南北に加えて45°・135°・225°姿勢を含みます。計画格子のcell sizeを変更しても、この参照を変えなければ同じ物理設定を投影できます。

## シナリオ

| ID | 動く台数 | 内容 |
|---|---:|---|
| 01 | 1 | x=0.3 mの北向きからx=2.3 mの南向きまで横断 |
| 02 | 1 | 左側の北向きから中央の南向きへ短距離移動 |
| 03 | 1 | 上段中央で西向きから東向きへ180°変更 |
| 04 | 2 | 左下→左中央と、右中央→中央下の同時再配置 |
| 05 | 2 | 左下→右上と、右中央→中央下の交差移動 |
| 06 | 2 | 下段の東西反転と、45°→135°の斜め姿勢移動 |
| 07 | 3 | 右下→右上、上中央→左下、左中央→右下のrelay |
| 08 | 3 | 左右移動と上下段移動が競合する密な3台移動 |
| 09 | 3 | 225°のgoalを含む斜め3台移動 |
| 10 | 3 | 左側2台の位置交換と、中央車両の右下退避 |
| 11 | 3 | 上下段と左中央を使う短時間3台cycle |
| 12 | 1 | 左中央の北向き車両を左下へdock |
| 13 | 1 | 右上の西向き車両を右中央の南向きへ変更 |
| 14 | 1 | 下段中央から右下へ移動し東西反転 |
| 15 | 1 | 右中央から上段中央へ移動 |
| 16 | 1 | 中央右の北向きから中央の135°poseへ移動するstress case |
| 17 | 1 | 45°poseから135°poseへの斜めshift |
| 18 | 1 | 上段左から上段中央へのlane移動 |
| 19 | 2 | 左下と右中央から異なる方向へdiverge |
| 20 | 2 | 上下・中央を使う2台exchange |
| 21 | 2 | 左下→左中央、右下→右中央の同時移動 |
| 22 | 2 | 中央縦poseと上段poseの交換的移動 |
| 23 | 2 | 上段左→上段中央、右中央→左中央の長距離移動 |
| 24 | 2 | 上段2地点への同時再配置 |
| 25 | 2 | 左下→中央縦poseと上段中央→右上のcross-lane |
| 26 | 3 | 135°poseを含むcorner relay stress case |
| 27 | 3 | 右上下と中央縦poseを使うcompact cycle |
| 28 | 3 | 左中央・上段中央のswapを含むcenter cycle |
| 29 | 3 | 上下段と中央縦poseを混ぜたlane移動 |
| 30 | 3 | 右側2地点と左上を使うcorner cycle |

01–11は手書きの基準例、12–30は次のコマンドで決定論的に再生成できます。

```bash
python3 examples/acceleration_sweep/generate_additional_scenarios.py
```

## 実行方法

```bash
cd /home/kakitsu/tel/lacam_primitive
python3 examples/acceleration_sweep/run_benchmark.py \
  --repeats 5 \
  --time-limit-ms 100 \
  --reference-time-limit-ms 1000
```

計測は逐次実行されます。次のファイルが更新されます。

- `benchmark_raw.csv`: 全反復の生データ
- `benchmark_summary.csv`: シナリオ・探索profileごとの中央値とmin/max
- `benchmark_details.json`: 各incumbentのcostと発見時刻を含む完全データ
- `outputs/acceleration_sweep/solutions/`: 各runのsolution YAML

plannerが出力する `search_ms` と各 improvement の `elapsed_ms` を使用するため、primitive/collision cache、transition cache、candidate cacheの構築時間は含みません。CPU負荷の影響を受けるので、絶対時間を比較するときは同じマシンで複数回測定してください。

## 判定

- 初期解「数ms」の機械的な閾値: median first solution ≤ 10 ms
- 品質の時限: search開始後50 ms
- `best-known`: 1,000 msの幅広探索と全100 ms profileで得た最小cost

ここでの `best-known` は最適性証明ではありません。結果と推奨profileは [RESULTS_JA.md](RESULTS_JA.md) を参照してください。

## 時系列データとアニメーション

計測後、各シナリオの代表runで発見された全incumbentを次のコマンドで書き出せます。

```bash
python3 examples/acceleration_sweep/export_artifacts.py
```

出力先は `artifacts/<scenario>/` です。各フォルダには次を保存します。

- `problem.yaml`: 使用した問題設定
- `solution.yaml`: 全incumbentのplanと発見時刻を含むplanner出力
- `solution_XX_cost_YY_waypoints.csv`: `macro_dt`ごとの厳密なplanner state、物理座標、heading、次primitive ID
- `solution_XX_cost_YY_samples.csv`: 0.05 s間隔の表示用補間poseと有限差分による速度・加速度
- `solution_XX_cost_YY_smooth.gif`: primitive内を補間したアニメーション
- `solution_XX_cost_YY_waypoints.gif`: planner waypointのみのアニメーション
- `manifest.json`: 採用profile、benchmark反復、cost、発見時刻、各ファイルの対応

全シナリオ横断の索引は `artifacts/manifest.csv` と `artifacts/manifest.json` です。`artifacts/scenario_poses.csv` には全agentのstart/goalをm・degree単位で一覧化しています。現在は30シナリオ、53 incumbentを保存しています。

`samples.csv` の `fd_*` 列は、表示用に補間したposeから有限差分で算出した参考値です。探索器が内部で判定した速度・加速度状態そのものではありません。制約判定の一次資料には `solution.yaml`、problemの制約値、plannerのkinodynamic validation結果を使用してください。
