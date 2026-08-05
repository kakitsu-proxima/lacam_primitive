# miniLaCAM

A dependency-free C++20 reference implementation of a deliberately small planning stack:

- integer grid states `[x_cell, y_cell, heading_bin]`
- fixed-duration motion primitives
- time-indexed swept-cell collision masks
- PIBT-style priority inheritance for one joint move
- LaCAM-style high-level search with a low-level constraint tree
- anytime weighted search (first solution, then strict-cost improvements)
- YAML input/output through a small built-in parser
- Jupyter/Pillow GIF visualization

The code intentionally does **not** depend on dynoplan, dynobench, OMPL, FCL, Eigen, Boost, or yaml-cpp.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Run

```bash
./build/minilacam \
  --input examples/crossing.yaml \
  --output outputs/crossing_solution.yaml \
  --time-limit-ms 1000
```

The command exits with status `0` when a solution is found, `1` when the time limit expires without a solution, and `2` for invalid input or usage.

## Notebook / GIF

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r python/requirements.txt
jupyter lab notebooks/demo.ipynb
```

The notebook builds the executable, runs the example, writes `outputs/notebook/solution.yaml`, creates `outputs/notebook/animation.gif`, and displays it inline.

## Single coordinate system

The planner has exactly one spatial discretization:

```yaml
grid:
  cell_size: 0.10
  origin: [0.0, 0.0]
```

Every search state is an integer cell. `origin` is used only to convert cells to metric coordinates for external systems and visualization. It is not used to decide whether collision checking is available, so there is no off-grid fallback.

## Motion and velocity

All primitives last `grid.macro_dt`. A translation of `d` cells has speed

```text
speed = d * cell_size / macro_dt
```

and is rejected at startup if it exceeds `robot.max_linear_velocity`. Rotation is checked in the same way against `robot.max_angular_velocity`.

## Collision checking

For every primitive and starting heading, the robot footprint is rasterized into common time intervals. Each interval stores a swept cell mask. Two primitives conflict only when their masks overlap in the **same time interval**. Static obstacles use the same masks.

The number of intervals is derived from primitive displacement, rotation, footprint radius, and `cell_size`; it is not an independent grid setting. A small derived guard band covers motion between the three samples used in each interval.

## Anytime behavior

`search.initial_weight` is used for the first weighted search. After a solution is found, the planner restarts under a strict incumbent bound while reducing the weight by `search.weight_step` down to `search.minimum_weight`. Each lower-cost solution replaces the incumbent and is recorded under `improvements` in the output YAML.

Set:

```yaml
search:
  anytime: false
```

to stop at the first solution.

## Scope

This is a clean reference baseline, not a drop-in replacement for all of db-LaCAM. It currently assumes:

- one rectangular robot model shared by all agents
- a 2-D integer occupancy grid
- global-axis translation primitives and in-place rotations
- synchronized fixed-duration primitives
- exact start and goal grid states

These restrictions are deliberate: they make the collision semantics inspectable and remove the mixed continuous/discrete failure modes discussed in the original repository.
