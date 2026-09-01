# LaCAM + PIBT + primitives

A dependency-free C++17 reference implementation of a deliberately small planning stack:

- integer grid states `[x_cell, y_cell, heading_bin]`
- fixed-duration motion primitives
- time-indexed swept-cell collision masks
- PIBT-style priority inheritance for one joint move
- LaCAM-style high-level search with a low-level constraint tree
- anytime weighted search (first solution, then strict-cost improvements)
- YAML input/output through a small built-in parser
- Jupyter/Pillow GIF visualization

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

The `three_agent_anchor_45` CTest uses the same physical start/goal poses and
robot/workspace dimensions as `examples/example.yaml`, but with eight headings,
a fixed `-0.3 m` rear-pivot anchor, and 45/90/135-degree rotations. Besides the
standalone solve, the unit regression verifies all three physical start/goal
poses, position and heading changes, continuous pairwise collision validity,
and actual use of a 45-degree primitive.

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

`primitives.rotation_bins` accepts either the historical scalar or a list. For
an eight-bin (45-degree) heading lattice, for example:

```yaml
grid:
  heading_bins: 8
  macro_dt: 0.4
robot:
  max_linear_acceleration: 2.0       # optional, m/s^2
  max_angular_acceleration: 20.0     # optional, rad/s^2
  max_body_point_acceleration: 30.0  # optional, m/s^2
primitives:
  rotation_bins: [1, 2, 3]           # 45, 90 and 135 degrees, both signs
  use_multiple_rotation_amounts: true
  use_acceleration_constraints: true
```

Both features are independently switchable without editing the configured
limits or rotation list:

```bash
./build/lacam_primitive \
  --input examples/kinematic_rotation_45.yaml \
  --output outputs/kinematic.yaml \
  --multiple-rotation-amounts off \
  --acceleration-constraints off
```

With multiple rotations off, only the smallest configured positive
`rotation_bins` value is generated. With acceleration constraints off, the
boundary-rate envelopes retain the velocity limits but ignore linear/angular
acceleration limits. The rigid-body acceleration helper remains available;
sequence-wide search enforcement is still the integration step described
below.

With acceleration constraints enabled, the search carries a reachable scalar
boundary-rate interval for every agent. Each primitive uses a cubic Hermite
time scaling: waypoint rates remain intervals, while acceleration varies
linearly inside the step. Candidates whose propagated interval is empty are
removed before PIBT branching, and a completed solution is validated again
from stationary start to stationary goal. Fixed-pivot rotations conservatively
combine tangential and centripetal acceleration so every point of the physical
rectangle remains below `max_body_point_acceleration`.

The dynamics-aware PIBT prefilter caches both the filtered candidate ordering
and each candidate's outgoing rate interval. The two performance changes can
be ablated without disabling acceleration correctness:

```yaml
search:
  use_dynamics_aware_pibt: true
  use_interval_dominance: true
```

or with `--dynamics-aware-pibt on|off` and
`--interval-dominance on|off`. With the prefilter off, the historical
post-PIBT propagation path is used. Solution statistics include prefilter
time/call counts, geometry and dynamic candidate totals, post-PIBT rejects,
and dominance comparisons.

The cubic feasible `(start_rate, end_rate)` region is constructed once per
primitive as a fixed-capacity convex polygon. Runtime propagation projects the
intersection with the current input-rate interval without rebuilding or heap
allocating the polygon. Boundary twist compatibility is likewise precomputed
for every `(connection heading, previous primitive, next primitive)` tuple;
runtime work is a small rule lookup and interval transform.

The lower-level constant-acceleration envelope API is retained for experiments,
but it is not the sequence model used by the planner: one constant acceleration
cannot move a non-zero distance from rest to rest within one step. Cubic time
scaling avoids that artificial restriction while leaving boundary velocities
undecided for the downstream continuous trajectory layer.

For a single fixed non-zero pivot, enable the heading-dependent pivot-anchor
lattice:

```yaml
primitives:
  rotation_pivot_offsets_m: [-0.3]
  use_pivot_anchor_lattice: true
```

The integer `(x, y)` then identifies an anchor class rather than the physical
centre directly. Each heading has a fractional metric offset, and `states_m`
in the solution remains the authoritative physical centre pose. This
represents off-centre 45-degree endpoints without snapping and without a
sub-cell state-count multiplier. The mode intentionally requires exactly one
non-zero pivot offset; several independently selectable pivots do not share a
single closed anchor lattice in general.

For experiments that change the planning `cell_size`, keep agent scenarios on
a separate, human-facing reference lattice:

```yaml
grid:
  cell_size: 0.05                 # planning resolution under test
  heading_bins: 8
  pose_reference:
    cell_size: 0.1                # fixed scenario convention
    origin: [0.0, 0.0]
    heading_bins: 4
  pose_snap_tolerance_m: 0.02     # explicit physical projection limit
agents:
  - start_ref: [5, 2, 0]
    goal_ref: [23, 5, 1]
```

Here the reference goal always means centre `(2.3 m, 0.5 m)` at 90 degrees.
On an eight-bin planning lattice its heading becomes bin 2 automatically.
With a non-centre pivot, the exact centre may lie between planning anchors;
loading succeeds only when the nearest anchor is within
`pose_snap_tolerance_m`. `start_m`/`goal_m` use the same explicit tolerance,
so values such as `[1.6, 0.6, 3]` need not be written as long trigonometric
decimals. The emitted `states_m` values are the authoritative poses actually
used by planning.

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

It currently assumes:

- one rectangular robot model shared by all agents
- a 2-D integer occupancy grid
- global-axis translation primitives and in-place rotations
- synchronized fixed-duration primitives
- exact start and goal grid states`
