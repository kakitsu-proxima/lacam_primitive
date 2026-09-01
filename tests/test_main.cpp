#include <cassert>
#include <cmath>
#include <iostream>
#include <set>

#include "geometry.hpp"
#include "io.hpp"
#include "kinematics.hpp"
#include "planner.hpp"
#include "yaml.hpp"

namespace {

lacam_primitive::Problem make_problem(
    bool transition_cache,
    bool candidate_cache,
    bool ara_star,
    const std::string& collision_mode = "time_indexed") {
  using namespace lacam_primitive;

  Problem problem;
  problem.grid = GridSpec{0.1, 0.0, 0.0, 12, 8, 4, 0.2};
  problem.robot = RobotSpec{0.08, 0.08, 1.0, 8.0, 0.0};
  problem.primitive_config.translation_cells = {1, 2};
  problem.primitive_config.rotation_bin_counts = {1};
  problem.search.time_limit_ms = 500.0;
  problem.search.anytime = true;
  problem.search.initial_weight = 2.0;
  problem.search.minimum_weight = 1.0;
  problem.search.weight_step = 0.5;
  problem.search.max_branching = 24;
  problem.search.use_transition_cache = transition_cache;
  problem.search.use_candidate_cache = candidate_cache;
  problem.search.use_ara_star = ara_star;
  problem.search.collision_mode = collision_mode;
  problem.agents = {
      Agent{State{2, 3, 0}, State{9, 3, 0}},
      Agent{State{6, 1, 1}, State{6, 6, 1}},
  };
  return problem;
}

void validate_solution(
    const lacam_primitive::Problem& problem,
    const lacam_primitive::Solution& solution) {
  using namespace lacam_primitive;
  assert(solution.success);
  PrimitiveTable primitives(problem);
  CollisionChecker collisions(problem, primitives);
  assert(solution.plans.size() == problem.agents.size());
  const std::size_t steps = solution.plans.front().primitive_ids.size();
  for (const AgentPlan& plan : solution.plans) {
    assert(plan.primitive_ids.size() == steps);
    assert(plan.states.size() == steps + 1);
  }
  for (std::size_t step = 0; step < steps; ++step) {
    for (std::size_t agent = 0; agent < solution.plans.size(); ++agent) {
      const AgentPlan& plan = solution.plans[agent];
      const PrimitiveId primitive = plan.primitive_ids[step];
      assert(collisions.statically_valid(plan.states[step], primitive));
      assert(primitives.apply(plan.states[step], primitive) ==
             plan.states[step + 1]);
      for (std::size_t other = agent + 1;
           other < solution.plans.size(); ++other) {
        assert(!collisions.conflict(
            plan.states[step], primitive,
            solution.plans[other].states[step],
            solution.plans[other].primitive_ids[step]));
      }
    }
  }
}

void check_solution(
    bool transition_cache,
    bool candidate_cache,
    bool ara_star,
    const std::string& collision_mode = "time_indexed") {
  using namespace lacam_primitive;

  Problem problem = make_problem(
      transition_cache,
      candidate_cache,
      ara_star,
      collision_mode);
  Planner planner(problem);
  const Solution solution = planner.solve();

  assert(solution.success);
  assert(solution.plans.size() == 2);
  assert(solution.plans[0].states.back() == problem.agents[0].goal);
  assert(solution.plans[1].states.back() == problem.agents[1].goal);
  assert(solution.elapsed_ms == solution.stats.search_ms);
  assert(solution.stats.transition_cache_enabled == transition_cache);
  assert(solution.stats.candidate_cache_enabled == candidate_cache);
  assert(solution.stats.ara_star_enabled == ara_star);
  assert(solution.stats.reverse_bfs_heuristic_enabled ==
         problem.search.use_reverse_bfs_heuristic);
  assert(solution.stats.candidate_diversification_enabled);
  assert(solution.stats.aabb_broadphase_enabled);
  assert(solution.stats.conflict_cache_enabled);
  assert(solution.stats.lazy_successors_enabled);
  assert(solution.stats.progressive_widening_enabled);
  assert(solution.stats.per_primitive_intervals_enabled);
  assert(!solution.stats.multiple_rotation_amounts_enabled);
  assert(!solution.stats.acceleration_constraints_enabled);
  assert(!solution.stats.pivot_anchor_lattice_enabled);
  assert(solution.stats.active_rotation_amount_count == 1);
  assert(solution.stats.collision_mode == collision_mode);
  assert(solution.stats.max_boundary_travel_per_interval_m ==
         problem.search.max_boundary_travel_per_interval_m);
  assert(solution.stats.collision_interval_count > 0);
  assert(solution.stats.collision_interval_polygon_count > 0);
  assert(solution.stats.expanded_nodes > 0);
  assert(solution.stats.low_level_constraint_nodes > 0);
  assert(solution.stats.pibt_plan_calls > 0);
  assert(solution.stats.pibt_assign_calls > 0);
  assert(solution.stats.max_open_size > 0);
  assert(solution.stats.lazy_successor_requests > 0);
  assert(solution.stats.max_candidate_width > 0);
  assert(solution.stats.conflict_calls > 0);
  assert(solution.stats.whole_step_aabb_tests > 0);
  assert(!solution.improvements.empty());
  assert(solution.stats.first_solution_ms ==
         solution.improvements.front().elapsed_ms);
  assert(solution.stats.first_solution_cost ==
         solution.improvements.front().cost);
  assert(solution.improvements.back().plans.size() == solution.plans.size());
  validate_solution(problem, solution);

  if (transition_cache) {
    assert(solution.stats.transition_cache_hits > 0);
    assert(solution.stats.transition_on_demand_computations == 0);
  } else {
    assert(solution.stats.transition_cache_hits == 0);
    assert(solution.stats.transition_on_demand_computations > 0);
  }

  if (candidate_cache) {
    assert(solution.stats.candidate_cache_hits > 0);
    assert(solution.stats.candidate_on_demand_computations == 0);
  } else {
    assert(solution.stats.candidate_cache_hits == 0);
    assert(solution.stats.candidate_on_demand_computations > 0);
  }
}

}  // namespace

int main() {
  using namespace lacam_primitive;

  const auto parsed = yaml::parse(
      "root:\n"
      "  values: [1, 2, 3]\n"
      "items:\n"
      "  - start: [1, 2, 0]\n"
      "    goal: [3, 4, 1]\n");
  assert(parsed.at("root").at("values").at(2).as_int() == 3);
  assert(parsed.at("items").at(0).at("goal").at(1).as_int() == 4);

  const CellMask a =
      rasterize_oriented_rectangle(0.0, 0.0, 0.0, 0.4, 0.4);
  const CellMask b =
      rasterize_oriented_rectangle(0.0, 0.0, 0.0, 0.4, 0.4);
  assert(shifted_intersects(a, 0, 0, b, 0, 0));
  assert(!shifted_intersects(a, 0, 0, b, 5, 0));

  const ConvexPolygon square = convex_hull({
      Point2{-0.4, -0.4}, Point2{0.4, -0.4},
      Point2{0.4, 0.4}, Point2{-0.4, 0.4}});
  assert(shifted_polygons_intersect(square, 0.0, 0.0, square, 0.5, 0.0));
  assert(!shifted_polygons_intersect(square, 0.0, 0.0, square, 2.0, 0.0));
  const AxisAlignedBounds square_bounds = polygon_bounds(square);
  assert(shifted_bounds_intersect(
      square_bounds, 0.0, 0.0, square_bounds, 0.5, 0.0));
  assert(!shifted_bounds_intersect(
      square_bounds, 0.0, 0.0, square_bounds, 2.0, 0.0));

  // Boundary speeds remain intervals: the planner need not commit to one
  // speed before the downstream continuous trajectory layer runs.
  const ConstantAccelerationEnvelope speed_envelope =
      constant_acceleration_envelope(0.1, 0.2, 1.0, 2.0);
  assert(speed_envelope.feasible());
  const ScalarInterval cubic_rest_to_rest =
      propagate_cubic_boundary_rates(
          0.5, 0.5, 3.0, 19.6, ScalarInterval{0.0, 0.0});
  assert(cubic_rest_to_rest.contains(0.0));
  const ScalarInterval cubic_too_fast =
      propagate_cubic_boundary_rates(
          0.5, 0.2, 3.0, 19.6, ScalarInterval{0.0, 0.0});
  assert(!cubic_too_fast.contains(0.0));
  const CubicBoundaryRateRelation precomputed_cubic =
      make_cubic_boundary_rate_relation(0.5, 0.5, 3.0, 19.6);
  assert(precomputed_cubic.feasible());
  assert(precomputed_cubic.vertex_count <=
         CubicBoundaryRateRelation::kMaximumVertices);
  for (const ScalarInterval incoming : {
           ScalarInterval{0.0, 0.0},
           ScalarInterval{0.0, 1.0},
           ScalarInterval{0.25, 2.25},
           ScalarInterval{2.0, 3.0}}) {
    const ScalarInterval direct = propagate_cubic_boundary_rates(
        0.5, 0.5, 3.0, 19.6, incoming);
    const ScalarInterval cached = precomputed_cubic.propagate(incoming);
    assert(direct.empty() == cached.empty());
    if (!direct.empty()) {
      assert(std::abs(direct.lower - cached.lower) < 1e-12);
      assert(std::abs(direct.upper - cached.upper) < 1e-12);
    }
  }
  assert(std::abs(speed_envelope.start_velocity.lower - 0.3) < 1e-12);
  assert(std::abs(speed_envelope.start_velocity.upper - 0.7) < 1e-12);
  assert(std::abs(speed_envelope.end_velocity.lower - 0.3) < 1e-12);
  assert(std::abs(speed_envelope.end_velocity.upper - 0.7) < 1e-12);
  const ScalarInterval stopped_to_moving = propagate(
      constant_acceleration_envelope(0.1, 0.2, 1.0, 5.0),
      ScalarInterval{0.0, 0.0});
  assert(stopped_to_moving.contains(1.0));
  assert(std::abs(stopped_to_moving.lower - 1.0) < 1e-12);
  assert(std::abs(stopped_to_moving.upper - 1.0) < 1e-12);

  const double corner_acceleration = max_rectangle_corner_acceleration(
      Point2{0.0, 0.0}, 0.37, 2.0, 0.0, 1.0, 0.5);
  const double expected_corner_acceleration =
      4.0 * 0.5 * std::hypot(1.0, 0.5);
  assert(std::abs(corner_acceleration - expected_corner_acceleration) < 1e-12);
  assert(std::abs(
             conservative_rectangle_acceleration_bound(
                 Point2{0.0, 0.0}, 2.0, 0.0, 1.0, 0.5) -
             expected_corner_acceleration) < 1e-12);

  // An eight-bin heading lattice can expose several rotation amounts at the
  // same dt. Center-pivot endpoints remain on the integer position lattice.
  Problem multi_rotation = make_problem(true, true, true);
  multi_rotation.grid.heading_bins = 8;
  multi_rotation.robot.max_angular_velocity = 20.0;
  multi_rotation.primitive_config.rotation_bin_counts = {1, 2, 3};
  multi_rotation.primitive_config.use_multiple_rotation_amounts = true;
  multi_rotation.primitive_config.rotation_pivot_offsets_cells = {0};
  const PrimitiveTable multi_rotation_primitives(multi_rotation);
  std::set<int> positive_rotation_amounts;
  for (const Primitive& primitive : multi_rotation_primitives.primitives()) {
    if (primitive.d_heading > 0) {
      positive_rotation_amounts.insert(primitive.d_heading);
      assert(primitive.progress_coordinate ==
             ProgressCoordinate::kRotationRadians);
      assert(primitive.progress_envelope.feasible());
      const PrimitiveVariant& variant =
          multi_rotation_primitives.variant(primitive.id, 0);
      assert(std::abs(variant.start_twist_per_progress_rate.vx) < 1e-12);
      assert(std::abs(variant.start_twist_per_progress_rate.vy) < 1e-12);
      assert(std::abs(variant.start_twist_per_progress_rate.omega - 1.0) <
             1e-12);
    }
  }
  assert((positive_rotation_amounts == std::set<int>{1, 2, 3}));
  assert(multi_rotation_primitives.max_rotation_bins() == 3);

  Problem single_rotation = multi_rotation;
  single_rotation.primitive_config.use_multiple_rotation_amounts = false;
  const PrimitiveTable single_rotation_primitives(single_rotation);
  std::set<int> single_rotation_amounts;
  for (const Primitive& primitive : single_rotation_primitives.primitives()) {
    if (primitive.d_heading > 0) {
      single_rotation_amounts.insert(primitive.d_heading);
    }
  }
  assert((single_rotation_amounts == std::set<int>{1}));
  assert(single_rotation_primitives.active_rotation_amount_count() == 1);

  Problem acceleration_toggle = make_problem(true, true, true);
  acceleration_toggle.robot.max_linear_acceleration = 2.0;
  acceleration_toggle.primitive_config.use_acceleration_constraints = true;
  const PrimitiveTable constrained_primitives(acceleration_toggle);
  acceleration_toggle.primitive_config.use_acceleration_constraints = false;
  const PrimitiveTable unconstrained_primitives(acceleration_toggle);
  for (const Primitive& constrained : constrained_primitives.primitives()) {
    if (constrained.dx != 1 || constrained.dy != 0) continue;
    const Primitive& unconstrained =
        unconstrained_primitives.primitive(constrained.id);
    assert(std::abs(constrained.progress_envelope.start_velocity.lower - 0.3) <
           1e-12);
    assert(std::abs(unconstrained.progress_envelope.start_velocity.lower) <
           1e-12);
  }

  // A repeated relative-state query is answered by the pairwise conflict
  // cache; the second call must not execute another AABB/SAT pipeline.
  Problem cache_problem = make_problem(true, true, true);
  PrimitiveTable cache_primitives(cache_problem);
  CollisionChecker cache_collisions(cache_problem, cache_primitives);
  cache_collisions.reset_runtime_stats();
  const bool first_conflict = cache_collisions.conflict(
      State{2, 3, 0}, cache_primitives.wait_id(),
      State{9, 3, 0}, cache_primitives.wait_id());
  const std::uint64_t tests_after_first =
      cache_collisions.whole_step_aabb_tests();
  const bool second_conflict = cache_collisions.conflict(
      State{9, 3, 0}, cache_primitives.wait_id(),
      State{2, 3, 0}, cache_primitives.wait_id());
  assert(first_conflict == second_conflict);
  assert(cache_collisions.conflict_calls() == 2);
  assert(cache_collisions.conflict_cache_hits() == 1);
  assert(cache_collisions.conflict_cache_entries() == 1);
  assert(cache_collisions.conflict_cache_canonical_swaps() > 0);
  assert(cache_collisions.whole_step_aabb_tests() == tests_after_first);

  PrimitiveId east_two = cache_primitives.wait_id();
  for (const Primitive& primitive : cache_primitives.primitives()) {
    if (primitive.dx == 2 && primitive.dy == 0 && primitive.d_heading == 0) {
      east_two = primitive.id;
      break;
    }
  }
  assert(east_two != cache_primitives.wait_id());
  assert(cache_primitives.variant(cache_primitives.wait_id(), 0)
             .interval_count !=
         cache_primitives.variant(east_two, 0).interval_count);
  assert(cache_collisions.conflict(
      State{4, 3, 0}, cache_primitives.wait_id(),
      State{2, 3, 0}, east_two));

  // Regression: the 0.825 m x 0.275 m robot physically fits while rotating
  // in the middle of a 1.0 m-high workspace, even on a 0.10 m planning grid.
  // The old swept-cell representation expanded this into an 11-cell mask and
  // rejected every rotation in a 10-cell-high grid.
  Problem coarse;
  coarse.grid = GridSpec{0.1, 0.0, 0.0, 26, 10, 4, 0.2};
  coarse.robot = RobotSpec{0.825, 0.275, 3.0, 8.0, 0.01};
  coarse.primitive_config.translation_cells = {1, 2, 3, 4, 5};
  coarse.primitive_config.rotation_bin_counts = {1};
  coarse.primitive_config.rotation_pivot_offsets_cells = {0};
  PrimitiveTable coarse_primitives(coarse);
  const std::size_t fine_interval_count = coarse_primitives.interval_count();
  Problem relaxed_coarse = coarse;
  relaxed_coarse.search.max_boundary_travel_per_interval_m = 0.02;
  PrimitiveTable relaxed_coarse_primitives(relaxed_coarse);
  assert(relaxed_coarse_primitives.interval_count() < fine_interval_count);
  CollisionChecker coarse_collisions(coarse, coarse_primitives);
  bool valid_rotation_found = false;
  for (const Primitive& primitive : coarse_primitives.primitives()) {
    if (primitive.d_heading != 0 &&
        coarse_collisions.statically_valid(State{12, 5, 0}, primitive.id)) {
      valid_rotation_found = true;
    }
  }
  assert(valid_rotation_found);

  // Metric YAML inputs describe the same physical problem at both planning
  // resolutions. Internal cell deltas differ, but all physical primitive
  // endpoints and collision results must agree.
  const std::string source_dir = LACAM_PRIMITIVE_SOURCE_DIR;
  const Problem rotation_45_problem =
      load_problem(source_dir + "/examples/kinematic_rotation_45.yaml");
  assert(rotation_45_problem.grid.heading_bins == 8);
  assert((rotation_45_problem.primitive_config.rotation_bin_counts ==
          std::vector<int>{1, 2, 3}));
  assert(rotation_45_problem.robot.max_linear_acceleration == 2.0);
  assert(rotation_45_problem.robot.max_angular_acceleration == 20.0);
  assert(rotation_45_problem.robot.max_body_point_acceleration == 30.0);
  assert(rotation_45_problem.primitive_config.use_multiple_rotation_amounts);
  assert(rotation_45_problem.primitive_config.use_acceleration_constraints);
  assert(rotation_45_problem.primitive_config.use_pivot_anchor_lattice);
  assert(rotation_45_problem.grid.pose_snap_tolerance_m == 0.02);
  assert((rotation_45_problem.agents.front().start == State{4, 6, 0}));
  assert((rotation_45_problem.agents.front().goal == State{16, 6, 3}));
  assert(std::hypot(
             rotation_45_problem.world_x(
                 rotation_45_problem.agents.front().goal) - 1.6,
             rotation_45_problem.world_y(
                 rotation_45_problem.agents.front().goal) - 0.6) <=
         rotation_45_problem.grid.pose_snap_tolerance_m);
  bool rejected_excessive_pose_snap = false;
  try {
    (void)load_problem(source_dir + "/tests/invalid_pose_snap.yaml");
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    rejected_excessive_pose_snap =
        message.find("nearest representable centre") != std::string::npos &&
        message.find("pose_snap_tolerance_m") != std::string::npos;
  }
  assert(rejected_excessive_pose_snap);
  const PrimitiveTable rotation_45_primitives(rotation_45_problem);
  assert(rotation_45_primitives.primitives().size() == 19);
  PrimitiveId rear_ccw_135 = rotation_45_primitives.wait_id();
  for (const Primitive& primitive : rotation_45_primitives.primitives()) {
    if (primitive.pivot_rotation && primitive.d_heading == 3) {
      rear_ccw_135 = primitive.id;
      break;
    }
  }
  assert(rear_ccw_135 != rotation_45_primitives.wait_id());
  const State before_anchor_rotation{21, 4, 0};
  const State after_anchor_rotation = rotation_45_primitives.apply(
      before_anchor_rotation, rear_ccw_135);
  assert((after_anchor_rotation == State{16, 6, 3}));
  const double rear_pivot_before_x =
      rotation_45_problem.world_x(before_anchor_rotation) - 0.3;
  const double rear_pivot_before_y =
      rotation_45_problem.world_y(before_anchor_rotation);
  const double end_yaw = 3.0 * rotation_45_problem.grid.heading_step();
  const double rear_pivot_after_x =
      rotation_45_problem.world_x(after_anchor_rotation) -
      0.3 * std::cos(end_yaw);
  const double rear_pivot_after_y =
      rotation_45_problem.world_y(after_anchor_rotation) -
      0.3 * std::sin(end_yaw);
  assert(std::abs(rear_pivot_before_x - rear_pivot_after_x) < 1e-12);
  assert(std::abs(rear_pivot_before_y - rear_pivot_after_y) < 1e-12);
  const CollisionChecker rotation_45_collisions(
      rotation_45_problem, rotation_45_primitives);
  assert(rotation_45_collisions.statically_valid(
      before_anchor_rotation, rear_ccw_135));

  Problem rotation_45_fine = rotation_45_problem;
  rotation_45_fine.grid.cell_size = 0.05;
  rotation_45_fine.grid.width_cells = 52;
  rotation_45_fine.grid.height_cells = 24;
  rotation_45_fine.primitive_config.translation_cells = {2, 4, 6};
  rotation_45_fine.primitive_config.rotation_pivot_offsets_cells = {-6};
  const PrimitiveTable rotation_45_fine_primitives(rotation_45_fine);
  const State fine_before_anchor_rotation{42, 8, 0};
  const State fine_after_anchor_rotation = rotation_45_fine_primitives.apply(
      fine_before_anchor_rotation, rear_ccw_135);
  assert((fine_after_anchor_rotation == State{32, 12, 3}));
  assert(std::abs(
             rotation_45_problem.world_x(after_anchor_rotation) -
             rotation_45_fine.world_x(fine_after_anchor_rotation)) < 1e-12);
  assert(std::abs(
             rotation_45_problem.world_y(after_anchor_rotation) -
             rotation_45_fine.world_y(fine_after_anchor_rotation)) < 1e-12);
  assert(rotation_45_primitives.interval_count() ==
         rotation_45_fine_primitives.interval_count());

  // Collision checks use physical heading anchors, not just integer state
  // indices. Differently headed agents with nearby anchor offsets still
  // conflict, while a sufficiently distant copy does not.
  assert(rotation_45_collisions.conflict(
      State{10, 6, 1}, rotation_45_primitives.wait_id(),
      State{10, 6, 7}, rotation_45_primitives.wait_id()));
  assert(!rotation_45_collisions.conflict(
      State{5, 6, 1}, rotation_45_primitives.wait_id(),
      State{20, 6, 7}, rotation_45_primitives.wait_id()));

  // Full three-agent regression based on examples/example.yaml. It preserves
  // all physical start/goal centres and orientations while enabling an
  // eight-heading, fixed rear-pivot lattice with 45/90/135-degree actions.
  const Problem anchor_three_agents = load_problem(
      source_dir + "/tests/example_anchor_45_three_agents.yaml");
  const Problem anchor_three_agents_fine = load_problem(
      source_dir + "/tests/example_anchor_45_three_agents_fine.yaml");
  const Problem base_three_agents =
      load_problem(source_dir + "/examples/example.yaml");
  assert(anchor_three_agents.agents.size() == 3);
  assert(anchor_three_agents_fine.agents.size() == 3);
  assert(anchor_three_agents.grid.heading_bins == 8);
  assert(anchor_three_agents.grid.has_pose_reference);
  assert(anchor_three_agents.grid.pose_reference_cell_size == 0.1);
  assert(anchor_three_agents.grid.pose_reference_heading_bins == 4);
  assert(anchor_three_agents.primitive_config.use_pivot_anchor_lattice);
  assert(anchor_three_agents.primitive_config.use_multiple_rotation_amounts);
  for (std::size_t i = 0; i < base_three_agents.agents.size(); ++i) {
    const State& base_start = base_three_agents.agents[i].start;
    const State& anchor_start = anchor_three_agents.agents[i].start;
    const State& fine_start = anchor_three_agents_fine.agents[i].start;
    const State& base_goal = base_three_agents.agents[i].goal;
    const State& anchor_goal = anchor_three_agents.agents[i].goal;
    const State& fine_goal = anchor_three_agents_fine.agents[i].goal;
    assert(std::abs(
               base_three_agents.world_x(base_start) -
               anchor_three_agents.world_x(anchor_start)) < 1e-12);
    assert(std::abs(
               base_three_agents.world_y(base_start) -
               anchor_three_agents.world_y(anchor_start)) < 1e-12);
    assert(std::abs(
               base_three_agents.world_x(base_goal) -
               anchor_three_agents.world_x(anchor_goal)) < 1e-12);
    assert(std::abs(
               base_three_agents.world_y(base_goal) -
               anchor_three_agents.world_y(anchor_goal)) < 1e-12);
    assert(std::abs(
               anchor_three_agents.world_x(anchor_start) -
               anchor_three_agents_fine.world_x(fine_start)) < 1e-12);
    assert(std::abs(
               anchor_three_agents.world_y(anchor_start) -
               anchor_three_agents_fine.world_y(fine_start)) < 1e-12);
    assert(std::abs(
               anchor_three_agents.world_x(anchor_goal) -
               anchor_three_agents_fine.world_x(fine_goal)) < 1e-12);
    assert(std::abs(
               anchor_three_agents.world_y(anchor_goal) -
               anchor_three_agents_fine.world_y(fine_goal)) < 1e-12);
    assert(std::abs(
               base_start.heading * base_three_agents.grid.heading_step() -
               anchor_start.heading * anchor_three_agents.grid.heading_step()) <
           1e-12);
    assert(std::abs(
               base_goal.heading * base_three_agents.grid.heading_step() -
               anchor_goal.heading * anchor_three_agents.grid.heading_step()) <
           1e-12);
    assert(std::abs(
               anchor_start.heading * anchor_three_agents.grid.heading_step() -
               fine_start.heading *
                   anchor_three_agents_fine.grid.heading_step()) < 1e-12);
    assert(std::abs(
               anchor_goal.heading * anchor_three_agents.grid.heading_step() -
               fine_goal.heading *
                   anchor_three_agents_fine.grid.heading_step()) < 1e-12);
  }
  const PrimitiveTable anchor_three_primitives(anchor_three_agents);
  Planner anchor_three_planner(anchor_three_agents);
  const Solution anchor_three_solution = anchor_three_planner.solve();
  validate_solution(anchor_three_agents, anchor_three_solution);
  assert(anchor_three_solution.plans.size() == 3);
  assert(anchor_three_solution.stats.pivot_anchor_lattice_enabled);
  assert(anchor_three_solution.stats.active_rotation_amount_count == 2);
  assert(anchor_three_solution.stats.acceleration_constraints_enabled);
  assert(anchor_three_solution.stats.dynamics_aware_pibt_enabled);
  assert(anchor_three_solution.stats.interval_dominance_enabled);
  assert(anchor_three_solution.stats.dynamic_prefilter_calls > 0);
  assert(anchor_three_solution.stats.connection_rule_precompute_ms > 0.0);
  assert(anchor_three_solution.stats.cubic_relation_queries > 0);
  assert(anchor_three_solution.stats.connection_rule_queries > 0);
  assert(anchor_three_solution.stats.dynamic_candidate_evaluations > 0);
  assert(anchor_three_solution.stats.dynamic_candidate_count <=
         anchor_three_solution.stats.geometry_candidate_count);
  assert(anchor_three_solution.stats.post_pibt_kinematic_rejects == 0);
  assert(anchor_three_solution.stats.kinematic_validation_calls >= 1);
  assert(anchor_three_solution.stats.kinematic_validation_failures == 0);

  Problem prefilter_ablation = anchor_three_agents;
  prefilter_ablation.search.use_dynamics_aware_pibt = false;
  Planner prefilter_ablation_planner(prefilter_ablation);
  const Solution prefilter_ablation_solution =
      prefilter_ablation_planner.solve();
  validate_solution(prefilter_ablation, prefilter_ablation_solution);
  assert(!prefilter_ablation_solution.stats.dynamics_aware_pibt_enabled);
  assert(prefilter_ablation_solution.stats.dynamic_prefilter_calls == 0);
  assert(prefilter_ablation_solution.stats.post_pibt_kinematic_rejects > 0);
  assert(std::abs(
             prefilter_ablation_solution.cost -
             anchor_three_solution.cost) < 1e-12);
  std::set<int> used_rotation_amounts;
  for (std::size_t i = 0; i < anchor_three_solution.plans.size(); ++i) {
    const AgentPlan& plan = anchor_three_solution.plans[i];
    assert(plan.states.front() == anchor_three_agents.agents[i].start);
    assert(plan.states.back() == anchor_three_agents.agents[i].goal);
    assert(plan.states.front().x != plan.states.back().x ||
           plan.states.front().y != plan.states.back().y);
    assert(plan.states.front().heading != plan.states.back().heading);
    for (PrimitiveId primitive_id : plan.primitive_ids) {
      const int rotation = std::abs(
          anchor_three_primitives.primitive(primitive_id).d_heading);
      if (rotation > 0) used_rotation_amounts.insert(rotation);
    }
  }
  assert(used_rotation_amounts.count(1) > 0);

  const Problem metric_coarse =
      load_problem(source_dir + "/examples/example.yaml");
  const Problem metric_fine =
      load_problem(source_dir + "/examples/example_cell_005.yaml");
  assert(metric_coarse.grid.has_metric_bounds);
  assert(metric_fine.grid.has_metric_bounds);
  assert(metric_coarse.grid.min_x_m == metric_fine.grid.min_x_m);
  assert(metric_coarse.grid.max_x_m == metric_fine.grid.max_x_m);
  assert(metric_coarse.grid.macro_dt == metric_fine.grid.macro_dt);
  assert(metric_coarse.robot.length == metric_fine.robot.length);
  assert(metric_coarse.robot.width == metric_fine.robot.width);
  assert(metric_coarse.robot.max_linear_velocity ==
         metric_fine.robot.max_linear_velocity);
  assert(metric_coarse.robot.max_angular_velocity ==
         metric_fine.robot.max_angular_velocity);
  assert(metric_coarse.robot.collision_padding ==
         metric_fine.robot.collision_padding);
  assert(metric_coarse.search.initial_weight ==
         metric_fine.search.initial_weight);
  assert(metric_coarse.search.weight_step == metric_fine.search.weight_step);
  assert(metric_coarse.search.max_branching ==
         metric_fine.search.max_branching);
  assert(metric_coarse.search.alternatives_per_agent ==
         metric_fine.search.alternatives_per_agent);
  assert(metric_coarse.agents.size() == metric_fine.agents.size());
  for (std::size_t i = 0; i < metric_coarse.agents.size(); ++i) {
    assert(std::abs(
               metric_coarse.world_x(metric_coarse.agents[i].start) -
               metric_fine.world_x(metric_fine.agents[i].start)) < 1e-12);
    assert(std::abs(
               metric_coarse.world_y(metric_coarse.agents[i].start) -
               metric_fine.world_y(metric_fine.agents[i].start)) < 1e-12);
    assert(metric_coarse.agents[i].start.heading ==
           metric_fine.agents[i].start.heading);
    assert(std::abs(
               metric_coarse.world_x(metric_coarse.agents[i].goal) -
               metric_fine.world_x(metric_fine.agents[i].goal)) < 1e-12);
    assert(std::abs(
               metric_coarse.world_y(metric_coarse.agents[i].goal) -
               metric_fine.world_y(metric_fine.agents[i].goal)) < 1e-12);
    assert(metric_coarse.agents[i].goal.heading ==
           metric_fine.agents[i].goal.heading);
  }

  const PrimitiveTable metric_coarse_primitives(metric_coarse);
  const PrimitiveTable metric_fine_primitives(metric_fine);
  assert(metric_coarse_primitives.primitives().size() ==
         metric_fine_primitives.primitives().size());
  assert(metric_coarse_primitives.interval_count() ==
         metric_fine_primitives.interval_count());
  const PrimitiveVariant& wait_variant = metric_coarse_primitives.variant(
      metric_coarse_primitives.wait_id(), 0);
  assert(wait_variant.interval_count == 1);
  assert(wait_variant.interval_polygons.size() == 1);
  bool found_finer_primitive_partition = false;
  for (std::size_t id = 0;
       id < metric_coarse_primitives.primitives().size();
       ++id) {
    const Primitive& left = metric_coarse_primitives.primitives()[id];
    const Primitive& right = metric_fine_primitives.primitives()[id];
    assert(left.d_heading == right.d_heading);
    assert(std::abs(
               left.dx * metric_coarse.grid.cell_size -
               right.dx * metric_fine.grid.cell_size) < 1e-12);
    assert(std::abs(
               left.dy * metric_coarse.grid.cell_size -
               right.dy * metric_fine.grid.cell_size) < 1e-12);
    assert(std::abs(
               left.pivot_offset_cells * metric_coarse.grid.cell_size -
               right.pivot_offset_cells * metric_fine.grid.cell_size) < 1e-12);
    if (metric_coarse_primitives.variant(left.id, 0).interval_count > 1) {
      found_finer_primitive_partition = true;
    }
  }
  assert(found_finer_primitive_partition);
  Problem global_interval_problem = metric_coarse;
  global_interval_problem.search.use_per_primitive_intervals = false;
  const PrimitiveTable global_interval_primitives(global_interval_problem);
  assert(metric_coarse_primitives.total_variant_intervals() <
         global_interval_primitives.total_variant_intervals());

  Planner metric_coarse_planner(metric_coarse);
  const Solution metric_coarse_solution = metric_coarse_planner.solve();
  Planner metric_fine_planner(metric_fine);
  const Solution metric_fine_solution = metric_fine_planner.solve();
  validate_solution(metric_coarse, metric_coarse_solution);
  validate_solution(metric_fine, metric_fine_solution);
  assert(std::abs(metric_coarse_solution.cost - metric_fine_solution.cost) <
         1e-12);

  const Problem pivot_coarse_problem =
      load_problem(source_dir + "/tests/metric_pivot_coarse.yaml");
  const Problem pivot_fine_problem =
      load_problem(source_dir + "/tests/metric_pivot_fine.yaml");
  const PrimitiveTable pivot_coarse(pivot_coarse_problem);
  const PrimitiveTable pivot_fine(pivot_fine_problem);
  assert(pivot_coarse_problem.metric_obstacles.size() == 1);
  assert(pivot_fine_problem.metric_obstacles.size() == 1);
  assert(pivot_coarse.primitives().size() == pivot_fine.primitives().size());
  assert(pivot_coarse.interval_count() == pivot_fine.interval_count());
  for (std::size_t id = 0; id < pivot_coarse.primitives().size(); ++id) {
    const Primitive& left = pivot_coarse.primitives()[id];
    const Primitive& right = pivot_fine.primitives()[id];
    assert(std::abs(
               left.pivot_offset_cells * pivot_coarse_problem.grid.cell_size -
               right.pivot_offset_cells * pivot_fine_problem.grid.cell_size) <
           1e-12);
    for (int heading = 0; heading < 4; ++heading) {
      const State left_delta = pivot_coarse.variant(left.id, heading).delta;
      const State right_delta = pivot_fine.variant(right.id, heading).delta;
      assert(std::abs(
                 left_delta.x * pivot_coarse_problem.grid.cell_size -
                 right_delta.x * pivot_fine_problem.grid.cell_size) < 1e-12);
      assert(std::abs(
                 left_delta.y * pivot_coarse_problem.grid.cell_size -
                 right_delta.y * pivot_fine_problem.grid.cell_size) < 1e-12);
    }
  }
  const CollisionChecker pivot_coarse_collision(
      pivot_coarse_problem, pivot_coarse);
  const CollisionChecker pivot_fine_collision(
      pivot_fine_problem, pivot_fine);
  for (std::size_t id = 0; id < pivot_coarse.primitives().size(); ++id) {
    assert(pivot_coarse_collision.statically_valid(
               pivot_coarse_problem.agents[0].start,
               static_cast<PrimitiveId>(id)) ==
           pivot_fine_collision.statically_valid(
               pivot_fine_problem.agents[0].start,
               static_cast<PrimitiveId>(id)));
  }

  bool rejected_unrepresentable_metric_pivot = false;
  try {
    (void)load_problem(
        source_dir + "/tests/invalid_metric_pivot.yaml");
  } catch (const std::runtime_error&) {
    rejected_unrepresentable_metric_pivot = true;
  }
  assert(rejected_unrepresentable_metric_pivot);

  check_solution(true, true, true);
  check_solution(true, false, true);
  check_solution(false, true, true);
  check_solution(false, false, false);
  check_solution(true, true, true, "whole_step");

  // Reverse BFS is optional and must preserve solvability and final cost.
  Problem geometric_problem = make_problem(true, true, true);
  geometric_problem.search.use_reverse_bfs_heuristic = false;
  Planner geometric_planner(geometric_problem);
  const Solution geometric_solution = geometric_planner.solve();
  Problem reverse_problem = make_problem(true, true, true);
  reverse_problem.search.use_reverse_bfs_heuristic = true;
  Planner reverse_planner(reverse_problem);
  const Solution reverse_solution = reverse_planner.solve();
  assert(geometric_solution.success && reverse_solution.success);
  assert(std::abs(geometric_solution.cost - reverse_solution.cost) < 1e-12);
  assert(!geometric_solution.stats.reverse_bfs_heuristic_enabled);
  assert(reverse_solution.stats.reverse_bfs_heuristic_enabled);

  Problem pivot_search_problem = make_problem(true, true, true);
  pivot_search_problem.primitive_config.rotation_pivot_offsets_cells = {-1, 0, 1};
  pivot_search_problem.search.use_reverse_bfs_heuristic = true;
  pivot_search_problem.search.initial_candidate_width = 4;
  Planner pivot_search_planner(pivot_search_problem);
  const Solution pivot_search_solution = pivot_search_planner.solve();
  validate_solution(pivot_search_problem, pivot_search_solution);

  Problem eager_problem = make_problem(true, true, true);
  eager_problem.search.use_lazy_successors = false;
  eager_problem.search.use_progressive_widening = false;
  Planner eager_planner(eager_problem);
  const Solution eager_solution = eager_planner.solve();
  assert(eager_solution.success);
  assert(!eager_solution.stats.lazy_successors_enabled);
  assert(!eager_solution.stats.progressive_widening_enabled);

  std::cout << "tests passed\n";
  return 0;
}
