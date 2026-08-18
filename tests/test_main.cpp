#include <cassert>
#include <cmath>
#include <iostream>

#include "geometry.hpp"
#include "io.hpp"
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
  problem.primitive_config.rotation_bins = 1;
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
  assert(solution.stats.collision_mode == collision_mode);
  assert(solution.stats.max_boundary_travel_per_interval_m ==
         problem.search.max_boundary_travel_per_interval_m);
  assert(solution.stats.collision_interval_count > 0);
  assert(solution.stats.expanded_nodes > 0);
  assert(!solution.improvements.empty());
  assert(solution.improvements.back().plans.size() == solution.plans.size());

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

  // Regression: the 0.825 m x 0.275 m robot physically fits while rotating
  // in the middle of a 1.0 m-high workspace, even on a 0.10 m planning grid.
  // The old swept-cell representation expanded this into an 11-cell mask and
  // rejected every rotation in a 10-cell-high grid.
  Problem coarse;
  coarse.grid = GridSpec{0.1, 0.0, 0.0, 26, 10, 4, 0.2};
  coarse.robot = RobotSpec{0.825, 0.275, 3.0, 8.0, 0.01};
  coarse.primitive_config.translation_cells = {1, 2, 3, 4, 5};
  coarse.primitive_config.rotation_bins = 1;
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
  const Problem metric_coarse =
      load_problem(source_dir + "/examples/example.yaml");
  const Problem metric_fine =
      load_problem(source_dir + "/examples/example_cell_005.yaml");
  assert(metric_coarse.grid.has_metric_bounds);
  assert(metric_fine.grid.has_metric_bounds);
  assert(metric_coarse.grid.min_x_m == metric_fine.grid.min_x_m);
  assert(metric_coarse.grid.max_x_m == metric_fine.grid.max_x_m);
  assert(metric_coarse.agents.size() == metric_fine.agents.size());
  for (std::size_t i = 0; i < metric_coarse.agents.size(); ++i) {
    assert(std::abs(
               metric_coarse.grid.world_x(metric_coarse.agents[i].start.x) -
               metric_fine.grid.world_x(metric_fine.agents[i].start.x)) < 1e-12);
    assert(std::abs(
               metric_coarse.grid.world_y(metric_coarse.agents[i].start.y) -
               metric_fine.grid.world_y(metric_fine.agents[i].start.y)) < 1e-12);
    assert(metric_coarse.agents[i].start.heading ==
           metric_fine.agents[i].start.heading);
  }

  const PrimitiveTable metric_coarse_primitives(metric_coarse);
  const PrimitiveTable metric_fine_primitives(metric_fine);
  assert(metric_coarse_primitives.primitives().size() ==
         metric_fine_primitives.primitives().size());
  assert(metric_coarse_primitives.interval_count() ==
         metric_fine_primitives.interval_count());
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
  }

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

  std::cout << "tests passed\n";
  return 0;
}
