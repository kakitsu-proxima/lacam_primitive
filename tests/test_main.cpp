#include <cassert>
#include <cmath>
#include <iostream>

#include "geometry.hpp"
#include "planner.hpp"
#include "yaml.hpp"

namespace {

lacam_primitive::Problem make_problem(
    bool transition_cache,
    bool candidate_cache,
    bool ara_star) {
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
  problem.agents = {
      Agent{State{2, 3, 0}, State{9, 3, 0}},
      Agent{State{6, 1, 1}, State{6, 6, 1}},
  };
  return problem;
}

void check_solution(
    bool transition_cache,
    bool candidate_cache,
    bool ara_star) {
  using namespace lacam_primitive;

  Problem problem = make_problem(
      transition_cache,
      candidate_cache,
      ara_star);
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
  assert(solution.stats.expanded_nodes > 0);

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

  check_solution(true, true, true);
  check_solution(true, false, true);
  check_solution(false, true, true);
  check_solution(false, false, false);

  std::cout << "tests passed\n";
  return 0;
}
