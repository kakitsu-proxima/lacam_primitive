#include <cassert>
#include <iostream>

#include "geometry.hpp"
#include "yaml.hpp"
#include "planner.hpp"

int main() {
  using namespace lacam_primitive;

  const auto yaml = lacam_primitive::yaml::parse(
      "root:\n"
      "  values: [1, 2, 3]\n"
      "items:\n"
      "  - start: [1, 2, 0]\n"
      "    goal: [3, 4, 1]\n");
  assert(yaml.at("root").at("values").at(2).as_int() == 3);
  assert(yaml.at("items").at(0).at("goal").at(1).as_int() == 4);

  const CellMask a = rasterize_oriented_rectangle(0.0, 0.0, 0.0, 0.4, 0.4);
  const CellMask b = rasterize_oriented_rectangle(0.0, 0.0, 0.0, 0.4, 0.4);
  assert(shifted_intersects(a, 0, 0, b, 0, 0));
  assert(!shifted_intersects(a, 0, 0, b, 5, 0));

  Problem problem;
  problem.grid = GridSpec{0.1, 0.0, 0.0, 12, 8, 4, 0.2};
  problem.robot = RobotSpec{0.08, 0.08, 1.0, 8.0, 0.0};
  problem.primitive_config.translation_cells = {1, 2};
  problem.primitive_config.rotation_bins = 1;
  problem.search.time_limit_ms = 500.0;
  problem.search.anytime = false;
  problem.search.max_branching = 24;
  problem.agents = {
      Agent{State{2, 3, 0}, State{9, 3, 0}},
      Agent{State{6, 1, 1}, State{6, 6, 1}},
  };

  Planner planner(problem);
  const Solution solution = planner.solve();
  assert(solution.success);
  assert(solution.plans.size() == 2);
  assert(solution.plans[0].states.back() == problem.agents[0].goal);
  assert(solution.plans[1].states.back() == problem.agents[1].goal);
  std::cout << "tests passed\n";
  return 0;
}
