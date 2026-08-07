#include "io.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "yaml.hpp"

namespace lacam_primitive {
namespace {

using lacam_primitive::yaml::Node;

const Node* optional_child(const Node& node, const std::string& key) {
  return node.contains(key) ? &node.at(key) : nullptr;
}

std::vector<double> read_double_sequence(
    const Node& node,
    std::size_t expected,
    const std::string& name) {
  if (!node.is_sequence() || node.size() != expected) {
    throw std::runtime_error(
        name + " must be a sequence of length " +
        std::to_string(expected));
  }
  std::vector<double> result;
  result.reserve(expected);
  for (std::size_t i = 0; i < expected; ++i) {
    result.push_back(node.at(i).as_double());
  }
  return result;
}

std::vector<int> read_int_sequence(
    const Node& node,
    std::size_t expected,
    const std::string& name) {
  if (!node.is_sequence() || node.size() != expected) {
    throw std::runtime_error(
        name + " must be a sequence of length " +
        std::to_string(expected));
  }
  std::vector<int> result;
  result.reserve(expected);
  for (std::size_t i = 0; i < expected; ++i) {
    result.push_back(node.at(i).as_int());
  }
  return result;
}

State read_state(
    const Node& node,
    const std::string& name,
    int heading_bins) {
  const auto values = read_int_sequence(node, 3, name);
  return State{values[0], values[1], positive_mod(values[2], heading_bins)};
}

void validate_problem(const Problem& problem) {
  if (problem.grid.cell_size <= 0.0 || problem.grid.macro_dt <= 0.0) {
    throw std::runtime_error(
        "grid.cell_size and grid.macro_dt must be positive");
  }
  if (problem.grid.width_cells <= 0 ||
      problem.grid.height_cells <= 0 ||
      problem.grid.heading_bins <= 0) {
    throw std::runtime_error(
        "grid dimensions and heading_bins must be positive");
  }
  if (problem.robot.length <= 0.0 ||
      problem.robot.width <= 0.0 ||
      problem.robot.max_linear_velocity <= 0.0 ||
      problem.robot.max_angular_velocity <= 0.0 ||
      problem.robot.collision_padding < 0.0) {
    throw std::runtime_error("robot dimensions/limits are invalid");
  }
  if (problem.search.time_limit_ms <= 0.0 ||
      problem.search.initial_weight < 1.0 ||
      problem.search.minimum_weight < 1.0 ||
      problem.search.weight_step <= 0.0 ||
      problem.search.minimum_weight > problem.search.initial_weight) {
    throw std::runtime_error("search timing/weight settings are invalid");
  }
  for (std::size_t i = 0; i < problem.agents.size(); ++i) {
    for (const auto& item : std::vector<std::pair<std::string, State>>{
             {"start", problem.agents[i].start},
             {"goal", problem.agents[i].goal}}) {
      const std::string& label = item.first;
      const State& state = item.second;
      if (state.x < 0 || state.y < 0 ||
          state.x >= problem.grid.width_cells ||
          state.y >= problem.grid.height_cells) {
        throw std::runtime_error(
            "agent " + std::to_string(i) + " " + label +
            " is outside the grid");
      }
    }
  }
}

}  // namespace

Problem load_problem(const std::string& path) {
  const Node root = lacam_primitive::yaml::parse_file(path);
  Problem problem;

  const Node& grid = root.at("grid");
  problem.grid.cell_size = grid.at("cell_size").as_double();
  problem.grid.width_cells = grid.at("width_cells").as_int();
  problem.grid.height_cells = grid.at("height_cells").as_int();
  problem.grid.heading_bins = grid.at("heading_bins").as_int();
  problem.grid.macro_dt = grid.at("macro_dt").as_double();
  if (const Node* origin = optional_child(grid, "origin")) {
    const auto values = read_double_sequence(*origin, 2, "grid.origin");
    problem.grid.origin_x = values[0];
    problem.grid.origin_y = values[1];
  }

  const Node& robot = root.at("robot");
  const auto size = read_double_sequence(robot.at("size"), 2, "robot.size");
  problem.robot.length = size[0];
  problem.robot.width = size[1];
  problem.robot.max_linear_velocity =
      robot.at("max_linear_velocity").as_double();
  problem.robot.max_angular_velocity =
      robot.at("max_angular_velocity").as_double();
  if (const Node* padding = optional_child(robot, "collision_padding")) {
    problem.robot.collision_padding = padding->as_double();
  }

  const Node& primitives = root.at("primitives");
  const Node& translations = primitives.at("translation_cells");
  if (!translations.is_sequence() || translations.size() == 0) {
    throw std::runtime_error(
        "primitives.translation_cells must be a non-empty sequence");
  }
  problem.primitive_config.translation_cells.clear();
  for (std::size_t i = 0; i < translations.size(); ++i) {
    problem.primitive_config.translation_cells.push_back(
        translations.at(i).as_int());
  }
  problem.primitive_config.rotation_bins =
      primitives.at("rotation_bins").as_int();

  if (const Node* pivot = optional_child(primitives, "rotation_pivot_offsets_cells")) {
    if (!pivot->is_sequence() || pivot->size() == 0) {
      throw std::runtime_error(
          "primitives.rotation_pivot_offsets_cells must be a non-empty sequence");
    }
    
    problem.primitive_config.rotation_pivot_offsets_cells.clear();

    for (std::size_t i = 0; i < pivot->size(); ++i) {
      problem.primitive_config.rotation_pivot_offsets_cells.push_back(
          pivot->at(i).as_int());
    }
  }

  if (const Node* wait = optional_child(primitives, "include_wait")) {
    problem.primitive_config.include_wait = wait->as_bool();
  }

  if (const Node* search = optional_child(root, "search")) {
    if (const Node* value = optional_child(*search, "time_limit_ms")) {
      problem.search.time_limit_ms = value->as_double();
    }
    if (const Node* value = optional_child(*search, "anytime")) {
      problem.search.anytime = value->as_bool();
    }
    if (const Node* value = optional_child(*search, "initial_weight")) {
      problem.search.initial_weight = value->as_double();
    }
    if (const Node* value = optional_child(*search, "minimum_weight")) {
      problem.search.minimum_weight = value->as_double();
    }
    if (const Node* value = optional_child(*search, "weight_step")) {
      problem.search.weight_step = value->as_double();
    }
    if (const Node* value = optional_child(*search, "objective")) {
      problem.search.objective = value->as_string();
    }
    if (const Node* value = optional_child(*search, "max_expansions")) {
      problem.search.max_expansions = value->as_size();
    }
    if (const Node* value = optional_child(*search, "max_branching")) {
      problem.search.max_branching = value->as_size();
    }
    if (const Node* value =
            optional_child(*search, "alternatives_per_agent")) {
      problem.search.alternatives_per_agent = value->as_size();
    }
    if (const Node* value = optional_child(*search, "random_seed")) {
      problem.search.random_seed =
          static_cast<std::uint32_t>(value->as_size());
    }
    if (const Node* value =
            optional_child(*search, "use_transition_cache")) {
      problem.search.use_transition_cache = value->as_bool();
    }
    if (const Node* value =
            optional_child(*search, "use_candidate_cache")) {
      problem.search.use_candidate_cache = value->as_bool();
    }
    if (const Node* value = optional_child(*search, "use_ara_star")) {
      problem.search.use_ara_star = value->as_bool();
    }
  }

  if (const Node* obstacles = optional_child(root, "obstacles")) {
    if (!obstacles->is_sequence()) {
      throw std::runtime_error("obstacles must be a sequence");
    }
    for (std::size_t i = 0; i < obstacles->size(); ++i) {
      const Node& item = obstacles->at(i);
      const auto rect =
          read_int_sequence(item.at("rect"), 4, "obstacles[].rect");
      problem.obstacles.push_back(
          ObstacleRect{rect[0], rect[1], rect[2], rect[3]});
    }
  }

  const Node& agents = root.at("agents");
  if (!agents.is_sequence() || agents.size() == 0) {
    throw std::runtime_error("agents must be non-empty");
  }
  for (std::size_t i = 0; i < agents.size(); ++i) {
    const Node& item = agents.at(i);
    problem.agents.push_back(Agent{
        read_state(
            item.at("start"),
            "agents[].start",
            problem.grid.heading_bins),
        read_state(
            item.at("goal"),
            "agents[].goal",
            problem.grid.heading_bins)});
  }

  validate_problem(problem);
  return problem;
}

void write_solution(
    const std::string& path,
    const Problem& problem,
    const Solution& solution) {
  const std::filesystem::path output(path);
  if (!output.parent_path().empty()) {
    std::filesystem::create_directories(output.parent_path());
  }
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open output file: " + path);
  }

  stream << std::setprecision(12);
  stream << "success: " << (solution.success ? "true" : "false") << '\n';
  stream << "elapsed_ms: " << solution.elapsed_ms << '\n';
  stream << "objective: " << solution.objective << '\n';
  if (solution.success) stream << "cost: " << solution.cost << '\n';

  stream << "timing:\n";
  stream << "  primitive_collision_precompute_ms: "
         << solution.stats.primitive_collision_precompute_ms << '\n';
  stream << "  transition_cache_precompute_ms: "
         << solution.stats.transition_cache_precompute_ms << '\n';
  stream << "  static_precompute_ms: "
         << solution.stats.static_precompute_ms << '\n';
  stream << "  candidate_cache_precompute_ms: "
         << solution.stats.candidate_cache_precompute_ms << '\n';
  stream << "  query_precompute_ms: "
         << solution.stats.query_precompute_ms << '\n';
  stream << "  search_ms: " << solution.stats.search_ms << '\n';
  stream << "  warm_request_ms: "
         << solution.stats.warm_request_ms << '\n';
  stream << "  cold_total_ms: "
         << solution.stats.cold_total_ms << '\n';

  stream << "runtime:\n";
  stream << "  transition_cache_enabled: "
         << (solution.stats.transition_cache_enabled ? "true" : "false")
         << '\n';
  stream << "  candidate_cache_enabled: "
         << (solution.stats.candidate_cache_enabled ? "true" : "false")
         << '\n';
  stream << "  ara_star_enabled: "
         << (solution.stats.ara_star_enabled ? "true" : "false") << '\n';
  stream << "  transition_lookups: "
         << solution.stats.transition_lookups << '\n';
  stream << "  transition_cache_hits: "
         << solution.stats.transition_cache_hits << '\n';
  stream << "  transition_on_demand_computations: "
         << solution.stats.transition_on_demand_computations << '\n';
  stream << "  candidate_lookups: "
         << solution.stats.candidate_lookups << '\n';
  stream << "  candidate_cache_hits: "
         << solution.stats.candidate_cache_hits << '\n';
  stream << "  candidate_on_demand_computations: "
         << solution.stats.candidate_on_demand_computations << '\n';
  stream << "  expanded_nodes: "
         << solution.stats.expanded_nodes << '\n';

  stream << "grid:\n";
  stream << "  cell_size: " << problem.grid.cell_size << '\n';
  stream << "  origin: [" << problem.grid.origin_x << ", "
         << problem.grid.origin_y << "]\n";
  stream << "  heading_bins: " << problem.grid.heading_bins << '\n';
  stream << "  macro_dt: " << problem.grid.macro_dt << '\n';

  stream << "improvements:\n";
  for (const Improvement& improvement : solution.improvements) {
    stream << "  - elapsed_ms: " << improvement.elapsed_ms << '\n';
    stream << "    cost: " << improvement.cost << '\n';
    stream << "    weight: " << improvement.weight << '\n';
  }

  stream << "plans:\n";
  for (std::size_t agent = 0; agent < solution.plans.size(); ++agent) {
    stream << "  - agent: " << agent << '\n';
    stream << "    states:\n";
    for (const State& state : solution.plans[agent].states) {
      stream << "      - [" << state.x << ", " << state.y << ", "
             << state.heading << "]\n";
    }
    stream << "    primitive_ids: [";
    for (std::size_t i = 0;
         i < solution.plans[agent].primitive_ids.size();
         ++i) {
      if (i != 0) stream << ", ";
      stream << solution.plans[agent].primitive_ids[i];
    }
    stream << "]\n";
  }
}

}  // namespace lacam_primitive
