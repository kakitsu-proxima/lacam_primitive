#include "io.hpp"

#include <cmath>
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

int exact_cell_value(
    double value_m,
    double origin_m,
    double cell_size,
    const std::string& name) {
  const double cells = (value_m - origin_m) / cell_size;
  const double rounded = std::round(cells);
  if (std::abs(cells - rounded) > 1e-8) {
    std::ostringstream message;
    message << name << "=" << value_m
            << " m is not representable on the " << cell_size
            << " m integer state lattice (cell coordinate " << cells
            << "). Choose a physically common lattice point or use a finer "
               "cell_size.";
    throw std::runtime_error(message.str());
  }
  return static_cast<int>(std::llround(rounded));
}

int exact_delta_cells(
    double distance_m,
    double cell_size,
    const std::string& name) {
  return exact_cell_value(distance_m, 0.0, cell_size, name);
}

State read_metric_state(
    const Node& node,
    const std::string& name,
    const GridSpec& grid) {
  const auto values = read_double_sequence(node, 3, name);
  const double rounded_heading = std::round(values[2]);
  if (std::abs(values[2] - rounded_heading) > 1e-8) {
    throw std::runtime_error(name + " heading must be an integer bin");
  }
  return State{
      exact_cell_value(values[0], grid.origin_x, grid.cell_size, name + " x"),
      exact_cell_value(values[1], grid.origin_y, grid.cell_size, name + " y"),
      positive_mod(
          static_cast<int>(std::llround(rounded_heading)),
          grid.heading_bins)};
}

State read_agent_state(
    const Node& item,
    const std::string& cell_key,
    const std::string& metric_key,
    const std::string& name,
    const GridSpec& grid) {
  const bool has_cells = item.contains(cell_key);
  const bool has_metric = item.contains(metric_key);
  if (has_cells == has_metric) {
    throw std::runtime_error(
        name + " must specify exactly one of " + cell_key + " or " +
        metric_key);
  }
  return has_metric
             ? read_metric_state(item.at(metric_key), name + "." + metric_key, grid)
             : read_state(
                   item.at(cell_key), name + "." + cell_key, grid.heading_bins);
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
  if (problem.grid.has_metric_bounds &&
      (problem.grid.max_x_m <= problem.grid.min_x_m ||
       problem.grid.max_y_m <= problem.grid.min_y_m)) {
    throw std::runtime_error("grid.bounds_m must have positive width and height");
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
  if (problem.search.collision_mode != "time_indexed" &&
      problem.search.collision_mode != "whole_step") {
    throw std::runtime_error(
        "search.collision_mode must be time_indexed or whole_step");
  }
  if (problem.search.max_boundary_travel_per_interval_m <= 0.0) {
    throw std::runtime_error(
        "search.max_boundary_travel_per_interval_m must be positive");
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
  problem.grid.heading_bins = grid.at("heading_bins").as_int();
  problem.grid.macro_dt = grid.at("macro_dt").as_double();
  if (problem.grid.cell_size <= 0.0) {
    throw std::runtime_error("grid.cell_size must be positive");
  }
  if (const Node* origin = optional_child(grid, "origin")) {
    const auto values = read_double_sequence(*origin, 2, "grid.origin");
    problem.grid.origin_x = values[0];
    problem.grid.origin_y = values[1];
  }
  if (const Node* bounds = optional_child(grid, "bounds_m")) {
    const auto values = read_double_sequence(*bounds, 4, "grid.bounds_m");
    problem.grid.has_metric_bounds = true;
    problem.grid.min_x_m = values[0];
    problem.grid.min_y_m = values[1];
    problem.grid.max_x_m = values[2];
    problem.grid.max_y_m = values[3];
  }

  const Node* width = optional_child(grid, "width_cells");
  const Node* height = optional_child(grid, "height_cells");
  if (width) {
    problem.grid.width_cells = width->as_int();
  } else if (problem.grid.has_metric_bounds) {
    problem.grid.width_cells = exact_delta_cells(
        problem.grid.max_x_m - problem.grid.min_x_m,
        problem.grid.cell_size,
        "grid bounds width");
  } else {
    throw std::runtime_error("grid.width_cells is required without grid.bounds_m");
  }
  if (height) {
    problem.grid.height_cells = height->as_int();
  } else if (problem.grid.has_metric_bounds) {
    problem.grid.height_cells = exact_delta_cells(
        problem.grid.max_y_m - problem.grid.min_y_m,
        problem.grid.cell_size,
        "grid bounds height");
  } else {
    throw std::runtime_error("grid.height_cells is required without grid.bounds_m");
  }
  if (problem.grid.has_metric_bounds) {
    const int expected_width = exact_delta_cells(
        problem.grid.max_x_m - problem.grid.min_x_m,
        problem.grid.cell_size,
        "grid bounds width");
    const int expected_height = exact_delta_cells(
        problem.grid.max_y_m - problem.grid.min_y_m,
        problem.grid.cell_size,
        "grid bounds height");
    if (problem.grid.width_cells != expected_width ||
        problem.grid.height_cells != expected_height) {
      throw std::runtime_error(
          "grid width_cells/height_cells must match bounds_m / cell_size");
    }
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
  const bool has_translation_cells = primitives.contains("translation_cells");
  const bool has_translation_m = primitives.contains("translation_distances_m");
  if (has_translation_cells == has_translation_m) {
    throw std::runtime_error(
        "primitives must specify exactly one of translation_cells or "
        "translation_distances_m");
  }
  const Node& translations = primitives.at(
      has_translation_m ? "translation_distances_m" : "translation_cells");
  if (!translations.is_sequence() || translations.size() == 0) {
    throw std::runtime_error(
        "primitive translations must be a non-empty sequence");
  }
  problem.primitive_config.translation_cells.clear();
  for (std::size_t i = 0; i < translations.size(); ++i) {
    problem.primitive_config.translation_cells.push_back(
        has_translation_m
            ? exact_delta_cells(
                  translations.at(i).as_double(),
                  problem.grid.cell_size,
                  "primitives.translation_distances_m[]")
            : translations.at(i).as_int());
  }
  problem.primitive_config.rotation_bins =
      primitives.at("rotation_bins").as_int();

  const bool has_pivot_cells = primitives.contains("rotation_pivot_offsets_cells");
  const bool has_pivot_m = primitives.contains("rotation_pivot_offsets_m");
  if (has_pivot_cells && has_pivot_m) {
    throw std::runtime_error(
        "specify only one of rotation_pivot_offsets_cells or "
        "rotation_pivot_offsets_m");
  }
  const Node* pivot =
      has_pivot_m
          ? optional_child(primitives, "rotation_pivot_offsets_m")
          : optional_child(primitives, "rotation_pivot_offsets_cells");
  if (pivot) {
    if (!pivot->is_sequence() || pivot->size() == 0) {
      throw std::runtime_error(
          "rotation pivot offsets must be a non-empty sequence");
    }
    
    problem.primitive_config.rotation_pivot_offsets_cells.clear();

    for (std::size_t i = 0; i < pivot->size(); ++i) {
      problem.primitive_config.rotation_pivot_offsets_cells.push_back(
          has_pivot_m
              ? exact_delta_cells(
                    pivot->at(i).as_double(),
                    problem.grid.cell_size,
                    "primitives.rotation_pivot_offsets_m[]")
              : pivot->at(i).as_int());
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
    if (const Node* value = optional_child(*search, "collision_mode")) {
      problem.search.collision_mode = value->as_string();
    }
    if (const Node* value = optional_child(
            *search, "max_boundary_travel_per_interval_m")) {
      problem.search.max_boundary_travel_per_interval_m =
          value->as_double();
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
      const bool has_rect = item.contains("rect");
      const bool has_rect_m = item.contains("rect_m");
      if (has_rect == has_rect_m) {
        throw std::runtime_error(
            "each obstacle must specify exactly one of rect or rect_m");
      }
      if (has_rect_m) {
        const auto rect = read_double_sequence(
            item.at("rect_m"), 4, "obstacles[].rect_m");
        if (rect[2] <= 0.0 || rect[3] <= 0.0) {
          throw std::runtime_error("obstacles[].rect_m size must be positive");
        }
        problem.metric_obstacles.push_back(
            MetricObstacleRect{rect[0], rect[1], rect[2], rect[3]});
      } else {
        const auto rect =
            read_int_sequence(item.at("rect"), 4, "obstacles[].rect");
        problem.obstacles.push_back(
            ObstacleRect{rect[0], rect[1], rect[2], rect[3]});
      }
    }
  }

  const Node& agents = root.at("agents");
  if (!agents.is_sequence() || agents.size() == 0) {
    throw std::runtime_error("agents must be non-empty");
  }
  for (std::size_t i = 0; i < agents.size(); ++i) {
    const Node& item = agents.at(i);
    problem.agents.push_back(Agent{
        read_agent_state(
            item, "start", "start_m", "agents[]", problem.grid),
        read_agent_state(
            item, "goal", "goal_m", "agents[]", problem.grid)});
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
  const auto write_plans =
      [&](const std::vector<AgentPlan>& plans, const std::string& indent) {
        stream << indent << "plans:\n";
        for (std::size_t agent = 0; agent < plans.size(); ++agent) {
          stream << indent << "  - agent: " << agent << '\n';
          stream << indent << "    states:\n";
          for (const State& state : plans[agent].states) {
            stream << indent << "      - [" << state.x << ", " << state.y
                   << ", " << state.heading << "]\n";
          }
          stream << indent << "    states_m:\n";
          for (const State& state : plans[agent].states) {
            stream << indent << "      - [" << problem.grid.world_x(state.x)
                   << ", " << problem.grid.world_y(state.y) << ", "
                   << state.heading << "]\n";
          }
          stream << indent << "    primitive_ids: [";
          for (std::size_t i = 0;
               i < plans[agent].primitive_ids.size();
               ++i) {
            if (i != 0) stream << ", ";
            stream << plans[agent].primitive_ids[i];
          }
          stream << "]\n";
        }
      };

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
  stream << "  collision_mode: " << solution.stats.collision_mode << '\n';
  stream << "  max_boundary_travel_per_interval_m: "
         << solution.stats.max_boundary_travel_per_interval_m << '\n';
  stream << "  collision_interval_count: "
         << solution.stats.collision_interval_count << '\n';
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
  if (problem.grid.has_metric_bounds) {
    stream << "  bounds_m: [" << problem.grid.min_x_m << ", "
           << problem.grid.min_y_m << ", " << problem.grid.max_x_m << ", "
           << problem.grid.max_y_m << "]\n";
  }

  stream << "improvements:\n";
  for (const Improvement& improvement : solution.improvements) {
    stream << "  - elapsed_ms: " << improvement.elapsed_ms << '\n';
    stream << "    cost: " << improvement.cost << '\n';
    stream << "    weight: " << improvement.weight << '\n';
    write_plans(improvement.plans, "    ");
  }

  write_plans(solution.plans, "");
}

}  // namespace lacam_primitive
