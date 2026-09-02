#include "io.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "primitives.hpp"
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
    const Problem& problem) {
  const auto values = read_double_sequence(node, 3, name);
  const double rounded_heading = std::round(values[2]);
  if (std::abs(values[2] - rounded_heading) > 1e-8) {
    throw std::runtime_error(name + " heading must be an integer bin");
  }
  const int heading = positive_mod(
      static_cast<int>(std::llround(rounded_heading)),
      problem.grid.heading_bins);
  const double raw_x = problem.grid.cell_x(values[0]) -
                       problem.heading_anchor_x_cells(heading);
  const double raw_y = problem.grid.cell_y(values[1]) -
                       problem.heading_anchor_y_cells(heading);
  const State nearest{
      static_cast<int>(std::llround(raw_x)),
      static_cast<int>(std::llround(raw_y)),
      heading};
  const double dx_m = problem.world_x(nearest) - values[0];
  const double dy_m = problem.world_y(nearest) - values[1];
  const double snap_distance_m = std::hypot(dx_m, dy_m);
  const double floating_point_slack = 1e-8 * problem.grid.cell_size;
  if (snap_distance_m >
      problem.grid.pose_snap_tolerance_m + floating_point_slack) {
    std::ostringstream message;
    message << std::setprecision(12) << name << " requested centre pose ["
            << values[0] << ", " << values[1] << ", " << heading
            << "] is not on the planning pose lattice. The nearest "
               "representable centre is ["
            << problem.world_x(nearest) << ", " << problem.world_y(nearest)
            << ", " << heading << "] (distance " << snap_distance_m
            << " m), exceeding grid.pose_snap_tolerance_m="
            << problem.grid.pose_snap_tolerance_m
            << " m. Increase that explicit metric tolerance, use a finer "
               "planning cell_size, or specify start_ref/goal_ref on "
               "grid.pose_reference.";
    throw std::runtime_error(message.str());
  }
  return nearest;
}

State read_reference_state(
    const Node& node,
    const std::string& name,
    const Problem& problem) {
  if (!problem.grid.has_pose_reference) {
    throw std::runtime_error(
        name + " requires grid.pose_reference");
  }
  const auto values = read_int_sequence(node, 3, name);
  const double heading_in_planning_bins =
      static_cast<double>(values[2]) *
      static_cast<double>(problem.grid.heading_bins) /
      static_cast<double>(problem.grid.pose_reference_heading_bins);
  const double rounded_heading = std::round(heading_in_planning_bins);
  if (std::abs(heading_in_planning_bins - rounded_heading) > 1e-8) {
    std::ostringstream message;
    message << name << " heading bin " << values[2] << " on the "
            << problem.grid.pose_reference_heading_bins
            << "-bin reference lattice is not representable with grid.heading_bins="
            << problem.grid.heading_bins;
    throw std::runtime_error(message.str());
  }
  const int heading = positive_mod(
      static_cast<int>(std::llround(rounded_heading)),
      problem.grid.heading_bins);

  // Reuse the metric projection path. The synthetic node is deliberately not
  // constructed here because yaml::Node has no public builder API.
  const double world_x = problem.grid.pose_reference_origin_x +
                         static_cast<double>(values[0]) *
                             problem.grid.pose_reference_cell_size;
  const double world_y = problem.grid.pose_reference_origin_y +
                         static_cast<double>(values[1]) *
                             problem.grid.pose_reference_cell_size;
  const double raw_x = problem.grid.cell_x(world_x) -
                       problem.heading_anchor_x_cells(heading);
  const double raw_y = problem.grid.cell_y(world_y) -
                       problem.heading_anchor_y_cells(heading);
  const State nearest{static_cast<int>(std::llround(raw_x)),
                      static_cast<int>(std::llround(raw_y)), heading};
  const double snap_distance_m = std::hypot(
      problem.world_x(nearest) - world_x,
      problem.world_y(nearest) - world_y);
  const double floating_point_slack = 1e-8 * problem.grid.cell_size;
  if (snap_distance_m >
      problem.grid.pose_snap_tolerance_m + floating_point_slack) {
    std::ostringstream message;
    message << std::setprecision(12) << name << " reference pose ["
            << values[0] << ", " << values[1] << ", " << values[2]
            << "] denotes metric centre [" << world_x << ", " << world_y
            << "] but its nearest planning-lattice centre is ["
            << problem.world_x(nearest) << ", " << problem.world_y(nearest)
            << "] (distance " << snap_distance_m
            << " m), exceeding grid.pose_snap_tolerance_m="
            << problem.grid.pose_snap_tolerance_m << " m.";
    throw std::runtime_error(message.str());
  }
  return nearest;
}

State read_agent_state(
    const Node& item,
    const std::string& cell_key,
    const std::string& metric_key,
    const std::string& reference_key,
    const std::string& name,
    const Problem& problem) {
  const bool has_cells = item.contains(cell_key);
  const bool has_metric = item.contains(metric_key);
  const bool has_reference = item.contains(reference_key);
  const int specified = static_cast<int>(has_cells) +
                        static_cast<int>(has_metric) +
                        static_cast<int>(has_reference);
  if (specified != 1) {
    throw std::runtime_error(
        name + " must specify exactly one of " + cell_key + ", " +
        metric_key + ", or " + reference_key);
  }
  if (has_metric) {
    return read_metric_state(
        item.at(metric_key), name + "." + metric_key, problem);
  }
  if (has_reference) {
    return read_reference_state(
        item.at(reference_key), name + "." + reference_key, problem);
  }
  return read_state(
      item.at(cell_key), name + "." + cell_key,
      problem.grid.heading_bins);
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
  if (problem.grid.pose_snap_tolerance_m < 0.0) {
    throw std::runtime_error("grid.pose_snap_tolerance_m must be non-negative");
  }
  if (problem.grid.has_pose_reference &&
      (problem.grid.pose_reference_cell_size <= 0.0 ||
       problem.grid.pose_reference_heading_bins <= 0)) {
    throw std::runtime_error(
        "grid.pose_reference cell_size and heading_bins must be positive");
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
      std::isnan(problem.robot.max_linear_acceleration) ||
      problem.robot.max_linear_acceleration <= 0.0 ||
      std::isnan(problem.robot.max_angular_acceleration) ||
      problem.robot.max_angular_acceleration <= 0.0 ||
      std::isnan(problem.robot.max_body_point_acceleration) ||
      problem.robot.max_body_point_acceleration <= 0.0 ||
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
  if (problem.search.max_branching == 0 ||
      problem.search.alternatives_per_agent == 0 ||
      problem.search.initial_candidate_width == 0) {
    throw std::runtime_error(
        "search branching and candidate widths must be positive");
  }
  if (problem.search.kinodynamic_lookahead_depth != 2 &&
      problem.search.kinodynamic_lookahead_depth != 3) {
    throw std::runtime_error(
        "search.kinodynamic_lookahead_depth must be 2 or 3");
  }
  if (problem.search.use_kinodynamic_lookahead &&
      (!problem.primitive_config.use_acceleration_constraints ||
       !problem.search.use_dynamics_aware_pibt)) {
    throw std::runtime_error(
        "kinodynamic lookahead requires acceleration constraints and "
        "dynamics-aware PIBT");
  }
  if (problem.primitive_config.use_pivot_anchor_lattice &&
      (problem.primitive_config.rotation_pivot_offsets_cells.size() != 1 ||
       problem.primitive_config.rotation_pivot_offsets_cells.front() == 0)) {
    throw std::runtime_error(
        "pivot anchor lattice requires exactly one non-zero pivot offset");
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
  problem.grid.pose_reference_origin_x = problem.grid.origin_x;
  problem.grid.pose_reference_origin_y = problem.grid.origin_y;
  if (const Node* value = optional_child(grid, "pose_snap_tolerance_m")) {
    problem.grid.pose_snap_tolerance_m = value->as_double();
  }
  if (const Node* reference = optional_child(grid, "pose_reference")) {
    if (!reference->is_map()) {
      throw std::runtime_error("grid.pose_reference must be a map");
    }
    problem.grid.has_pose_reference = true;
    problem.grid.pose_reference_cell_size =
        reference->at("cell_size").as_double();
    problem.grid.pose_reference_heading_bins =
        reference->at("heading_bins").as_int();
    if (const Node* origin = optional_child(*reference, "origin")) {
      const auto values =
          read_double_sequence(*origin, 2, "grid.pose_reference.origin");
      problem.grid.pose_reference_origin_x = values[0];
      problem.grid.pose_reference_origin_y = values[1];
    }
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
  if (const Node* value = optional_child(robot, "max_linear_acceleration")) {
    problem.robot.max_linear_acceleration = value->as_double();
  }
  if (const Node* value = optional_child(robot, "max_angular_acceleration")) {
    problem.robot.max_angular_acceleration = value->as_double();
  }
  if (const Node* value = optional_child(robot, "max_body_point_acceleration")) {
    problem.robot.max_body_point_acceleration = value->as_double();
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
  const Node& rotation_bins = primitives.at("rotation_bins");
  problem.primitive_config.rotation_bin_counts.clear();
  if (rotation_bins.is_sequence()) {
    if (rotation_bins.size() == 0) {
      throw std::runtime_error("primitives.rotation_bins must not be empty");
    }
    for (std::size_t i = 0; i < rotation_bins.size(); ++i) {
      problem.primitive_config.rotation_bin_counts.push_back(
          rotation_bins.at(i).as_int());
    }
  } else {
    problem.primitive_config.rotation_bin_counts.push_back(
        rotation_bins.as_int());
  }
  if (const Node* value = optional_child(
          primitives, "use_multiple_rotation_amounts")) {
    problem.primitive_config.use_multiple_rotation_amounts = value->as_bool();
  }
  if (const Node* value = optional_child(
          primitives, "use_acceleration_constraints")) {
    problem.primitive_config.use_acceleration_constraints = value->as_bool();
  }
  if (const Node* value = optional_child(
          primitives, "use_pivot_anchor_lattice")) {
    problem.primitive_config.use_pivot_anchor_lattice = value->as_bool();
  }

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

  if (problem.primitive_config.use_pivot_anchor_lattice) {
    const auto& offsets =
        problem.primitive_config.rotation_pivot_offsets_cells;
    if (offsets.size() != 1 || offsets.front() == 0) {
      throw std::runtime_error(
          "use_pivot_anchor_lattice requires exactly one non-zero "
          "rotation pivot offset");
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
    if (const Node* value = optional_child(
            *search, "initial_solution_max_branching")) {
      problem.search.initial_solution_max_branching = value->as_size();
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
    if (const Node* value = optional_child(
            *search, "use_reverse_bfs_heuristic")) {
      problem.search.use_reverse_bfs_heuristic = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "diversify_candidates")) {
      problem.search.diversify_candidates = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "use_aabb_broadphase")) {
      problem.search.use_aabb_broadphase = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "use_conflict_cache")) {
      problem.search.use_conflict_cache = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "use_lazy_successors")) {
      problem.search.use_lazy_successors = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "use_progressive_widening")) {
      problem.search.use_progressive_widening = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "initial_candidate_width")) {
      problem.search.initial_candidate_width = value->as_size();
    }
    if (const Node* value = optional_child(
            *search, "use_per_primitive_intervals")) {
      problem.search.use_per_primitive_intervals = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "use_dynamics_aware_pibt")) {
      problem.search.use_dynamics_aware_pibt = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "use_interval_dominance")) {
      problem.search.use_interval_dominance = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "use_kinodynamic_lookahead")) {
      problem.search.use_kinodynamic_lookahead = value->as_bool();
    }
    if (const Node* value = optional_child(
            *search, "kinodynamic_lookahead_depth")) {
      problem.search.kinodynamic_lookahead_depth = value->as_size();
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
            item, "start", "start_m", "start_ref", "agents[]", problem),
        read_agent_state(
            item, "goal", "goal_m", "goal_ref", "agents[]", problem)});
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
            stream << indent << "      - [" << problem.world_x(state)
                   << ", " << problem.world_y(state) << ", "
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
          if (!plans[agent].primitive_start_rates.empty()) {
            stream << indent << "    primitive_start_rate_ranges:\n";
            for (const ScalarInterval& interval :
                 plans[agent].primitive_start_rates) {
              stream << indent << "      - [" << interval.lower << ", "
                     << interval.upper << "]\n";
            }
            stream << indent << "    primitive_end_rate_ranges:\n";
            for (const ScalarInterval& interval :
                 plans[agent].primitive_end_rates) {
              stream << indent << "      - [" << interval.lower << ", "
                     << interval.upper << "]\n";
            }
          }
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
  stream << "  connection_rule_precompute_ms: "
         << solution.stats.connection_rule_precompute_ms << '\n';
  stream << "  kinodynamic_lookahead_precompute_ms: "
         << solution.stats.kinodynamic_lookahead_precompute_ms << '\n';
  stream << "  query_precompute_ms: "
         << solution.stats.query_precompute_ms << '\n';
  stream << "  search_ms: " << solution.stats.search_ms << '\n';
  if (solution.success) {
    stream << "  first_solution_ms: " << solution.stats.first_solution_ms << '\n';
    stream << "  first_solution_cost: " << solution.stats.first_solution_cost
           << '\n';
  }
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
  stream << "  reverse_bfs_heuristic_enabled: "
         << (solution.stats.reverse_bfs_heuristic_enabled ? "true" : "false")
         << '\n';
  stream << "  candidate_diversification_enabled: "
         << (solution.stats.candidate_diversification_enabled ? "true" : "false")
         << '\n';
  stream << "  aabb_broadphase_enabled: "
         << (solution.stats.aabb_broadphase_enabled ? "true" : "false") << '\n';
  stream << "  conflict_cache_enabled: "
         << (solution.stats.conflict_cache_enabled ? "true" : "false") << '\n';
  stream << "  lazy_successors_enabled: "
         << (solution.stats.lazy_successors_enabled ? "true" : "false") << '\n';
  stream << "  progressive_widening_enabled: "
         << (solution.stats.progressive_widening_enabled ? "true" : "false")
         << '\n';
  stream << "  initial_candidate_width: "
         << solution.stats.initial_candidate_width << '\n';
  stream << "  initial_solution_max_branching: "
         << solution.stats.initial_solution_max_branching << '\n';
  stream << "  per_primitive_intervals_enabled: "
         << (solution.stats.per_primitive_intervals_enabled ? "true" : "false")
         << '\n';
  stream << "  multiple_rotation_amounts_enabled: "
         << (solution.stats.multiple_rotation_amounts_enabled ? "true" : "false")
         << '\n';
  stream << "  acceleration_constraints_enabled: "
         << (solution.stats.acceleration_constraints_enabled ? "true" : "false")
         << '\n';
  stream << "  dynamics_aware_pibt_enabled: "
         << (solution.stats.dynamics_aware_pibt_enabled ? "true" : "false")
         << '\n';
  stream << "  interval_dominance_enabled: "
         << (solution.stats.interval_dominance_enabled ? "true" : "false")
         << '\n';
  stream << "  kinodynamic_lookahead_enabled: "
         << (solution.stats.kinodynamic_lookahead_enabled ? "true" : "false")
         << '\n';
  stream << "  kinodynamic_lookahead_depth: "
         << solution.stats.kinodynamic_lookahead_depth << '\n';
  stream << "  kinodynamic_lookahead_entry_count: "
         << solution.stats.kinodynamic_lookahead_entry_count << '\n';
  stream << "  pivot_anchor_lattice_enabled: "
         << (solution.stats.pivot_anchor_lattice_enabled ? "true" : "false")
         << '\n';
  stream << "  active_rotation_amount_count: "
         << solution.stats.active_rotation_amount_count << '\n';
  stream << "  collision_mode: " << solution.stats.collision_mode << '\n';
  stream << "  max_boundary_travel_per_interval_m: "
         << solution.stats.max_boundary_travel_per_interval_m << '\n';
  stream << "  collision_interval_count: "
         << solution.stats.collision_interval_count << '\n';
  stream << "  collision_interval_polygon_count: "
         << solution.stats.collision_interval_polygon_count << '\n';
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
  stream << "  low_level_constraint_nodes: "
         << solution.stats.low_level_constraint_nodes << '\n';
  stream << "  pibt_plan_calls: " << solution.stats.pibt_plan_calls << '\n';
  stream << "  pibt_assign_calls: " << solution.stats.pibt_assign_calls << '\n';
  stream << "  pibt_candidate_attempts: "
         << solution.stats.pibt_candidate_attempts << '\n';
  stream << "  pibt_kinematic_candidate_rejects: "
         << solution.stats.pibt_kinematic_candidate_rejects << '\n';
  stream << "  pibt_backtracks: " << solution.stats.pibt_backtracks << '\n';
  stream << "  kinematic_dominance_rejects: "
         << solution.stats.kinematic_dominance_rejects << '\n';
  stream << "  dynamic_prefilter_ms: "
         << solution.stats.dynamic_prefilter_ms << '\n';
  stream << "  dynamic_prefilter_calls: "
         << solution.stats.dynamic_prefilter_calls << '\n';
  stream << "  dynamic_candidate_evaluations: "
         << solution.stats.dynamic_candidate_evaluations << '\n';
  stream << "  geometry_candidate_count: "
         << solution.stats.geometry_candidate_count << '\n';
  stream << "  dynamic_candidate_count: "
         << solution.stats.dynamic_candidate_count << '\n';
  stream << "  post_pibt_kinematic_rejects: "
         << solution.stats.post_pibt_kinematic_rejects << '\n';
  stream << "  dominance_check_count: "
         << solution.stats.dominance_check_count << '\n';
  stream << "  cubic_relation_queries: "
         << solution.stats.cubic_relation_queries << '\n';
  stream << "  connection_rule_queries: "
         << solution.stats.connection_rule_queries << '\n';
  stream << "  kinodynamic_lookahead_score_queries: "
         << solution.stats.kinodynamic_lookahead_score_queries << '\n';
  stream << "  kinodynamic_lookahead_sequences_evaluated: "
         << solution.stats.kinodynamic_lookahead_sequences_evaluated << '\n';
  stream << "  kinodynamic_lookahead_feasible_sequences: "
         << solution.stats.kinodynamic_lookahead_feasible_sequences << '\n';
  stream << "  joint_moves_generated: "
         << solution.stats.joint_moves_generated << '\n';
  stream << "  joint_move_duplicates: "
         << solution.stats.joint_move_duplicates << '\n';
  stream << "  max_open_size: " << solution.stats.max_open_size << '\n';
  stream << "  lazy_successor_requests: "
         << solution.stats.lazy_successor_requests << '\n';
  stream << "  successor_continuations: "
         << solution.stats.successor_continuations << '\n';
  stream << "  progressive_widening_stages: "
         << solution.stats.progressive_widening_stages << '\n';
  stream << "  max_candidate_width: "
         << solution.stats.max_candidate_width << '\n';
  stream << "  conflict_calls: " << solution.stats.conflict_calls << '\n';
  stream << "  conflict_cache_hits: "
         << solution.stats.conflict_cache_hits << '\n';
  stream << "  conflict_cache_canonical_swaps: "
         << solution.stats.conflict_cache_canonical_swaps << '\n';
  stream << "  conflict_cache_entries: "
         << solution.stats.conflict_cache_entries << '\n';
  stream << "  whole_step_aabb_tests: "
         << solution.stats.whole_step_aabb_tests << '\n';
  stream << "  whole_step_aabb_rejects: "
         << solution.stats.whole_step_aabb_rejects << '\n';
  stream << "  interval_aabb_tests: "
         << solution.stats.interval_aabb_tests << '\n';
  stream << "  interval_aabb_rejects: "
         << solution.stats.interval_aabb_rejects << '\n';
  stream << "  polygon_sat_tests: " << solution.stats.polygon_sat_tests << '\n';
  stream << "  kinematic_validation_calls: "
         << solution.stats.kinematic_validation_calls << '\n';
  stream << "  kinematic_validation_failures: "
         << solution.stats.kinematic_validation_failures << '\n';
  stream << "  kinematic_search_restarts: "
         << solution.stats.kinematic_search_restarts << '\n';
  stream << "  kinematic_no_good_count: "
         << solution.stats.kinematic_no_good_count << '\n';

  stream << "grid:\n";
  stream << "  cell_size: " << problem.grid.cell_size << '\n';
  stream << "  origin: [" << problem.grid.origin_x << ", "
         << problem.grid.origin_y << "]\n";
  stream << "  heading_bins: " << problem.grid.heading_bins << '\n';
  stream << "  macro_dt: " << problem.grid.macro_dt << '\n';
  stream << "  pose_snap_tolerance_m: "
         << problem.grid.pose_snap_tolerance_m << '\n';
  if (problem.grid.has_pose_reference) {
    stream << "  pose_reference:\n";
    stream << "    cell_size: "
           << problem.grid.pose_reference_cell_size << '\n';
    stream << "    origin: [" << problem.grid.pose_reference_origin_x << ", "
           << problem.grid.pose_reference_origin_y << "]\n";
    stream << "    heading_bins: "
           << problem.grid.pose_reference_heading_bins << '\n';
  }
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

void write_trajectory_csv(
    const std::string& path,
    const Problem& problem,
    const Solution& solution) {
  if (!solution.success || solution.plans.empty()) {
    throw std::runtime_error(
        "cannot write a trajectory CSV without a successful solution");
  }

  const std::filesystem::path output_path(path);
  if (!output_path.parent_path().empty()) {
    std::filesystem::create_directories(output_path.parent_path());
  }
  std::ofstream stream(output_path);
  if (!stream) {
    throw std::runtime_error("cannot write trajectory CSV: " + path);
  }
  stream << std::setprecision(12);

  const PrimitiveTable primitives(problem);
  const std::size_t step_count =
      solution.plans.front().primitive_ids.size();
  for (const AgentPlan& plan : solution.plans) {
    if (plan.primitive_ids.size() != step_count ||
        plan.states.size() != step_count + 1) {
      throw std::runtime_error(
          "all plans must have one common macro-step count");
    }
  }

  const double map_left_m = problem.grid.has_metric_bounds
      ? problem.grid.min_x_m
      : problem.grid.origin_x - 0.5 * problem.grid.cell_size;
  const double map_bottom_m = problem.grid.has_metric_bounds
      ? problem.grid.min_y_m
      : problem.grid.origin_y - 0.5 * problem.grid.cell_size;
  const auto tuple2 = [](double first, double second) {
    std::ostringstream value;
    value << std::setprecision(12) << "\"(" << first << "," << second
          << ")\"";
    return value.str();
  };
  const auto tuple3 = [](double first, double second, double third) {
    std::ostringstream value;
    value << std::setprecision(12) << "\"(" << first << "," << second
          << "," << third << ")\"";
    return value.str();
  };
  const auto scaled_range = [&](const ScalarInterval& interval, double scale) {
    if (interval.empty() || scale <= 0.0) return tuple2(0.0, 0.0);
    return tuple2(interval.lower / scale, interval.upper / scale);
  };

  stream << "time_ms";
  for (std::size_t agent = 0; agent < solution.plans.size(); ++agent) {
    const std::string prefix = "agent_" + std::to_string(agent + 1) + "_";
    stream << ',' << prefix << "position_m"
           << ',' << prefix << "heading_deg"
           << ',' << prefix << "beta_start"
           << ',' << prefix << "beta_end"
           << ',' << prefix << "sdot_start_range_per_s"
           << ',' << prefix << "sdot_end_range_per_s";
  }
  stream << '\n';

  // One row per boundary. At non-terminal rows beta/rate fields describe the
  // segment starting at that boundary. The final row is the stopped goal.
  for (std::size_t boundary = 0; boundary <= step_count; ++boundary) {
    stream << boundary * problem.grid.macro_dt * 1000.0;
    for (const AgentPlan& plan : solution.plans) {
      const State& state = plan.states[boundary];
      const double x_m = problem.world_x(state) - map_left_m;
      const double y_m = problem.world_y(state) - map_bottom_m;
      const double heading_deg =
          static_cast<double>(state.heading) * 360.0 /
          static_cast<double>(problem.grid.heading_bins);

      Twist2D beta_start;
      Twist2D beta_end;
      ScalarInterval start_rate{0.0, 0.0};
      ScalarInterval end_rate{0.0, 0.0};
      double progress_displacement = 0.0;
      if (boundary < step_count) {
        const PrimitiveId primitive_id = plan.primitive_ids[boundary];
        const Primitive& primitive = primitives.primitive(primitive_id);
        const PrimitiveVariant& variant =
            primitives.variant(primitive_id, state.heading);
        progress_displacement = primitive.progress_envelope.displacement;
        if (progress_displacement > 0.0) {
          // Use one dimensionless phase s in [0,1] for every segment.
          // q=(x_m,y_m,theta_deg), hence beta=dq/ds has m/m/degree units.
          beta_start = Twist2D{
              variant.start_twist_per_progress_rate.vx *
                  progress_displacement,
              variant.start_twist_per_progress_rate.vy *
                  progress_displacement,
              variant.start_twist_per_progress_rate.omega *
                  progress_displacement * 180.0 / kPi};
          beta_end = Twist2D{
              variant.end_twist_per_progress_rate.vx *
                  progress_displacement,
              variant.end_twist_per_progress_rate.vy *
                  progress_displacement,
              variant.end_twist_per_progress_rate.omega *
                  progress_displacement * 180.0 / kPi};
          if (plan.primitive_start_rates.size() == step_count &&
              plan.primitive_end_rates.size() == step_count) {
            start_rate = plan.primitive_start_rates[boundary];
            end_rate = plan.primitive_end_rates[boundary];
          }
        }
      }

      stream << ',' << tuple2(x_m, y_m)
             << ',' << heading_deg
             << ',' << tuple3(beta_start.vx, beta_start.vy, beta_start.omega)
             << ',' << tuple3(beta_end.vx, beta_end.vy, beta_end.omega)
             << ',' << scaled_range(start_rate, progress_displacement)
             << ',' << scaled_range(end_rate, progress_displacement);
    }
    stream << '\n';
  }

  std::filesystem::path metadata_path = output_path;
  metadata_path.replace_extension(".metadata.yaml");
  std::ofstream metadata(metadata_path);
  if (!metadata) {
    throw std::runtime_error(
        "cannot write trajectory metadata: " + metadata_path.string());
  }
  metadata << std::setprecision(12)
           << "interface_schema: trajectory_interface.yaml\n"
           << "trajectory_csv: " << output_path.filename().string() << '\n'
           << "map_lower_left_world_m: [" << map_left_m << ", "
           << map_bottom_m << "]\n"
           << "reserved_collision_padding_m: "
           << problem.robot.collision_padding << '\n'
           << "guaranteed_center_translation_deviation_margin_m: "
           << problem.robot.collision_padding << '\n'
           << "margin_bound_is_strict: true\n"
           << "margin_conditions:\n"
           << "  - heading and body shape are unchanged\n"
           << "  - each body follows the planned space-time swept region\n"
           << "  - lower-level retiming is collision-checked again if it "
              "changes interval occupancy\n";
}

}  // namespace lacam_primitive
