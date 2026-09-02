#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef LACAM_PRIMITIVE_DEFAULT_TRANSITION_CACHE
#define LACAM_PRIMITIVE_DEFAULT_TRANSITION_CACHE 1
#endif

#ifndef LACAM_PRIMITIVE_DEFAULT_CANDIDATE_CACHE
#define LACAM_PRIMITIVE_DEFAULT_CANDIDATE_CACHE 1
#endif

#ifndef LACAM_PRIMITIVE_DEFAULT_ARA_STAR
#define LACAM_PRIMITIVE_DEFAULT_ARA_STAR 1
#endif

namespace lacam_primitive {

constexpr double kPi = 3.141592653589793238462643383279502884;

// for the rotation
inline int positive_mod(int value, int modulus) {
  const int result = value % modulus;
  return result < 0 ? result + modulus : result;
}

struct State {
  int x = 0;
  int y = 0;
  int heading = 0;

  bool operator==(const State& other) const noexcept {
    return x == other.x && y == other.y && heading == other.heading;
  }

  bool operator!=(const State& other) const noexcept {
    return !(*this == other);
  }
};

struct StateHash {
  std::size_t operator()(const State& state) const noexcept {
    std::size_t seed = static_cast<std::size_t>(state.x) * 0x9e3779b185ebca87ULL;
    seed ^= static_cast<std::size_t>(state.y) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    seed ^= static_cast<std::size_t>(state.heading) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct JointStateHash {
  std::size_t operator()(const std::vector<State>& states) const noexcept {
    std::size_t seed = 0;
    StateHash hasher;
    for (const State& state : states) {
      const std::size_t value = hasher(state);
      seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
    }
    return seed;
  }
};

struct GridSpec {
  // can be changed by yaml file
  double cell_size = 0.1; // meter
  double origin_x = 0.0; // meter
  double origin_y = 0.0; // meter
  int width_cells = 1;
  int height_cells = 1;
  int heading_bins = 4;
  double macro_dt = 0.2;

  // Optional human-facing scenario lattice. Agent poses expressed with
  // start_ref/goal_ref are converted to metric poses on this lattice first,
  // so changing the planning cell_size or heading_bins does not redefine the
  // scenario.
  bool has_pose_reference = false;
  double pose_reference_cell_size = 0.1;
  double pose_reference_origin_x = 0.0;
  double pose_reference_origin_y = 0.0;
  int pose_reference_heading_bins = 4;

  // Maximum metric displacement allowed when a requested centre pose must be
  // projected onto the heading-dependent planning lattice. Zero retains the
  // historical exact-only behaviour.
  double pose_snap_tolerance_m = 0.0;

  // Physical workspace boundary. When omitted by legacy inputs, collision
  // checking falls back to the historical outer half-cell boundary.
  bool has_metric_bounds = false;
  double min_x_m = 0.0;
  double min_y_m = 0.0;
  double max_x_m = 0.0;
  double max_y_m = 0.0;

  [[nodiscard]] double heading_step() const {
    return 2.0 * kPi / static_cast<double>(heading_bins);
  }

  [[nodiscard]] double world_x(int x) const {
    return origin_x + static_cast<double>(x) * cell_size;
  }

  [[nodiscard]] double world_y(int y) const {
    return origin_y + static_cast<double>(y) * cell_size;
  }

  [[nodiscard]] double cell_x(double world_x_m) const {
    return (world_x_m - origin_x) / cell_size;
  }

  [[nodiscard]] double cell_y(double world_y_m) const {
    return (world_y_m - origin_y) / cell_size;
  }

  [[nodiscard]] double collision_min_x_cells() const {
    return has_metric_bounds ? cell_x(min_x_m) : -0.5;
  }

  [[nodiscard]] double collision_min_y_cells() const {
    return has_metric_bounds ? cell_y(min_y_m) : -0.5;
  }

  [[nodiscard]] double collision_max_x_cells() const {
    return has_metric_bounds
               ? cell_x(max_x_m)
               : static_cast<double>(width_cells) - 0.5;
  }

  [[nodiscard]] double collision_max_y_cells() const {
    return has_metric_bounds
               ? cell_y(max_y_m)
               : static_cast<double>(height_cells) - 0.5;
  }
};

struct RobotSpec {
  double length = 0.2;
  double width = 0.2;
  double max_linear_velocity = 1.0;
  double max_angular_velocity = 8.0;
  double collision_padding = 0.0;

  // Optional dynamic limits. Infinity preserves the historical
  // velocity-only behaviour. These are metric/physical quantities and do not
  // change when grid.cell_size changes.
  double max_linear_acceleration =
      std::numeric_limits<double>::infinity();       // m/s^2
  double max_angular_acceleration =
      std::numeric_limits<double>::infinity();       // rad/s^2
  double max_body_point_acceleration =
      std::numeric_limits<double>::infinity();       // m/s^2
};

struct SearchOptions {
  double time_limit_ms = 1000.0;
  bool anytime = true;
  double initial_weight = 2.5;
  double minimum_weight = 1.0;
  double weight_step = 0.5;
  std::string objective = "sum_of_costs";
  std::size_t max_expansions = 200000;
  std::size_t max_branching = 24;
  // If positive and smaller than max_branching, run one first-solution pass
  // with this branching cap before the main anytime pass.
  std::size_t initial_solution_max_branching = 0;
  std::size_t alternatives_per_agent = 5;
  std::uint32_t random_seed = 7;

  // time_indexed: compare swept polygons from matching sub-intervals.
  // whole_step: compare one conservative polygon for the entire primitive.
  std::string collision_mode = "time_indexed";

  // Maximum physical travel of any robot-boundary point in one internal
  // collision interval. Smaller values produce more, tighter intervals.
  double max_boundary_travel_per_interval_m = 0.005;

  bool use_transition_cache = LACAM_PRIMITIVE_DEFAULT_TRANSITION_CACHE != 0;
  bool use_candidate_cache = LACAM_PRIMITIVE_DEFAULT_CANDIDATE_CACHE != 0;
  bool use_ara_star = LACAM_PRIMITIVE_DEFAULT_ARA_STAR != 0;
  // Exact in the single-agent graph, but not always the best ranking for the
  // coupled multi-agent PIBT search. Keep it as an explicit experiment.
  bool use_reverse_bfs_heuristic = false;
  bool diversify_candidates = true;
  bool use_aabb_broadphase = true;
  bool use_conflict_cache = true;
  bool use_lazy_successors = true;
  bool use_progressive_widening = true;
  std::size_t initial_candidate_width = 1;
  bool use_per_primitive_intervals = true;
  // Performance ablations. Both preserve acceleration-constraint semantics:
  // when the PIBT prefilter is off, propagation is performed after PIBT.
  bool use_dynamics_aware_pibt = true;
  bool use_interval_dominance = true;

};

struct PrimitiveConfig {
  std::vector<int> translation_cells{1};

  // Positive heading-bin counts. With heading_bins=8, {1,2,3} creates
  // +/-45, +/-90 and +/-135 degree rotations. A scalar `rotation_bins` in a
  // legacy YAML file is read as a one-element vector.
  std::vector<int> rotation_bin_counts{1};

  // Feature toggles used for controlled comparisons. When multiple rotation
  // amounts are disabled, only the smallest configured rotation is active.
  // When acceleration constraints are disabled, velocity envelopes ignore
  // acceleration bounds while retaining velocity bounds.
  bool use_multiple_rotation_amounts = false;
  bool use_acceleration_constraints = false;

  // Represent the integer state as a heading-dependent lattice anchor tied
  // to one fixed physical pivot. This permits exact (up to floating-point
  // evaluation of sin/cos) off-centre 45-degree rotation endpoints without
  // multiplying the number of x/y states.
  bool use_pivot_anchor_lattice = false;

  // Rotation axis position measured from the rectangle center along the robot's longitudinal (+x body) axis.
  //
  // Examples:
  //   {0}       -> rotate about rectangle center
  //   {-5,0,5}  -> rear / center / front pivots
  //
  // Unit: grid cells.
  std::vector<int> rotation_pivot_offsets_cells{0};

  bool include_wait = true;
};

struct ObstacleRect {
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

struct MetricObstacleRect {
  double x_m = 0.0;
  double y_m = 0.0;
  double width_m = 0.0;
  double height_m = 0.0;
};

struct Agent {
  State start;
  State goal;
};

struct Problem {
  GridSpec grid;
  RobotSpec robot;
  PrimitiveConfig primitive_config;
  SearchOptions search;
  std::vector<ObstacleRect> obstacles;
  std::vector<MetricObstacleRect> metric_obstacles;
  std::vector<Agent> agents;

  [[nodiscard]] double heading_anchor_x_cells(int heading) const {
    if (!primitive_config.use_pivot_anchor_lattice) return 0.0;
    const double pivot_offset_cells = static_cast<double>(
        primitive_config.rotation_pivot_offsets_cells.front());
    const double yaw = static_cast<double>(positive_mod(
                           heading, grid.heading_bins)) *
                       grid.heading_step();
    const double raw = -pivot_offset_cells * std::cos(yaw);
    return raw - std::floor(raw + 0.5);
  }

  [[nodiscard]] double heading_anchor_y_cells(int heading) const {
    if (!primitive_config.use_pivot_anchor_lattice) return 0.0;
    const double pivot_offset_cells = static_cast<double>(
        primitive_config.rotation_pivot_offsets_cells.front());
    const double yaw = static_cast<double>(positive_mod(
                           heading, grid.heading_bins)) *
                       grid.heading_step();
    const double raw = -pivot_offset_cells * std::sin(yaw);
    return raw - std::floor(raw + 0.5);
  }

  [[nodiscard]] double center_x_cells(const State& state) const {
    return static_cast<double>(state.x) + heading_anchor_x_cells(state.heading);
  }

  [[nodiscard]] double center_y_cells(const State& state) const {
    return static_cast<double>(state.y) + heading_anchor_y_cells(state.heading);
  }

  [[nodiscard]] double world_x(const State& state) const {
    return grid.origin_x + center_x_cells(state) * grid.cell_size;
  }

  [[nodiscard]] double world_y(const State& state) const {
    return grid.origin_y + center_y_cells(state) * grid.cell_size;
  }
};

using PrimitiveId = std::uint16_t;

struct AgentPlan {
  std::vector<State> states;
  std::vector<PrimitiveId> primitive_ids;
};

struct Improvement {
  double elapsed_ms = 0.0;
  double cost = std::numeric_limits<double>::infinity();
  double weight = 1.0;
  std::vector<AgentPlan> plans;
};

struct RuntimeStats {
  // Reusable when map, robot and primitive settings do not change.
  double primitive_collision_precompute_ms = 0.0;
  double transition_cache_precompute_ms = 0.0;
  double static_precompute_ms = 0.0;

  // Depends on the current agents/goals.
  double query_precompute_ms = 0.0;
  double candidate_cache_precompute_ms = 0.0;
  double connection_rule_precompute_ms = 0.0;

  // Starts immediately before graph search.
  double search_ms = 0.0;
  double first_solution_ms = std::numeric_limits<double>::infinity();
  double first_solution_cost = std::numeric_limits<double>::infinity();

  // static_precompute_ms + query_precompute_ms + search_ms.
  double cold_total_ms = 0.0;

  // query_precompute_ms + search_ms. This approximates request latency when
  // static map/robot caches are retained by a persistent planner process.
  double warm_request_ms = 0.0;

  bool transition_cache_enabled = false;
  bool candidate_cache_enabled = false;
  bool ara_star_enabled = false;
  bool reverse_bfs_heuristic_enabled = false;
  bool candidate_diversification_enabled = false;
  bool aabb_broadphase_enabled = false;
  bool conflict_cache_enabled = false;
  bool lazy_successors_enabled = false;
  bool progressive_widening_enabled = false;
  std::size_t initial_candidate_width = 1;
  std::size_t initial_solution_max_branching = 0;
  bool per_primitive_intervals_enabled = false;
  bool multiple_rotation_amounts_enabled = false;
  bool acceleration_constraints_enabled = false;
  bool dynamics_aware_pibt_enabled = false;
  bool interval_dominance_enabled = false;
  bool pivot_anchor_lattice_enabled = false;
  std::size_t active_rotation_amount_count = 1;
  std::string collision_mode = "time_indexed";
  double max_boundary_travel_per_interval_m = 0.005;
  std::size_t collision_interval_count = 0;
  std::size_t collision_interval_polygon_count = 0;

  std::uint64_t transition_lookups = 0;
  std::uint64_t transition_cache_hits = 0;
  std::uint64_t transition_on_demand_computations = 0;

  std::uint64_t candidate_lookups = 0;
  std::uint64_t candidate_cache_hits = 0;
  std::uint64_t candidate_on_demand_computations = 0;

  std::uint64_t expanded_nodes = 0;
  std::uint64_t low_level_constraint_nodes = 0;
  std::uint64_t pibt_plan_calls = 0;
  std::uint64_t pibt_assign_calls = 0;
  std::uint64_t pibt_candidate_attempts = 0;
  std::uint64_t pibt_kinematic_candidate_rejects = 0;
  std::uint64_t pibt_backtracks = 0;
  std::uint64_t kinematic_dominance_rejects = 0;
  double dynamic_prefilter_ms = 0.0;
  std::uint64_t dynamic_prefilter_calls = 0;
  std::uint64_t dynamic_candidate_evaluations = 0;
  std::uint64_t geometry_candidate_count = 0;
  std::uint64_t dynamic_candidate_count = 0;
  std::uint64_t post_pibt_kinematic_rejects = 0;
  std::uint64_t dominance_check_count = 0;
  std::uint64_t cubic_relation_queries = 0;
  std::uint64_t connection_rule_queries = 0;
  std::uint64_t joint_moves_generated = 0;
  std::uint64_t joint_move_duplicates = 0;
  std::uint64_t max_open_size = 0;
  std::uint64_t lazy_successor_requests = 0;
  std::uint64_t successor_continuations = 0;
  std::uint64_t progressive_widening_stages = 0;
  std::uint64_t max_candidate_width = 0;

  std::uint64_t conflict_calls = 0;
  std::uint64_t conflict_cache_hits = 0;
  std::uint64_t conflict_cache_canonical_swaps = 0;
  std::uint64_t conflict_cache_entries = 0;
  std::uint64_t whole_step_aabb_tests = 0;
  std::uint64_t whole_step_aabb_rejects = 0;
  std::uint64_t interval_aabb_tests = 0;
  std::uint64_t interval_aabb_rejects = 0;
  std::uint64_t polygon_sat_tests = 0;

  std::uint64_t kinematic_validation_calls = 0;
  std::uint64_t kinematic_validation_failures = 0;
  std::uint64_t kinematic_search_restarts = 0;
  std::size_t kinematic_no_good_count = 0;
};

struct Solution {
  bool success = false;

  // Kept for compatibility. It is identical to stats.search_ms.
  double elapsed_ms = 0.0;
  double cost = std::numeric_limits<double>::infinity();
  std::string objective;
  std::vector<AgentPlan> plans;
  std::vector<Improvement> improvements;
  RuntimeStats stats;
};

class Deadline {
 public:
  explicit Deadline(double limit_ms)
      : start_(std::chrono::steady_clock::now()), limit_ms_(limit_ms) {}

  [[nodiscard]] double elapsed_ms() const {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start_)
        .count();
  }

  [[nodiscard]] bool expired() const { return elapsed_ms() >= limit_ms_; }

 private:
  std::chrono::steady_clock::time_point start_;
  double limit_ms_;
};

inline int circular_heading_distance(int a, int b, int bins) {
  const int direct = std::abs(positive_mod(a, bins) - positive_mod(b, bins));
  return std::min(direct, bins - direct);
}

}  // namespace lacam_primitive
