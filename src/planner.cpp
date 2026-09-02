#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>

namespace lacam_primitive {
namespace {

constexpr double kEpsilon = 1e-9;

std::string primitive_combo_key(const std::vector<PrimitiveId>& primitives) {
  std::string key;
  key.reserve(primitives.size() * sizeof(PrimitiveId));
  for (PrimitiveId id : primitives) {
    key.append(reinterpret_cast<const char*>(&id), sizeof(id));
  }
  return key;
}

double elapsed_ms_since(
    const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

double twist_norm(const Twist2D& twist) {
  return std::sqrt(
      twist.vx * twist.vx + twist.vy * twist.vy +
      twist.omega * twist.omega);
}

bool boundary_rates_dominate(
    const std::vector<ScalarInterval>& superset,
    const std::vector<ScalarInterval>& subset) {
  if (superset.size() != subset.size()) return false;
  for (std::size_t i = 0; i < superset.size(); ++i) {
    if (superset[i].lower > subset[i].lower + kEpsilon ||
        superset[i].upper + kEpsilon < subset[i].upper) {
      return false;
    }
  }
  return true;
}

}  // namespace

std::size_t Planner::KinematicNoGoodHash::operator()(
    const KinematicNoGood& item) const noexcept {
  std::size_t seed = 0;
  const auto mix = [&seed](std::size_t value) {
    seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
  };
  mix(std::hash<int>{}(item.previous_primitive));
  mix(std::hash<int>{}(item.next_primitive));
  mix(std::hash<int>{}(item.connection_heading));
  mix(std::hash<int>{}(item.terminal_agent));
  return seed;
}

std::size_t Planner::SearchKeyHash::operator()(
    const SearchKey& item) const noexcept {
  std::size_t seed = JointStateHash{}(item.configuration);
  for (PrimitiveId primitive : item.previous_primitives) {
    seed ^= std::hash<PrimitiveId>{}(primitive) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
  }
  for (const ScalarInterval& interval : item.boundary_rates) {
    seed ^= std::hash<double>{}(interval.lower) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<double>{}(interval.upper) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
  }
  return seed;
}

std::size_t Planner::BaseSearchKeyHash::operator()(
    const BaseSearchKey& item) const noexcept {
  std::size_t seed = JointStateHash{}(item.configuration);
  for (PrimitiveId primitive : item.previous_primitives) {
    seed ^= std::hash<PrimitiveId>{}(primitive) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
  }
  return seed;
}

Planner::SearchKey Planner::make_search_key(
    const std::vector<State>& configuration,
    const std::vector<PrimitiveId>& previous_primitives,
    const std::vector<ScalarInterval>& boundary_rates) const {
  return SearchKey{
      configuration,
      problem_.primitive_config.use_acceleration_constraints
          ? previous_primitives
          : std::vector<PrimitiveId>{},
      problem_.primitive_config.use_acceleration_constraints
          ? boundary_rates
          : std::vector<ScalarInterval>{}};
}

std::size_t CollisionChecker::ConflictKeyHash::operator()(
    const ConflictKey& key) const noexcept {
  std::size_t seed = 0;
  const auto mix = [&seed](std::size_t value) {
    seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
  };
  mix(std::hash<int>{}(key.dx));
  mix(std::hash<int>{}(key.dy));
  mix(std::hash<int>{}(key.left_heading));
  mix(std::hash<int>{}(key.right_heading));
  mix(std::hash<PrimitiveId>{}(key.left_primitive));
  mix(std::hash<PrimitiveId>{}(key.right_primitive));
  return seed;
}

CollisionChecker::CollisionChecker(
    const Problem& problem,
    const PrimitiveTable& primitives)
    : problem_(problem),
      primitives_(primitives) {
  obstacle_polygons_.reserve(
      problem.obstacles.size() + problem.metric_obstacles.size());
  for (const ObstacleRect& obstacle : problem.obstacles) {
    // ObstacleRect denotes a union of occupied grid cells. Cell (x, y) is the
    // square [x-0.5, x+0.5] x [y-0.5, y+0.5] in the same coordinate system as
    // State. Adjacent cells collapse into one metric rectangle.
    obstacle_polygons_.push_back(ConvexPolygon{{
        Point2{static_cast<double>(obstacle.x) - 0.5,
               static_cast<double>(obstacle.y) - 0.5},
        Point2{static_cast<double>(obstacle.x + obstacle.width) - 0.5,
               static_cast<double>(obstacle.y) - 0.5},
        Point2{static_cast<double>(obstacle.x + obstacle.width) - 0.5,
               static_cast<double>(obstacle.y + obstacle.height) - 0.5},
        Point2{static_cast<double>(obstacle.x) - 0.5,
               static_cast<double>(obstacle.y + obstacle.height) - 0.5},
    }});
  }
  for (const MetricObstacleRect& obstacle : problem.metric_obstacles) {
    const double min_x = problem.grid.cell_x(obstacle.x_m);
    const double min_y = problem.grid.cell_y(obstacle.y_m);
    const double max_x =
        problem.grid.cell_x(obstacle.x_m + obstacle.width_m);
    const double max_y =
        problem.grid.cell_y(obstacle.y_m + obstacle.height_m);
    obstacle_polygons_.push_back(ConvexPolygon{{
        Point2{min_x, min_y},
        Point2{max_x, min_y},
        Point2{max_x, max_y},
        Point2{min_x, max_y},
    }});
  }
}

bool CollisionChecker::statically_valid(
    const State& start,
    PrimitiveId primitive_id) const {
  const PrimitiveVariant& variant =
      primitives_.variant(primitive_id, start.heading);

  // Metric inputs use an explicit physical boundary converted into lattice
  // coordinates. Legacy inputs retain the historical outer half-cell bounds.
  // Direct polygon tests avoid any extra full-cell dilation.
  const double min_x = problem_.grid.collision_min_x_cells();
  const double min_y = problem_.grid.collision_min_y_cells();
  const double max_x = problem_.grid.collision_max_x_cells();
  const double max_y = problem_.grid.collision_max_y_cells();
  const double start_x = problem_.center_x_cells(start);
  const double start_y = problem_.center_y_cells(start);

  for (const ConvexPolygon& polygon : variant.interval_polygons) {
    if (!shifted_polygon_inside_bounds(
            polygon,
            start_x,
            start_y,
            min_x,
            min_y,
            max_x,
            max_y)) {
      return false;
    }
    for (const ConvexPolygon& obstacle : obstacle_polygons_) {
      if (shifted_polygons_intersect(
              polygon,
              start_x,
              start_y,
              obstacle,
              0.0,
              0.0)) {
        return false;
      }
    }
  }
  return true;
}

bool CollisionChecker::conflict(
    const State& left_start,
    PrimitiveId left_primitive,
    const State& right_start,
    PrimitiveId right_primitive) const {
  ++conflict_calls_;
  ConflictKey key{
      right_start.x - left_start.x,
      right_start.y - left_start.y,
      left_start.heading,
      right_start.heading,
      left_primitive,
      right_primitive};
  if (problem_.search.use_conflict_cache) {
    const ConflictKey swapped{
        -key.dx,
        -key.dy,
        key.right_heading,
        key.left_heading,
        key.right_primitive,
        key.left_primitive};
    const auto as_tuple = [](const ConflictKey& value) {
      return std::make_tuple(
          value.dx, value.dy, value.left_heading, value.right_heading,
          value.left_primitive, value.right_primitive);
    };
    if (as_tuple(swapped) < as_tuple(key)) {
      key = swapped;
      ++conflict_cache_canonical_swaps_;
    }
  }
  if (problem_.search.use_conflict_cache) {
    const auto found = conflict_cache_.find(key);
    if (found != conflict_cache_.end()) {
      ++conflict_cache_hits_;
      return found->second;
    }
  }

  const PrimitiveVariant& left =
      primitives_.variant(left_primitive, left_start.heading);
  const PrimitiveVariant& right =
      primitives_.variant(right_primitive, right_start.heading);
  const double left_x = problem_.center_x_cells(left_start);
  const double left_y = problem_.center_y_cells(left_start);
  const double right_x = problem_.center_x_cells(right_start);
  const double right_y = problem_.center_y_cells(right_start);
  const auto finish = [&](bool result) {
    if (problem_.search.use_conflict_cache) {
      conflict_cache_.emplace(key, result);
    }
    return result;
  };

  if (problem_.search.use_aabb_broadphase) {
    ++whole_step_aabb_tests_;
    if (!shifted_bounds_intersect(
            left.whole_step_bounds,
            left_x,
            left_y,
            right.whole_step_bounds,
            right_x,
            right_y)) {
      ++whole_step_aabb_rejects_;
      return finish(false);
    }
  }
  if (problem_.search.collision_mode == "whole_step") {
    ++polygon_sat_tests_;
    return finish(shifted_polygons_intersect(
        left.whole_step_polygon,
        left_x,
        left_y,
        right.whole_step_polygon,
        right_x,
        right_y));
  }
  if (left.interval_bounds.size() != left.interval_polygons.size() ||
      right.interval_bounds.size() != right.interval_polygons.size()) {
    throw std::logic_error("primitive interval AABB counts differ");
  }
  if (left.interval_polygons.empty() || right.interval_polygons.empty()) {
    throw std::logic_error("primitive has no collision intervals");
  }

  // Each primitive may use a different uniform time partition. Walk the
  // overlapping normalized intervals so comparisons remain synchronized.
  std::size_t left_interval = 0;
  std::size_t right_interval = 0;
  while (left_interval < left.interval_polygons.size() &&
         right_interval < right.interval_polygons.size()) {
    if (problem_.search.use_aabb_broadphase) {
      ++interval_aabb_tests_;
      if (!shifted_bounds_intersect(
              left.interval_bounds[left_interval],
              left_x,
              left_y,
              right.interval_bounds[right_interval],
              right_x,
              right_y)) {
        ++interval_aabb_rejects_;
      } else {
        ++polygon_sat_tests_;
        if (shifted_polygons_intersect(
                left.interval_polygons[left_interval],
                left_x,
                left_y,
                right.interval_polygons[right_interval],
                right_x,
                right_y)) {
          return finish(true);
        }
      }
    } else {
      ++polygon_sat_tests_;
      if (shifted_polygons_intersect(
              left.interval_polygons[left_interval],
              left_x,
              left_y,
              right.interval_polygons[right_interval],
              right_x,
              right_y)) {
        return finish(true);
      }
    }

    const std::uint64_t left_end_scaled =
        static_cast<std::uint64_t>(left_interval + 1) *
        static_cast<std::uint64_t>(right.interval_polygons.size());
    const std::uint64_t right_end_scaled =
        static_cast<std::uint64_t>(right_interval + 1) *
        static_cast<std::uint64_t>(left.interval_polygons.size());
    if (left_end_scaled <= right_end_scaled) ++left_interval;
    if (right_end_scaled <= left_end_scaled) ++right_interval;
  }
  return finish(false);
}

void CollisionChecker::reset_runtime_stats() const {
  conflict_cache_.clear();
  conflict_calls_ = 0;
  conflict_cache_hits_ = 0;
  conflict_cache_canonical_swaps_ = 0;
  whole_step_aabb_tests_ = 0;
  whole_step_aabb_rejects_ = 0;
  interval_aabb_tests_ = 0;
  interval_aabb_rejects_ = 0;
  polygon_sat_tests_ = 0;
}

TransitionProvider::TransitionProvider(
    const Problem& problem,
    const PrimitiveTable& primitives,
    const CollisionChecker& collision_checker,
    bool cache_enabled)
    : problem_(problem),
      primitives_(primitives),
      collision_checker_(collision_checker),
      cache_enabled_(cache_enabled),
      state_count_(
          static_cast<std::size_t>(problem.grid.width_cells) *
          static_cast<std::size_t>(problem.grid.height_cells) *
          static_cast<std::size_t>(problem.grid.heading_bins)),
      primitive_count_(primitives.primitives().size()) {}

std::size_t TransitionProvider::state_index(const State& state) const {
  if (state.x < 0 || state.y < 0 ||
      state.x >= problem_.grid.width_cells ||
      state.y >= problem_.grid.height_cells ||
      state.heading < 0 || state.heading >= problem_.grid.heading_bins) {
    throw std::out_of_range("state is outside the transition table");
  }
  return static_cast<std::size_t>(
      (state.heading * problem_.grid.height_cells + state.y) *
          problem_.grid.width_cells +
      state.x);
}

std::size_t TransitionProvider::cache_index(
    const State& state,
    PrimitiveId primitive_id) const {
  const std::size_t primitive = static_cast<std::size_t>(primitive_id);
  if (primitive >= primitive_count_) {
    throw std::out_of_range("primitive id is outside the transition table");
  }
  return state_index(state) * primitive_count_ + primitive;
}

TransitionEntry TransitionProvider::compute(
    const State& state,
    PrimitiveId primitive_id) const {
  return TransitionEntry{
      primitives_.apply(state, primitive_id),
      collision_checker_.statically_valid(state, primitive_id)};
}

void TransitionProvider::build_cache() {
  if (!cache_enabled_) {
    precompute_ms_ = 0.0;
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  cache_.resize(state_count_ * primitive_count_);

  for (int heading = 0; heading < problem_.grid.heading_bins; ++heading) {
    for (int y = 0; y < problem_.grid.height_cells; ++y) {
      for (int x = 0; x < problem_.grid.width_cells; ++x) {
        const State state{x, y, heading};
        for (const Primitive& primitive : primitives_.primitives()) {
          cache_[cache_index(state, primitive.id)] = compute(state, primitive.id);
        }
      }
    }
  }

  precompute_ms_ = elapsed_ms_since(start);
  reset_runtime_stats();
}

TransitionEntry TransitionProvider::lookup(
    const State& state,
    PrimitiveId primitive_id) const {
  ++lookups_;
  if (cache_enabled_) {
    ++cache_hits_;
    return cache_.at(cache_index(state, primitive_id));
  }
  ++on_demand_computations_;
  return compute(state, primitive_id);
}

void TransitionProvider::reset_runtime_stats() const {
  lookups_ = 0;
  cache_hits_ = 0;
  on_demand_computations_ = 0;
}

CandidateProvider::CandidateProvider(
    const Problem& problem,
    const PrimitiveTable& primitives,
    const TransitionProvider& transitions,
    bool cache_enabled)
    : problem_(problem),
      primitives_(primitives),
      transitions_(transitions),
      cache_enabled_(cache_enabled),
      state_count_(transitions.state_count()) {}

std::size_t CandidateProvider::state_index(const State& state) const {
  if (state.x < 0 || state.y < 0 ||
      state.x >= problem_.grid.width_cells ||
      state.y >= problem_.grid.height_cells ||
      state.heading < 0 || state.heading >= problem_.grid.heading_bins) {
    throw std::out_of_range("state is outside the candidate table");
  }
  return static_cast<std::size_t>(
      (state.heading * problem_.grid.height_cells + state.y) *
          problem_.grid.width_cells +
      state.x);
}

std::size_t CandidateProvider::cache_index(
    std::size_t agent,
    const State& state) const {
  if (agent >= problem_.agents.size()) {
    throw std::out_of_range("agent is outside the candidate table");
  }
  return agent * state_count_ + state_index(state);
}

double CandidateProvider::agent_heuristic(
    std::size_t agent,
    const State& state) const {
  const State& goal =
      problem_.agents.at(agent).goal;

  const int manhattan =
      std::abs(state.x - goal.x) +
      std::abs(state.y - goal.y);

  const int max_position_delta =
      std::max(
          1,
          primitives_
              .max_position_delta_cells());

  const int position_steps =
      (manhattan +
       max_position_delta - 1) /
      max_position_delta;

  const int heading_distance =
      circular_heading_distance(
          state.heading,
          goal.heading,
          problem_.grid.heading_bins);

  const int rotation_steps =
      (heading_distance +
       primitives_.max_rotation_bins() - 1) /
      primitives_.max_rotation_bins();

  if (primitives_
          .has_coupled_rotation_translation()) {
    // One pivot primitive may make progress in both position and heading simultaneously.
    return static_cast<double>(
        std::max(
            position_steps,
            rotation_steps));
  }

  // Preserve the stronger old heuristic when all rotation primitives rotate in place.
  return static_cast<double>(
      position_steps + rotation_steps);
}

State CandidateProvider::state_from_index(std::size_t index) const {
  const int x = static_cast<int>(index % problem_.grid.width_cells);
  index /= static_cast<std::size_t>(problem_.grid.width_cells);
  const int y = static_cast<int>(index % problem_.grid.height_cells);
  index /= static_cast<std::size_t>(problem_.grid.height_cells);
  return State{x, y, static_cast<int>(index)};
}

void CandidateProvider::build_reverse_distances() {
  reverse_edges_.assign(state_count_, {});
  for (std::size_t index = 0; index < state_count_; ++index) {
    const State state = state_from_index(index);
    for (const Primitive& primitive : primitives_.primitives()) {
      const TransitionEntry transition = transitions_.lookup(state, primitive.id);
      if (!transition.valid) continue;
      reverse_edges_[state_index(transition.next)].push_back(
          static_cast<int>(index));
    }
  }

  const int unreachable = std::numeric_limits<int>::max();
  distance_to_goal_.assign(
      problem_.agents.size(), std::vector<int>(state_count_, unreachable));
  for (std::size_t agent = 0; agent < problem_.agents.size(); ++agent) {
    auto& distance = distance_to_goal_[agent];
    std::queue<std::size_t> open;
    const std::size_t goal = state_index(problem_.agents[agent].goal);
    distance[goal] = 0;
    open.push(goal);
    while (!open.empty()) {
      const std::size_t next = open.front();
      open.pop();
      for (int predecessor : reverse_edges_[next]) {
        const std::size_t previous = static_cast<std::size_t>(predecessor);
        if (distance[previous] != unreachable) continue;
        distance[previous] = distance[next] + 1;
        open.push(previous);
      }
    }
  }
}

double CandidateProvider::agent_distance(
    std::size_t agent,
    const State& state) const {
  if (problem_.search.use_reverse_bfs_heuristic &&
      !distance_to_goal_.empty()) {
    const int distance = distance_to_goal_.at(agent).at(state_index(state));
    if (distance == std::numeric_limits<int>::max()) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(distance);
  }
  return agent_heuristic(agent, state);
}

std::vector<PrimitiveId> CandidateProvider::compute(
    std::size_t agent,
    const State& state) const {
  struct Ranked {
    PrimitiveId id;
    double score;
    int wait_penalty;
  };

  std::vector<Ranked> ranked;
  ranked.reserve(primitives_.primitives().size());
  const bool at_goal = state == problem_.agents.at(agent).goal;

  for (const Primitive& primitive : primitives_.primitives()) {
    if (problem_.primitive_config.use_acceleration_constraints &&
        !primitive.kinematically_feasible) {
      continue;
    }
    const TransitionEntry transition = transitions_.lookup(state, primitive.id);
    if (!transition.valid) continue;
    const int wait_penalty =
        primitive.id == primitives_.wait_id() ? (at_goal ? 0 : 1) : 0;
    ranked.push_back(Ranked{
        primitive.id,
        agent_distance(agent, transition.next),
        wait_penalty});
  }

  std::stable_sort(
      ranked.begin(), ranked.end(),
      [](const Ranked& left, const Ranked& right) {
        if (left.score != right.score) return left.score < right.score;
        if (left.wait_penalty != right.wait_penalty) {
          return left.wait_penalty < right.wait_penalty;
        }
        return left.id < right.id;
      });

  std::vector<PrimitiveId> result;
  result.reserve(ranked.size());
  for (const Ranked& item : ranked) result.push_back(item.id);
  return result;
}

void CandidateProvider::build_cache() {
  const auto start = std::chrono::steady_clock::now();
  if (problem_.search.use_reverse_bfs_heuristic) {
    build_reverse_distances();
  }
  if (cache_enabled_) {
    cache_.resize(problem_.agents.size() * state_count_);

    for (std::size_t agent = 0; agent < problem_.agents.size(); ++agent) {
      for (int heading = 0; heading < problem_.grid.heading_bins; ++heading) {
        for (int y = 0; y < problem_.grid.height_cells; ++y) {
          for (int x = 0; x < problem_.grid.width_cells; ++x) {
            const State state{x, y, heading};
            cache_[cache_index(agent, state)] = compute(agent, state);
          }
        }
      }
    }
  }

  precompute_ms_ = elapsed_ms_since(start);
  reset_runtime_stats();
}

const std::vector<PrimitiveId>& CandidateProvider::ordered_candidates(
    std::size_t agent,
    const State& state,
    std::vector<PrimitiveId>& scratch) const {
  ++lookups_;
  if (cache_enabled_) {
    ++cache_hits_;
    return cache_.at(cache_index(agent, state));
  }

  ++on_demand_computations_;
  scratch = compute(agent, state);
  return scratch;
}

void CandidateProvider::reset_runtime_stats() const {
  lookups_ = 0;
  cache_hits_ = 0;
  on_demand_computations_ = 0;
}

PIBT::PIBT(
    const Problem& problem,
    const PrimitiveTable& primitives,
    const CollisionChecker& collision_checker,
    const TransitionProvider& transitions,
    const CandidateProvider& candidates,
    std::mt19937& random,
    SearchInstrumentation& instrumentation)
    : problem_(problem),
      primitives_(primitives),
      collision_checker_(collision_checker),
      transitions_(transitions),
      candidates_(candidates),
      random_(random),
      instrumentation_(instrumentation) {}

const std::vector<PrimitiveId>& PIBT::ordered_candidates(
    std::size_t agent,
    const State& state,
    std::vector<PrimitiveId>& scratch) const {
  return candidates_.ordered_candidates(agent, state, scratch);
}

bool PIBT::plan(
    const std::vector<State>& current,
    const std::vector<int>& priority_order,
    const std::vector<std::optional<PrimitiveId>>& forced,
    const std::vector<std::vector<DynamicCandidateInfo>>& dynamic_info,
    const std::vector<std::vector<PrimitiveId>>& dynamic_candidates,
    std::vector<PrimitiveId>& selected,
    std::vector<State>& next) {
  ++instrumentation_.pibt_plan_calls;
  const std::size_t count = current.size();
  selected.assign(count, primitives_.wait_id());
  next = current;
  std::vector<bool> assigned(count, false);
  std::vector<bool> visiting(count, false);

  for (int agent : priority_order) {
    if (!assigned[static_cast<std::size_t>(agent)] &&
        !assign_agent(
            agent, current, forced, dynamic_info, dynamic_candidates,
            selected, next, assigned, visiting)) {
      return false;
    }
  }
  return true;
}

bool PIBT::assign_agent(
    int agent,
    const std::vector<State>& current,
    const std::vector<std::optional<PrimitiveId>>& forced,
    const std::vector<std::vector<DynamicCandidateInfo>>& dynamic_info,
    const std::vector<std::vector<PrimitiveId>>& dynamic_candidates,
    std::vector<PrimitiveId>& selected,
    std::vector<State>& next,
    std::vector<bool>& assigned,
    std::vector<bool>& visiting) {
  ++instrumentation_.pibt_assign_calls;
  const std::size_t id = static_cast<std::size_t>(agent);
  if (assigned[id]) return true;
  if (visiting[id]) return false;
  visiting[id] = true;

  std::vector<PrimitiveId> scratch;
  std::vector<PrimitiveId> forced_candidate;
  const std::vector<PrimitiveId>* candidates = nullptr;
  if (forced[id].has_value()) {
    forced_candidate.push_back(*forced[id]);
    candidates = &forced_candidate;
  } else if (!dynamic_info.empty()) {
    candidates = &dynamic_candidates.at(id);
  } else {
    candidates = &ordered_candidates(id, current[id], scratch);
  }

  for (PrimitiveId candidate : *candidates) {
    ++instrumentation_.pibt_candidate_attempts;
    // Forced constraints intentionally bypass the filtered list, so retain
    // this guard for correctness. Unforced candidates are already filtered.
    if (!dynamic_info.empty() &&
        !dynamic_info.at(id).at(candidate).feasible) {
      ++instrumentation_.pibt_kinematic_candidate_rejects;
      continue;
    }
    const TransitionEntry transition = transitions_.lookup(current[id], candidate);
    if (!transition.valid) continue;

    const auto selected_snapshot = selected;
    const auto next_snapshot = next;
    const auto assigned_snapshot = assigned;
    const auto visiting_snapshot = visiting;

    selected[id] = candidate;
    next[id] = transition.next;
    assigned[id] = true;

    bool valid = true;
    for (std::size_t other = 0; other < current.size() && valid; ++other) {
      if (other == id || !assigned[other]) continue;
      if (collision_checker_.conflict(
              current[id], candidate, current[other], selected[other])) {
        valid = false;
      }
    }

    for (std::size_t other = 0; other < current.size() && valid; ++other) {
      if (other == id || assigned[other]) continue;
      if (!collision_checker_.conflict(
              current[id], candidate,
              current[other], primitives_.wait_id())) {
        continue;
      }
      if (!assign_agent(
              static_cast<int>(other), current, forced,
              dynamic_info, dynamic_candidates,
              selected, next, assigned, visiting)) {
        valid = false;
        break;
      }
    }

    if (valid) {
      for (std::size_t other = 0; other < current.size(); ++other) {
        if (other == id || !assigned[other]) continue;
        if (collision_checker_.conflict(
                current[id], candidate, current[other], selected[other])) {
          valid = false;
          break;
        }
      }
    }

    if (valid) {
      visiting[id] = false;
      return true;
    }

    selected = selected_snapshot;
    next = next_snapshot;
    assigned = assigned_snapshot;
    visiting = visiting_snapshot;
    ++instrumentation_.pibt_backtracks;
  }

  visiting[id] = false;
  return false;
}

std::size_t Planner::connection_rule_index(
    int connection_heading,
    PrimitiveId previous,
    PrimitiveId next) const {
  const std::size_t primitive_count = primitives_.primitives().size();
  return (static_cast<std::size_t>(positive_mod(
              connection_heading, problem_.grid.heading_bins)) *
              primitive_count +
          static_cast<std::size_t>(previous)) *
             primitive_count +
         static_cast<std::size_t>(next);
}

void Planner::build_connection_rules() {
  const std::size_t primitive_count = primitives_.primitives().size();
  connection_rules_.assign(
      static_cast<std::size_t>(problem_.grid.heading_bins) *
          primitive_count * primitive_count,
      ConnectionRule{});
  constexpr double basis_tolerance = 1e-10;

  for (int heading = 0; heading < problem_.grid.heading_bins; ++heading) {
    for (const Primitive& previous : primitives_.primitives()) {
      const int previous_start_heading = positive_mod(
          heading - previous.d_heading, problem_.grid.heading_bins);
      const Twist2D& outgoing_basis =
          primitives_.variant(previous.id, previous_start_heading)
              .end_twist_per_progress_rate;
      const double outgoing_norm = twist_norm(outgoing_basis);

      for (const Primitive& next : primitives_.primitives()) {
        const Twist2D& incoming_basis =
            primitives_.variant(next.id, heading)
                .start_twist_per_progress_rate;
        const double incoming_norm = twist_norm(incoming_basis);
        ConnectionRule rule;

        if (outgoing_norm <= basis_tolerance &&
            incoming_norm <= basis_tolerance) {
          rule.kind = ConnectionRuleKind::kUnconstrained;
        } else if (outgoing_norm <= basis_tolerance) {
          rule.kind = ConnectionRuleKind::kZero;
        } else if (incoming_norm <= basis_tolerance) {
          rule.kind = ConnectionRuleKind::kRequiresZeroUnconstrained;
        } else {
          const double incoming_components[3] = {
              incoming_basis.vx, incoming_basis.vy, incoming_basis.omega};
          const double outgoing_components[3] = {
              outgoing_basis.vx, outgoing_basis.vy, outgoing_basis.omega};
          int pivot = 0;
          for (int component = 1; component < 3; ++component) {
            if (std::abs(incoming_components[component]) >
                std::abs(incoming_components[pivot])) {
              pivot = component;
            }
          }
          rule.scale =
              outgoing_components[pivot] / incoming_components[pivot];
          double residual_squared = 0.0;
          for (int component = 0; component < 3; ++component) {
            const double residual = outgoing_components[component] -
                                    rule.scale * incoming_components[component];
            residual_squared += residual * residual;
          }
          const double tolerance = 1e-9 * std::max(
              {1.0, outgoing_norm,
               std::abs(rule.scale) * incoming_norm});
          rule.kind = std::sqrt(residual_squared) <= tolerance
                          ? ConnectionRuleKind::kScaled
                          : ConnectionRuleKind::kRequiresZero;
        }
        connection_rules_[connection_rule_index(
            heading, previous.id, next.id)] = rule;
      }
    }
  }
}

std::size_t Planner::kinodynamic_lookahead_index(
    int start_heading,
    PrimitiveId first,
    PrimitiveId second,
    PrimitiveId third) const {
  const std::size_t primitive_count = primitives_.primitives().size();
  std::size_t index =
      (static_cast<std::size_t>(positive_mod(
           start_heading, problem_.grid.heading_bins)) * primitive_count +
       static_cast<std::size_t>(first)) * primitive_count +
      static_cast<std::size_t>(second);
  if (problem_.search.kinodynamic_lookahead_depth == 3) {
    index = index * primitive_count + static_cast<std::size_t>(third);
  }
  return index;
}

void Planner::build_kinodynamic_lookahead_table() {
  kinodynamic_lookahead_table_.clear();
  kinodynamic_lookahead_entry_count_ = 0;
  if (!problem_.search.use_kinodynamic_lookahead) return;

  const std::size_t primitive_count = primitives_.primitives().size();
  std::size_t table_size =
      static_cast<std::size_t>(problem_.grid.heading_bins) *
      primitive_count * primitive_count;
  if (problem_.search.kinodynamic_lookahead_depth == 3) {
    table_size *= primitive_count;
  }
  kinodynamic_lookahead_table_.assign(table_size, std::uint8_t{0});

  for (int heading = 0; heading < problem_.grid.heading_bins; ++heading) {
    for (const Primitive& first : primitives_.primitives()) {
      const ScalarInterval broad_first_incoming =
          first.progress_coordinate == ProgressCoordinate::kStationary
              ? ScalarInterval{0.0, 0.0}
              : ScalarInterval{0.0, first.kinematic_max_rate};
      const ScalarInterval first_outgoing =
          propagate_primitive_rates(first, broad_first_incoming);
      if (first_outgoing.empty()) continue;

      const int second_heading = positive_mod(
          heading + first.d_heading, problem_.grid.heading_bins);
      for (const Primitive& second : primitives_.primitives()) {
        const ScalarInterval second_incoming = connect_boundary_rates(
            second_heading, first.id, second.id, first_outgoing);
        const ScalarInterval second_outgoing =
            propagate_primitive_rates(second, second_incoming);
        if (second_outgoing.empty()) continue;

        if (problem_.search.kinodynamic_lookahead_depth == 2) {
          kinodynamic_lookahead_table_[kinodynamic_lookahead_index(
              heading, first.id, second.id)] = 1;
          ++kinodynamic_lookahead_entry_count_;
          continue;
        }

        const int third_heading = positive_mod(
            second_heading + second.d_heading,
            problem_.grid.heading_bins);
        for (const Primitive& third : primitives_.primitives()) {
          const ScalarInterval third_incoming = connect_boundary_rates(
              third_heading, second.id, third.id, second_outgoing);
          if (propagate_primitive_rates(third, third_incoming).empty()) {
            continue;
          }
          kinodynamic_lookahead_table_[kinodynamic_lookahead_index(
              heading, first.id, second.id, third.id)] = 1;
          ++kinodynamic_lookahead_entry_count_;
        }
      }
    }
  }
}

double Planner::kinodynamic_lookahead_score(
    std::size_t agent,
    const State& start,
    PrimitiveId first,
    const ScalarInterval& first_outgoing) const {
  ++instrumentation_.kinodynamic_lookahead_score_queries;
  const TransitionEntry first_transition = transitions_.lookup(start, first);
  if (!first_transition.valid) {
    return std::numeric_limits<double>::infinity();
  }

  double best = std::numeric_limits<double>::infinity();
  std::vector<PrimitiveId> second_scratch;
  const std::vector<PrimitiveId>& second_candidates =
      candidates_.ordered_candidates(
          agent, first_transition.next, second_scratch);
  const std::size_t future_width = std::min(
      problem_.search.alternatives_per_agent, second_candidates.size());
  for (std::size_t second_index = 0;
       second_index < future_width; ++second_index) {
    const PrimitiveId second = second_candidates[second_index];
    ++instrumentation_.kinodynamic_lookahead_sequences_evaluated;
    if (problem_.search.kinodynamic_lookahead_depth == 2 &&
        kinodynamic_lookahead_table_[kinodynamic_lookahead_index(
            start.heading, first, second)] == 0) continue;
    const TransitionEntry second_transition = transitions_.lookup(
        first_transition.next, second);
    if (!second_transition.valid) continue;
    const ScalarInterval second_incoming = connect_boundary_rates(
        first_transition.next.heading, first, second, first_outgoing);
    const ScalarInterval second_outgoing = propagate_primitive_rates(
        primitives_.primitive(second), second_incoming);
    if (!second_transition.valid || second_outgoing.empty()) continue;

    State endpoint = second_transition.next;
    ScalarInterval endpoint_rates = second_outgoing;
    if (problem_.search.kinodynamic_lookahead_depth == 3) {
      std::vector<PrimitiveId> third_scratch;
      const std::vector<PrimitiveId>& third_candidates =
          candidates_.ordered_candidates(
              agent, second_transition.next, third_scratch);
      const std::size_t third_width = std::min(
          problem_.search.alternatives_per_agent,
          third_candidates.size());
      for (std::size_t third_index = 0;
           third_index < third_width; ++third_index) {
        const PrimitiveId third = third_candidates[third_index];
        ++instrumentation_.kinodynamic_lookahead_sequences_evaluated;
        if (kinodynamic_lookahead_table_[kinodynamic_lookahead_index(
                start.heading, first, second, third)] == 0) continue;
        const TransitionEntry third_transition = transitions_.lookup(
            second_transition.next, third);
        if (!third_transition.valid) continue;
        const ScalarInterval third_incoming = connect_boundary_rates(
            second_transition.next.heading, second, third, second_outgoing);
        const ScalarInterval third_outgoing = propagate_primitive_rates(
            primitives_.primitive(third), third_incoming);
        if (third_outgoing.empty()) continue;
        ++instrumentation_.kinodynamic_lookahead_feasible_sequences;
        double score = candidates_.agent_distance(
            agent, third_transition.next);
        if (third_transition.next == problem_.agents[agent].goal &&
            !third_outgoing.contains(0.0)) {
          score += 1.0;
        }
        best = std::min(best, score);
      }
      continue;
    }

    ++instrumentation_.kinodynamic_lookahead_feasible_sequences;
    double score = candidates_.agent_distance(agent, endpoint);
    // A goal pose is not terminal until the boundary rate can contain zero.
    // Keep such a continuation usable, but rank it behind a stoppable one.
    if (endpoint == problem_.agents[agent].goal &&
        !endpoint_rates.contains(0.0)) {
      score += 1.0;
    }
    best = std::min(best, score);
  }
  return best;
}

ScalarInterval Planner::connect_boundary_rates(
    int connection_heading,
    PrimitiveId previous,
    PrimitiveId next,
    const ScalarInterval& outgoing) const {
  ++instrumentation_.connection_rule_queries;
  if (outgoing.empty()) return ScalarInterval{};
  const ConnectionRule& rule = connection_rules_[
      connection_rule_index(connection_heading, previous, next)];
  const ScalarInterval unconstrained{
      -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
  switch (rule.kind) {
    case ConnectionRuleKind::kUnconstrained:
      return unconstrained;
    case ConnectionRuleKind::kZero:
      return ScalarInterval{0.0, 0.0};
    case ConnectionRuleKind::kRequiresZeroUnconstrained:
      return outgoing.contains(0.0) ? unconstrained : ScalarInterval{};
    case ConnectionRuleKind::kRequiresZero:
      return outgoing.contains(0.0)
                 ? ScalarInterval{0.0, 0.0}
                 : ScalarInterval{};
    case ConnectionRuleKind::kScaled: {
      const double first = rule.scale * outgoing.lower;
      const double second = rule.scale * outgoing.upper;
      return ScalarInterval{
          std::min(first, second), std::max(first, second)};
    }
  }
  return ScalarInterval{};
}

Planner::Planner(Problem problem)
    : construction_start_(std::chrono::steady_clock::now()),
      problem_(std::move(problem)),
      primitives_(problem_),
      collision_checker_(problem_, primitives_),
      transitions_(
          problem_, primitives_, collision_checker_,
          problem_.search.use_transition_cache),
      candidates_(
          problem_, primitives_, transitions_,
          problem_.search.use_candidate_cache),
      random_(problem_.search.random_seed) {
  primitive_collision_precompute_ms_ = elapsed_ms_since(construction_start_);

  if (problem_.agents.empty()) {
    throw std::invalid_argument("at least one agent is required");
  }
  if (problem_.search.objective != "sum_of_costs" &&
      problem_.search.objective != "makespan") {
    throw std::invalid_argument("objective must be sum_of_costs or makespan");
  }
  if (problem_.search.kinodynamic_lookahead_depth != 2 &&
      problem_.search.kinodynamic_lookahead_depth != 3) {
    throw std::invalid_argument(
        "kinodynamic lookahead depth must be 2 or 3");
  }
  if (problem_.search.use_kinodynamic_lookahead &&
      (!problem_.primitive_config.use_acceleration_constraints ||
       !problem_.search.use_dynamics_aware_pibt)) {
    throw std::invalid_argument(
        "kinodynamic lookahead requires acceleration constraints and "
        "dynamics-aware PIBT");
  }

  const auto connection_start = std::chrono::steady_clock::now();
  build_connection_rules();
  connection_rule_precompute_ms_ = elapsed_ms_since(connection_start);

  const auto lookahead_start = std::chrono::steady_clock::now();
  build_kinodynamic_lookahead_table();
  kinodynamic_lookahead_precompute_ms_ =
      elapsed_ms_since(lookahead_start);

  transitions_.build_cache();
  static_precompute_ms_ = elapsed_ms_since(construction_start_);

  const auto query_start = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < problem_.agents.size(); ++i) {
    const Agent& agent = problem_.agents[i];
    if (!transitions_.lookup(agent.start, primitives_.wait_id()).valid) {
      throw std::invalid_argument(
          "agent " + std::to_string(i) + " start footprint is invalid");
    }
    if (!transitions_.lookup(agent.goal, primitives_.wait_id()).valid) {
      throw std::invalid_argument(
          "agent " + std::to_string(i) + " goal footprint is invalid");
    }
    for (std::size_t j = i + 1; j < problem_.agents.size(); ++j) {
      if (collision_checker_.conflict(
              agent.start, primitives_.wait_id(),
              problem_.agents[j].start, primitives_.wait_id())) {
        throw std::invalid_argument("start footprints overlap");
      }
      if (collision_checker_.conflict(
              agent.goal, primitives_.wait_id(),
              problem_.agents[j].goal, primitives_.wait_id())) {
        throw std::invalid_argument("goal footprints overlap");
      }
    }
  }

  candidates_.build_cache();
  query_precompute_ms_ = elapsed_ms_since(query_start);

  // Runtime counters should describe only the graph search, not cache building.
  transitions_.reset_runtime_stats();
  candidates_.reset_runtime_stats();
  instrumentation_ = SearchInstrumentation{};
}

double Planner::single_agent_steps(
    std::size_t agent,
    const State& state) const {
  return candidates_.agent_distance(agent, state);
}

double Planner::heuristic(
    const std::vector<State>& configuration) const {
  if (problem_.search.objective == "makespan") {
    double result = 0.0;
    for (std::size_t i = 0; i < configuration.size(); ++i) {
      result = std::max(result, single_agent_steps(i, configuration[i]));
    }
    return result * problem_.grid.macro_dt;
  }

  double result = 0.0;
  for (std::size_t i = 0; i < configuration.size(); ++i) {
    result += single_agent_steps(i, configuration[i]);
  }
  return result * problem_.grid.macro_dt;
}

double Planner::edge_cost(
    const std::vector<State>& configuration) const {
  if (problem_.search.objective == "makespan") {
    return problem_.grid.macro_dt;
  }

  std::size_t unfinished = 0;
  for (std::size_t i = 0; i < configuration.size(); ++i) {
    if (configuration[i] != problem_.agents[i].goal) ++unfinished;
  }
  return static_cast<double>(unfinished) * problem_.grid.macro_dt;
}

bool Planner::is_goal(
    const std::vector<State>& configuration) const {
  for (std::size_t i = 0; i < configuration.size(); ++i) {
    if (configuration[i] != problem_.agents[i].goal) return false;
  }
  return true;
}

std::vector<int> Planner::priority_order(
    const std::vector<State>& configuration) const {
  std::vector<int> order(configuration.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int left, int right) {
    const double left_h = single_agent_steps(
        static_cast<std::size_t>(left),
        configuration[static_cast<std::size_t>(left)]);
    const double right_h = single_agent_steps(
        static_cast<std::size_t>(right),
        configuration[static_cast<std::size_t>(right)]);
    if (left_h != right_h) return left_h > right_h;
    return left < right;
  });
  return order;
}

std::optional<Planner::JointMove> Planner::next_joint_move(
    const std::vector<State>& configuration,
    const std::vector<PrimitiveId>& previous_primitives,
    const std::vector<ScalarInterval>& current_boundary_rates,
    const std::vector<int>& order,
    PIBT& pibt,
    Deadline& deadline,
    JointMoveGeneratorState& generator) {
  ++instrumentation_.lazy_successor_requests;
  const std::size_t maximum_width =
      std::max<std::size_t>(1, problem_.search.alternatives_per_agent);

  const auto reset_frontier = [&]() {
    generator.open = std::queue<LowLevelConstraintNode>();
    generator.seen_constraint_nodes.clear();
    generator.open.push(LowLevelConstraintNode{
        0,
        std::vector<std::optional<PrimitiveId>>(configuration.size())});
  };

  if (!generator.initialized) {
    generator.initialized = true;
    generator.candidate_width = problem_.search.use_progressive_widening
        ? std::min(maximum_width,
                   std::max<std::size_t>(
                       1, problem_.search.initial_candidate_width))
        : maximum_width;
    instrumentation_.max_candidate_width = std::max<std::uint64_t>(
        instrumentation_.max_candidate_width, generator.candidate_width);

    if (problem_.primitive_config.use_acceleration_constraints &&
        problem_.search.use_dynamics_aware_pibt) {
      const auto prefilter_start = std::chrono::steady_clock::now();
      ++instrumentation_.dynamic_prefilter_calls;
      generator.dynamic_info.assign(
          configuration.size(),
          std::vector<DynamicCandidateInfo>(
              primitives_.primitives().size()));
      generator.dynamic_candidates.resize(configuration.size());
      for (std::size_t agent = 0; agent < configuration.size(); ++agent) {
        std::vector<PrimitiveId> scratch;
        const std::vector<PrimitiveId>& geometry_candidates =
            pibt.ordered_candidates(agent, configuration[agent], scratch);
        instrumentation_.geometry_candidate_count +=
            geometry_candidates.size();
        auto& filtered = generator.dynamic_candidates[agent];
        filtered.reserve(geometry_candidates.size());
        for (PrimitiveId candidate_id : geometry_candidates) {
          ++instrumentation_.dynamic_candidate_evaluations;
          const Primitive& candidate =
              primitives_.primitive(candidate_id);
          ScalarInterval incoming{0.0, 0.0};
          if (!previous_primitives.empty()) {
            const PrimitiveId previous = previous_primitives[agent];
            incoming = connect_boundary_rates(
                configuration[agent].heading,
                previous,
                candidate_id,
                current_boundary_rates[agent]);
          }
          DynamicCandidateInfo& info =
              generator.dynamic_info[agent][candidate_id];
          info.outgoing_rates = propagate_primitive_rates(candidate, incoming);
          info.feasible = !info.outgoing_rates.empty();
          if (info.feasible) filtered.push_back(candidate_id);
        }
        if (problem_.search.use_kinodynamic_lookahead &&
            filtered.size() > 1) {
          std::vector<double> scores(
              primitives_.primitives().size(),
              std::numeric_limits<double>::infinity());
          for (PrimitiveId candidate_id : filtered) {
            scores[candidate_id] = kinodynamic_lookahead_score(
                agent, configuration[agent], candidate_id,
                generator.dynamic_info[agent][candidate_id].outgoing_rates);
          }
          std::stable_sort(
              filtered.begin(), filtered.end(),
              [&](PrimitiveId left, PrimitiveId right) {
                return scores[left] < scores[right];
              });
        }
        instrumentation_.dynamic_candidate_count += filtered.size();
      }
      instrumentation_.dynamic_prefilter_ms +=
          elapsed_ms_since(prefilter_start);
    }
    reset_frontier();
  }

  while (!deadline.expired()) {
    if (generator.yielded >= problem_.search.max_branching) {
      generator.exhausted = true;
      return std::nullopt;
    }
    if (generator.open.empty()) {
      if (problem_.search.use_progressive_widening &&
          generator.candidate_width < maximum_width) {
        generator.candidate_width = std::min(
            maximum_width,
            std::max(generator.candidate_width + 1,
                     generator.candidate_width * 2));
        ++instrumentation_.progressive_widening_stages;
        instrumentation_.max_candidate_width = std::max<std::uint64_t>(
            instrumentation_.max_candidate_width, generator.candidate_width);
        reset_frontier();
        continue;
      }
      generator.exhausted = true;
      return std::nullopt;
    }

    LowLevelConstraintNode node = std::move(generator.open.front());
    generator.open.pop();
    ++instrumentation_.low_level_constraint_nodes;

    std::vector<PrimitiveId> selected;
    std::vector<State> next;
    std::vector<ScalarInterval> next_boundary_rates;
    bool unique_result = false;
    if (pibt.plan(
            configuration, order, node.forced,
            generator.dynamic_info, generator.dynamic_candidates,
            selected, next)) {
      const std::string combo = primitive_combo_key(selected);
      if (generator.seen_joint_moves.insert(combo).second) {
        const bool no_good = violates_kinematic_no_good(
            configuration, previous_primitives, selected, next);
        bool kinematically_feasible = false;
        if (!no_good && !generator.dynamic_info.empty()) {
          kinematically_feasible = true;
          const bool terminal = is_goal(next);
          next_boundary_rates.resize(selected.size());
          for (std::size_t agent = 0; agent < selected.size(); ++agent) {
            const DynamicCandidateInfo& info =
                generator.dynamic_info[agent][selected[agent]];
            if (!info.feasible ||
                (terminal && !info.outgoing_rates.contains(0.0))) {
              kinematically_feasible = false;
              break;
            }
            next_boundary_rates[agent] = info.outgoing_rates;
          }
        } else if (!no_good) {
          kinematically_feasible = propagate_joint_boundary_rates(
              configuration,
              previous_primitives,
              current_boundary_rates,
              selected,
              next,
              next_boundary_rates);
        }
        if (!no_good && !kinematically_feasible) {
          ++instrumentation_.post_pibt_kinematic_rejects;
        }
        unique_result = !no_good && kinematically_feasible;
      } else {
        ++instrumentation_.joint_move_duplicates;
      }
    }

    if (node.depth >= order.size()) {
      if (unique_result) {
        ++generator.yielded;
        ++instrumentation_.joint_moves_generated;
        return JointMove{
            std::move(selected),
            std::move(next),
            std::move(next_boundary_rates)};
      }
      continue;
    }
    const int agent = order[node.depth];

    LowLevelConstraintNode skip = node;
    ++skip.depth;
    generator.open.push(std::move(skip));

    std::vector<PrimitiveId> candidate_scratch;
    std::vector<PrimitiveId> reachable_candidates;
    const std::vector<PrimitiveId>* available_candidates = nullptr;
    if (!generator.dynamic_info.empty()) {
      available_candidates =
          &generator.dynamic_candidates[static_cast<std::size_t>(agent)];
    } else {
      const std::vector<PrimitiveId>& geometry_candidates =
          pibt.ordered_candidates(
              static_cast<std::size_t>(agent),
              configuration[static_cast<std::size_t>(agent)],
              candidate_scratch);
      available_candidates = &geometry_candidates;
      // Ablation baseline: preserve the implementation that existed before
      // dynamics-aware PIBT. Low-level forced alternatives were filtered,
      // while PIBT recursion itself still scanned geometry-only candidates.
      if (problem_.primitive_config.use_acceleration_constraints &&
          !problem_.search.use_dynamics_aware_pibt) {
        reachable_candidates.reserve(geometry_candidates.size());
        for (PrimitiveId candidate : geometry_candidates) {
          ScalarInterval incoming{0.0, 0.0};
          if (!previous_primitives.empty()) {
            const PrimitiveId previous = previous_primitives[
                static_cast<std::size_t>(agent)];
            incoming = connect_boundary_rates(
                configuration[static_cast<std::size_t>(agent)].heading,
                previous,
                candidate,
                current_boundary_rates[static_cast<std::size_t>(agent)]);
          }
          if (!propagate_primitive_rates(
                   primitives_.primitive(candidate), incoming)
                   .empty()) {
            reachable_candidates.push_back(candidate);
          }
        }
        available_candidates = &reachable_candidates;
      }
    }
    const std::size_t limit = std::min(
        generator.candidate_width,
        available_candidates->size());

    std::vector<PrimitiveId> diversified;
    const std::vector<PrimitiveId>* alternatives = available_candidates;
    if (problem_.search.diversify_candidates &&
        primitives_.has_off_center_pivots() && limit > 0) {
      diversified.reserve(limit);
      enum class Family {
        kLongTranslation,
        kShortTranslation,
        kCenterRotation,
        kFrontPivot,
        kRearPivot,
        kWait,
      };
      const auto family = [&](PrimitiveId id) {
        const Primitive& primitive = primitives_.primitive(id);
        if (id == primitives_.wait_id()) return Family::kWait;
        if (!primitive.pivot_rotation) {
          const int distance = std::abs(primitive.dx) + std::abs(primitive.dy);
          return distance == primitives_.max_translation_cells()
                     ? Family::kLongTranslation
                     : Family::kShortTranslation;
        }
        if (primitive.pivot_offset_cells > 0) return Family::kFrontPivot;
        if (primitive.pivot_offset_cells < 0) return Family::kRearPivot;
        return Family::kCenterRotation;
      };
      std::vector<Family> used_families;
      for (PrimitiveId id : *available_candidates) {
        const Family candidate_family = family(id);
        if (std::find(
                used_families.begin(), used_families.end(), candidate_family) !=
            used_families.end()) {
          continue;
        }
        used_families.push_back(candidate_family);
        diversified.push_back(id);
        if (diversified.size() >= limit) break;
      }
      alternatives = &diversified;
    }

    for (std::size_t i = 0; i < alternatives->size() && i < limit; ++i) {
      LowLevelConstraintNode child = node;
      child.depth = node.depth + 1;
      child.forced[static_cast<std::size_t>(agent)] = (*alternatives)[i];

      std::ostringstream key;
      key << child.depth << ':';
      for (const auto& value : child.forced) {
        key << (value.has_value() ? static_cast<int>(*value) : -1) << ',';
      }
      if (generator.seen_constraint_nodes.insert(key.str()).second) {
        generator.open.push(std::move(child));
      }
    }

    if (unique_result) {
      ++generator.yielded;
      ++instrumentation_.joint_moves_generated;
      return JointMove{
          std::move(selected),
          std::move(next),
          std::move(next_boundary_rates)};
    }
  }
  return std::nullopt;
}

bool Planner::violates_kinematic_no_good(
    const std::vector<State>& configuration,
    const std::vector<PrimitiveId>& previous_primitives,
    const std::vector<PrimitiveId>& selected,
    const std::vector<State>& next) const {
  if (!problem_.primitive_config.use_acceleration_constraints) {
    return false;
  }
  for (std::size_t agent = 0; agent < selected.size(); ++agent) {
    const int previous = previous_primitives.empty()
                             ? -1
                             : static_cast<int>(previous_primitives[agent]);
    if (kinematic_no_goods_.count(KinematicNoGood{
            previous,
            static_cast<int>(selected[agent]),
            configuration[agent].heading,
            -1}) != 0) {
      return true;
    }
  }
  if (is_goal(next)) {
    for (std::size_t agent = 0; agent < selected.size(); ++agent) {
      if (kinematic_no_goods_.count(KinematicNoGood{
              static_cast<int>(selected[agent]),
              -2,
              next[agent].heading,
              static_cast<int>(agent)}) != 0) {
        return true;
      }
    }
  }
  return false;
}

bool Planner::propagate_joint_boundary_rates(
    const std::vector<State>& configuration,
    const std::vector<PrimitiveId>& previous_primitives,
    const std::vector<ScalarInterval>& current_boundary_rates,
    const std::vector<PrimitiveId>& selected,
    const std::vector<State>& next,
    std::vector<ScalarInterval>& next_boundary_rates) const {
  if (!problem_.primitive_config.use_acceleration_constraints) {
    next_boundary_rates.clear();
    return true;
  }
  next_boundary_rates.resize(selected.size());
  const bool terminal = is_goal(next);
  for (std::size_t agent = 0; agent < selected.size(); ++agent) {
    ScalarInterval incoming{0.0, 0.0};
    if (!previous_primitives.empty()) {
      const PrimitiveId previous_id = previous_primitives[agent];
      incoming = connect_boundary_rates(
          configuration[agent].heading,
          previous_id,
          selected[agent],
          current_boundary_rates[agent]);
    }
    const ScalarInterval outgoing = propagate_primitive_rates(
        primitives_.primitive(selected[agent]), incoming);
    if (outgoing.empty() || (terminal && !outgoing.contains(0.0))) {
      return false;
    }
    next_boundary_rates[agent] = outgoing;
  }
  return true;
}

ScalarInterval Planner::propagate_primitive_rates(
    const Primitive& primitive,
    const ScalarInterval& incoming) const {
  if (primitive.progress_coordinate == ProgressCoordinate::kStationary) {
    return incoming.contains(0.0)
               ? ScalarInterval{0.0, 0.0}
               : ScalarInterval{};
  }
  if (!primitive.kinematically_feasible) return ScalarInterval{};
  ++instrumentation_.cubic_relation_queries;
  return primitive.kinematic_relation.propagate(incoming);
}

ScalarInterval Planner::local_candidate_end_rates(
    const State& connection_state,
    int previous_primitive,
    PrimitiveId candidate_id) const {
  ScalarInterval candidate_incoming{0.0, 0.0};
  if (previous_primitive >= 0) {
    const Primitive& previous = primitives_.primitive(
        static_cast<PrimitiveId>(previous_primitive));
    const ScalarInterval arbitrary_previous_start =
        previous.progress_coordinate == ProgressCoordinate::kStationary
            ? ScalarInterval{0.0, 0.0}
            : ScalarInterval{0.0, previous.kinematic_max_rate};
    const ScalarInterval previous_end = propagate_primitive_rates(
        previous, arbitrary_previous_start);
    if (previous_end.empty()) return ScalarInterval{};
    candidate_incoming = connect_boundary_rates(
        connection_state.heading,
        static_cast<PrimitiveId>(previous_primitive),
        candidate_id,
        previous_end);
  }
  return propagate_primitive_rates(
      primitives_.primitive(candidate_id), candidate_incoming);
}

Planner::KinematicValidation Planner::validate_kinematics(
    const SearchAttempt& attempt) const {
  for (std::size_t agent = 0; agent < attempt.primitives.size(); ++agent) {
    const auto& primitive_ids = attempt.primitives[agent];
    const auto& states = attempt.states[agent];
    ScalarInterval incoming_rate{0.0, 0.0};
    for (std::size_t i = 0; i < primitive_ids.size(); ++i) {
      const Primitive& primitive = primitives_.primitive(primitive_ids[i]);
      if (primitive.progress_coordinate == ProgressCoordinate::kStationary) {
        if (!incoming_rate.contains(0.0)) {
          return KinematicValidation{false, agent, i};
        }
        const ScalarInterval outgoing_rate{0.0, 0.0};
        if (i + 1 == primitive_ids.size()) continue;
        incoming_rate = connect_boundary_rates(
            states[i + 1].heading,
            primitive_ids[i],
            primitive_ids[i + 1],
            outgoing_rate);
        continue;
      }
      const ScalarInterval outgoing_rate = propagate_primitive_rates(
          primitive, incoming_rate);
      if (outgoing_rate.empty()) {
        return KinematicValidation{false, agent, i};
      }
      if (i + 1 == primitive_ids.size()) {
        if (!outgoing_rate.contains(0.0)) {
          return KinematicValidation{false, agent, primitive_ids.size()};
        }
        continue;
      }

      incoming_rate = connect_boundary_rates(
          states[i + 1].heading,
          primitive_ids[i],
          primitive_ids[i + 1],
          outgoing_rate);
      if (incoming_rate.empty()) {
        return KinematicValidation{false, agent, i + 1};
      }
    }
  }
  return KinematicValidation{};
}

bool Planner::add_kinematic_no_good(
    const SearchAttempt& attempt,
    const KinematicValidation& failure) {
  const auto& primitive_ids = attempt.primitives.at(failure.agent);
  const auto& states = attempt.states.at(failure.agent);
  if (primitive_ids.empty()) return false;

  KinematicNoGood cut;
  if (failure.primitive_index == 0) {
    cut.previous_primitive = -1;
    cut.next_primitive = static_cast<int>(primitive_ids.front());
    cut.connection_heading = states.front().heading;
  } else if (failure.primitive_index >= primitive_ids.size()) {
    cut.previous_primitive = static_cast<int>(primitive_ids.back());
    cut.next_primitive = -2;
    cut.connection_heading = states.back().heading;
    cut.terminal_agent = static_cast<int>(failure.agent);
  } else {
    cut.previous_primitive = static_cast<int>(
        primitive_ids[failure.primitive_index - 1]);
    cut.next_primitive = static_cast<int>(
        primitive_ids[failure.primitive_index]);
    cut.connection_heading = states[failure.primitive_index].heading;
  }
  return kinematic_no_goods_.insert(cut).second;
}

bool Planner::generator_has_more(
    const JointMoveGeneratorState& generator) const {
  return !generator.exhausted &&
         generator.yielded < problem_.search.max_branching;
}

Planner::SearchAttempt Planner::reconstruct(
    const std::vector<SearchNode>& nodes,
    int goal_index) const {
  std::vector<int> chain;
  for (int index = goal_index;
       index >= 0;
       index = nodes[static_cast<std::size_t>(index)].parent) {
    chain.push_back(index);
  }
  std::reverse(chain.begin(), chain.end());

  SearchAttempt result;
  result.success = true;
  result.cost = nodes[static_cast<std::size_t>(goal_index)].g;
  result.states.resize(problem_.agents.size());
  result.primitives.resize(problem_.agents.size());
  result.boundary_rates.resize(problem_.agents.size());

  for (std::size_t agent = 0; agent < problem_.agents.size(); ++agent) {
    result.states[agent].push_back(
        nodes[static_cast<std::size_t>(chain.front())].configuration[agent]);
    result.boundary_rates[agent].push_back(
        problem_.primitive_config.use_acceleration_constraints
            ? nodes[static_cast<std::size_t>(chain.front())]
                  .boundary_rates[agent]
            : ScalarInterval{0.0, 0.0});
  }
  for (std::size_t c = 1; c < chain.size(); ++c) {
    const SearchNode& node = nodes[static_cast<std::size_t>(chain[c])];
    for (std::size_t agent = 0; agent < problem_.agents.size(); ++agent) {
      result.primitives[agent].push_back(node.incoming[agent]);
      result.states[agent].push_back(node.configuration[agent]);
      result.boundary_rates[agent].push_back(
          problem_.primitive_config.use_acceleration_constraints
              ? node.boundary_rates[agent]
              : ScalarInterval{0.0, 0.0});
    }
  }
  return result;
}

Planner::SearchAttempt Planner::weighted_search(
    double weight,
    double incumbent_cost,
    Deadline& deadline,
    std::uint64_t& expanded_nodes) {
  std::vector<State> start;
  start.reserve(problem_.agents.size());
  for (const Agent& agent : problem_.agents) start.push_back(agent.start);

  std::vector<SearchNode> nodes;
  SearchNode start_node;
  start_node.configuration = start;
  start_node.parent = -1;
  start_node.g = 0.0;
  start_node.h = heuristic(start);
  start_node.depth = 0;
  if (problem_.primitive_config.use_acceleration_constraints) {
    start_node.boundary_rates.assign(
        problem_.agents.size(), ScalarInterval{0.0, 0.0});
  }
  nodes.push_back(std::move(start_node));

  std::priority_queue<QueueEntry> open;
  std::uint64_t serial = 0;
  open.push(QueueEntry{
      weight * nodes[0].h,
      nodes[0].h,
      serial++,
      0,
      0});
  instrumentation_.max_open_size = std::max<std::uint64_t>(
      instrumentation_.max_open_size, open.size());

  std::unordered_map<SearchKey, double, SearchKeyHash> best_g;
  best_g[make_search_key(start, {}, nodes[0].boundary_rates)] = 0.0;
  std::unordered_map<BaseSearchKey, std::vector<int>, BaseSearchKeyHash>
      labels_by_base;
  labels_by_base[BaseSearchKey{start, {}}].push_back(0);

  PIBT pibt(
      problem_, primitives_, collision_checker_, transitions_, candidates_, random_,
      instrumentation_);
  std::size_t attempt_expansions = 0;

  while (!open.empty() &&
         !deadline.expired() &&
         attempt_expansions < problem_.search.max_expansions) {
    const QueueEntry entry = open.top();
    open.pop();
    const SearchNode& current_ref =
        nodes[static_cast<std::size_t>(entry.node_index)];
    const auto best = best_g.find(make_search_key(
        current_ref.configuration,
        current_ref.incoming,
        current_ref.boundary_rates));
    if (best == best_g.end() || current_ref.g > best->second + kEpsilon) {
      continue;
    }
    if (current_ref.g + current_ref.h >= incumbent_cost - kEpsilon) {
      continue;
    }
    if (is_goal(current_ref.configuration)) {
      return reconstruct(nodes, entry.node_index);
    }

    SearchNode& current_node =
        nodes[static_cast<std::size_t>(entry.node_index)];
    if (!current_node.expansion_counted) {
      current_node.expansion_counted = true;
      ++attempt_expansions;
      ++expanded_nodes;
    }
    const std::vector<State> configuration = current_ref.configuration;
    const double current_g = current_ref.g;
    const int current_depth = current_ref.depth;
    const std::vector<int> order = priority_order(configuration);
    if (!current_node.successor_generator) {
      current_node.successor_generator =
          std::make_shared<JointMoveGeneratorState>();
    }
    const std::size_t successor_quota =
        problem_.search.use_lazy_successors
            ? 1
            : problem_.search.max_branching;
    std::vector<JointMove> joint_moves;
    joint_moves.reserve(successor_quota);
    for (std::size_t generated = 0;
         generated < successor_quota && !deadline.expired();
         ++generated) {
      std::optional<JointMove> move = next_joint_move(
          configuration,
          current_node.incoming,
          current_node.boundary_rates,
          order,
          pibt,
          deadline,
          *current_node.successor_generator);
      if (!move.has_value()) break;
      joint_moves.push_back(std::move(*move));
    }
    if (generator_has_more(*current_node.successor_generator) &&
        !deadline.expired()) {
      ++instrumentation_.successor_continuations;
      open.push(QueueEntry{
          current_g + weight * current_ref.h,
          current_ref.h,
          serial++,
          0,
          entry.node_index});
      instrumentation_.max_open_size = std::max<std::uint64_t>(
          instrumentation_.max_open_size, open.size());
    }
    const double step_cost = edge_cost(configuration);

    for (const auto& move : joint_moves) {
      const std::vector<PrimitiveId>& incoming = move.primitives;
      const std::vector<State>& next = move.next;
      if (next == configuration) continue;

      const double next_g = current_g + step_cost;
      const double next_h = heuristic(next);
      if (next_g + next_h >= incumbent_cost - kEpsilon) continue;

      const SearchKey next_key = make_search_key(
          next, incoming, move.boundary_rates);
      const auto found = best_g.find(next_key);
      if (found != best_g.end() && found->second <= next_g + kEpsilon) {
        continue;
      }

      const BaseSearchKey base_key{
          next,
          problem_.primitive_config.use_acceleration_constraints
              ? incoming
              : std::vector<PrimitiveId>{}};
      bool dominated = false;
      const auto base_found = labels_by_base.find(base_key);
      if (problem_.search.use_interval_dominance &&
          base_found != labels_by_base.end()) {
        for (int label_index : base_found->second) {
          ++instrumentation_.dominance_check_count;
          const SearchNode& label =
              nodes[static_cast<std::size_t>(label_index)];
          if (label.g <= next_g + kEpsilon &&
              boundary_rates_dominate(
                  label.boundary_rates, move.boundary_rates)) {
            dominated = true;
            break;
          }
        }
      }
      if (dominated) {
        ++instrumentation_.kinematic_dominance_rejects;
        continue;
      }
      best_g[next_key] = next_g;

      SearchNode child;
      child.configuration = next;
      child.parent = entry.node_index;
      child.incoming = incoming;
      child.boundary_rates = move.boundary_rates;
      child.g = next_g;
      child.h = next_h;
      child.depth = current_depth + 1;
      nodes.push_back(std::move(child));

      const int index = static_cast<int>(nodes.size() - 1);
      labels_by_base[base_key].push_back(index);
      open.push(QueueEntry{
          next_g + weight * next_h,
          next_h,
          serial++,
          0,
          index});
      instrumentation_.max_open_size = std::max<std::uint64_t>(
          instrumentation_.max_open_size, open.size());
    }
  }
  return SearchAttempt{};
}

void Planner::publish_solution(
    const SearchAttempt& attempt,
    double weight,
    Deadline& deadline,
    Solution& solution) {
  if (!attempt.success || attempt.cost >= solution.cost - kEpsilon) return;

  if (problem_.primitive_config.use_acceleration_constraints) {
    ++kinematic_validation_calls_;
    const KinematicValidation validation = validate_kinematics(attempt);
    if (!validation.feasible) {
      ++kinematic_validation_failures_;
      kinematic_restart_requested_ = true;
      const bool inserted = add_kinematic_no_good(attempt, validation);
      std::cout << "kinematic candidate rejected: agent="
                << validation.agent
                << " boundary=" << validation.primitive_index
                << " cut=" << (inserted ? "added" : "duplicate")
                << " search_ms=" << deadline.elapsed_ms() << '\n';
      if (!inserted) {
        kinematic_restart_blocked_ = true;
      }
      return;
    }
  }

  solution.success = true;
  solution.cost = attempt.cost;
  std::vector<AgentPlan> plans(problem_.agents.size());
  for (std::size_t i = 0; i < problem_.agents.size(); ++i) {
    plans[i].states = attempt.states[i];
    plans[i].primitive_ids = attempt.primitives[i];
    if (problem_.primitive_config.use_acceleration_constraints) {
      plans[i].primitive_start_rates.reserve(attempt.primitives[i].size());
      plans[i].primitive_end_rates.reserve(attempt.primitives[i].size());
      for (std::size_t step = 0; step < attempt.primitives[i].size(); ++step) {
        ScalarInterval start_rate{0.0, 0.0};
        if (step > 0) {
          start_rate = connect_boundary_rates(
              attempt.states[i][step].heading,
              attempt.primitives[i][step - 1],
              attempt.primitives[i][step],
              attempt.boundary_rates[i][step]);
        }
        if (primitives_.primitive(attempt.primitives[i][step])
                .progress_coordinate == ProgressCoordinate::kStationary) {
          start_rate = ScalarInterval{0.0, 0.0};
        }
        plans[i].primitive_start_rates.push_back(start_rate);
        plans[i].primitive_end_rates.push_back(
            attempt.boundary_rates[i][step + 1]);
      }
    }
  }
  solution.plans = plans;
  solution.improvements.push_back(Improvement{
      deadline.elapsed_ms(), attempt.cost, weight, std::move(plans)});

  std::cout << "solution improved: cost=" << attempt.cost
            << " weight=" << weight
            << " search_ms=" << deadline.elapsed_ms() << '\n';
}

void Planner::publish_solution_from_node(
    const std::vector<SearchNode>& nodes,
    int goal_index,
    double weight,
    Deadline& deadline,
    Solution& solution) {
  if (goal_index < 0) return;
  publish_solution(reconstruct(nodes, goal_index), weight, deadline, solution);
}

void Planner::solve_repeated_weighted(
    Deadline& deadline,
    Solution& solution,
    std::uint64_t& expanded_nodes) {
  double weight = std::max(
      problem_.search.minimum_weight,
      problem_.search.initial_weight);

  while (!deadline.expired()) {
    const SearchAttempt attempt = weighted_search(
        weight, solution.cost, deadline, expanded_nodes);
    publish_solution(attempt, weight, deadline, solution);

    if (kinematic_restart_requested_) return;

    if (!problem_.search.anytime) break;
    if (weight <= problem_.search.minimum_weight + kEpsilon) break;
    weight = std::max(
        problem_.search.minimum_weight,
        weight - problem_.search.weight_step);
  }
}

void Planner::solve_ara_star(
    Deadline& deadline,
    Solution& solution,
    std::uint64_t& expanded_nodes) {
  std::vector<State> start;
  start.reserve(problem_.agents.size());
  for (const Agent& agent : problem_.agents) start.push_back(agent.start);

  std::vector<SearchNode> nodes;
  SearchNode start_node;
  start_node.configuration = start;
  start_node.parent = -1;
  start_node.g = 0.0;
  start_node.h = heuristic(start);
  start_node.depth = 0;
  if (problem_.primitive_config.use_acceleration_constraints) {
    start_node.boundary_rates.assign(
        problem_.agents.size(), ScalarInterval{0.0, 0.0});
  }
  nodes.push_back(std::move(start_node));

  std::unordered_map<SearchKey, int, SearchKeyHash> node_index;
  node_index.emplace(
      make_search_key(start, {}, nodes[0].boundary_rates), 0);
  std::unordered_map<BaseSearchKey, std::vector<int>, BaseSearchKeyHash>
      labels_by_base;
  labels_by_base[BaseSearchKey{start, {}}].push_back(0);

  std::priority_queue<QueueEntry> open;
  std::uint64_t serial = 0;
  double weight = std::max(
      problem_.search.minimum_weight,
      problem_.search.initial_weight);

  auto push_open = [&](int index) {
    SearchNode& node = nodes[static_cast<std::size_t>(index)];
    node.in_open = true;
    node.in_incons = false;
    ++node.open_stamp;
    open.push(QueueEntry{
        node.g + weight * node.h,
        node.h,
        serial++,
        node.open_stamp,
        index});
    instrumentation_.max_open_size = std::max<std::uint64_t>(
        instrumentation_.max_open_size, open.size());
  };

  auto discard_stale_open_entries = [&]() {
    while (!open.empty()) {
      const QueueEntry& entry = open.top();
      const SearchNode& node =
          nodes[static_cast<std::size_t>(entry.node_index)];
      if (node.in_open && entry.open_stamp == node.open_stamp) break;
      open.pop();
    }
  };

  push_open(0);
  int goal_index = is_goal(start) ? 0 : -1;
  if (goal_index == 0) {
    publish_solution_from_node(nodes, goal_index, weight, deadline, solution);
    if (kinematic_restart_requested_) return;
  }

  PIBT pibt(
      problem_, primitives_, collision_checker_, transitions_, candidates_, random_,
      instrumentation_);

  while (!deadline.expired() &&
         expanded_nodes < problem_.search.max_expansions) {
    // ImprovePath for the current epsilon/weight.
    while (!deadline.expired() &&
           expanded_nodes < problem_.search.max_expansions) {
      discard_stale_open_entries();
      if (open.empty()) break;

      const double goal_g =
          goal_index >= 0
              ? nodes[static_cast<std::size_t>(goal_index)].g
              : std::numeric_limits<double>::infinity();
      if (goal_g <= open.top().f + kEpsilon) break;

      const QueueEntry entry = open.top();
      open.pop();
      SearchNode& current_node =
          nodes[static_cast<std::size_t>(entry.node_index)];
      if (!current_node.in_open ||
          entry.open_stamp != current_node.open_stamp) {
        continue;
      }
      current_node.in_open = false;
      current_node.closed = false;
      if (!current_node.expansion_counted) {
        current_node.expansion_counted = true;
        ++expanded_nodes;
      }
      const std::vector<State> configuration = current_node.configuration;
      const double current_g = current_node.g;
      const int current_depth = current_node.depth;
      const std::vector<int> order = priority_order(configuration);
      if (!current_node.successor_generator) {
        current_node.successor_generator =
            std::make_shared<JointMoveGeneratorState>();
      }
      const std::size_t successor_quota =
          problem_.search.use_lazy_successors
              ? 1
              : problem_.search.max_branching;
      std::vector<JointMove> joint_moves;
      joint_moves.reserve(successor_quota);
      for (std::size_t generated = 0;
           generated < successor_quota && !deadline.expired();
           ++generated) {
        std::optional<JointMove> move = next_joint_move(
            configuration,
            current_node.incoming,
            current_node.boundary_rates,
            order,
            pibt,
            deadline,
            *current_node.successor_generator);
        if (!move.has_value()) break;
        joint_moves.push_back(std::move(*move));
      }
      const bool continuation_required =
          generator_has_more(*current_node.successor_generator) &&
          !deadline.expired();
      const double step_cost = edge_cost(configuration);

      for (const auto& move : joint_moves) {
        const std::vector<PrimitiveId>& incoming = move.primitives;
        const std::vector<State>& next = move.next;
        if (next == configuration) continue;

        const double next_g = current_g + step_cost;
        const double next_h = heuristic(next);
        if (next_g + next_h >= solution.cost - kEpsilon) continue;

        int next_index = -1;
        const SearchKey next_key = make_search_key(
            next, incoming, move.boundary_rates);
        const auto found = node_index.find(next_key);

        const BaseSearchKey base_key{
            next,
            problem_.primitive_config.use_acceleration_constraints
                ? incoming
                : std::vector<PrimitiveId>{}};
        bool dominated = false;
        const auto base_found = labels_by_base.find(base_key);
        if (problem_.search.use_interval_dominance &&
            base_found != labels_by_base.end()) {
          for (int label_index : base_found->second) {
            if (found != node_index.end() && label_index == found->second) {
              continue;
            }
            ++instrumentation_.dominance_check_count;
            const SearchNode& label =
                nodes[static_cast<std::size_t>(label_index)];
            if (label.g <= next_g + kEpsilon &&
                boundary_rates_dominate(
                    label.boundary_rates, move.boundary_rates)) {
              dominated = true;
              break;
            }
          }
        }
        if (dominated) {
          ++instrumentation_.kinematic_dominance_rejects;
          continue;
        }

        if (found == node_index.end()) {
          SearchNode child;
          child.configuration = next;
          child.h = next_h;
          nodes.push_back(std::move(child));
          next_index = static_cast<int>(nodes.size() - 1);
          node_index.emplace(next_key, next_index);
          labels_by_base[base_key].push_back(next_index);
        } else {
          next_index = found->second;
        }

        SearchNode& child = nodes[static_cast<std::size_t>(next_index)];
        if (next_g + kEpsilon >= child.g) continue;

        child.g = next_g;
        child.parent = entry.node_index;
        child.incoming = incoming;
        child.boundary_rates = move.boundary_rates;
        child.depth = current_depth + 1;
        child.successor_generator.reset();
        child.expansion_counted = false;

        if (!child.closed) {
          push_open(next_index);
        } else {
          child.in_incons = true;
        }

        if (is_goal(next)) {
          goal_index = next_index;
          publish_solution_from_node(
              nodes, goal_index, weight, deadline, solution);
          if (kinematic_restart_requested_) return;
        }
      }

      SearchNode& resumed =
          nodes[static_cast<std::size_t>(entry.node_index)];
      if (continuation_required) {
        ++instrumentation_.successor_continuations;
        resumed.closed = false;
        push_open(entry.node_index);
      } else {
        resumed.closed = true;
      }

    }

    if (!problem_.search.anytime) break;
    if (weight <= problem_.search.minimum_weight + kEpsilon) break;
    if (deadline.expired() ||
        expanded_nodes >= problem_.search.max_expansions) {
      break;
    }

    // Reuse all generated nodes. Only OPEN and INCONS need to be re-keyed;
    // CLOSED is cleared, as in ARA*.
    std::vector<int> reopen;
    reopen.reserve(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (nodes[i].in_open || nodes[i].in_incons) {
        reopen.push_back(static_cast<int>(i));
      }
    }
    if (reopen.empty()) break;

    weight = std::max(
        problem_.search.minimum_weight,
        weight - problem_.search.weight_step);
    open = std::priority_queue<QueueEntry>();
    for (SearchNode& node : nodes) {
      node.closed = false;
      node.in_open = false;
      node.in_incons = false;
    }
    for (int index : reopen) push_open(index);
  }
}

Solution Planner::solve() {
  transitions_.reset_runtime_stats();
  candidates_.reset_runtime_stats();
  collision_checker_.reset_runtime_stats();
  instrumentation_ = SearchInstrumentation{};
  kinematic_no_goods_.clear();
  kinematic_restart_requested_ = false;
  kinematic_restart_blocked_ = false;
  kinematic_validation_calls_ = 0;
  kinematic_validation_failures_ = 0;
  kinematic_search_restarts_ = 0;

  Deadline deadline(problem_.search.time_limit_ms);
  Solution solution;
  solution.objective = problem_.search.objective;

  std::uint64_t expanded_nodes = 0;
  const std::size_t main_max_branching = problem_.search.max_branching;
  const bool main_anytime = problem_.search.anytime;
  const bool use_initial_solution_pass =
      problem_.search.initial_solution_max_branching > 0 &&
      problem_.search.initial_solution_max_branching < main_max_branching;
  if (use_initial_solution_pass && !deadline.expired()) {
    problem_.search.max_branching =
        problem_.search.initial_solution_max_branching;
    problem_.search.anytime = false;
    if (problem_.search.use_ara_star) {
      solve_ara_star(deadline, solution, expanded_nodes);
    } else {
      solve_repeated_weighted(deadline, solution, expanded_nodes);
    }
    problem_.search.max_branching = main_max_branching;
    problem_.search.anytime = main_anytime;
  }

  do {
    if (solution.success && !main_anytime) break;
    kinematic_restart_requested_ = false;
    if (problem_.search.use_ara_star) {
      solve_ara_star(deadline, solution, expanded_nodes);
    } else {
      solve_repeated_weighted(deadline, solution, expanded_nodes);
    }
    if (kinematic_restart_requested_ &&
        !kinematic_restart_blocked_ &&
        !deadline.expired() &&
        expanded_nodes < problem_.search.max_expansions) {
      ++kinematic_search_restarts_;
      continue;
    }
    break;
  } while (true);

  solution.elapsed_ms = deadline.elapsed_ms();
  solution.stats.primitive_collision_precompute_ms =
      primitive_collision_precompute_ms_;
  solution.stats.transition_cache_precompute_ms =
      transitions_.precompute_ms();
  solution.stats.static_precompute_ms = static_precompute_ms_;
  solution.stats.candidate_cache_precompute_ms =
      candidates_.precompute_ms();
  solution.stats.connection_rule_precompute_ms =
      connection_rule_precompute_ms_;
  solution.stats.kinodynamic_lookahead_precompute_ms =
      kinodynamic_lookahead_precompute_ms_;
  solution.stats.query_precompute_ms = query_precompute_ms_;
  solution.stats.search_ms = solution.elapsed_ms;
  if (!solution.improvements.empty()) {
    solution.stats.first_solution_ms = solution.improvements.front().elapsed_ms;
    solution.stats.first_solution_cost = solution.improvements.front().cost;
  }
  solution.stats.cold_total_ms =
      static_precompute_ms_ + query_precompute_ms_ + solution.elapsed_ms;
  solution.stats.warm_request_ms =
      query_precompute_ms_ + solution.elapsed_ms;

  solution.stats.transition_cache_enabled = transitions_.cache_enabled();
  solution.stats.candidate_cache_enabled = candidates_.cache_enabled();
  solution.stats.ara_star_enabled = problem_.search.use_ara_star;
  solution.stats.reverse_bfs_heuristic_enabled =
      problem_.search.use_reverse_bfs_heuristic;
  solution.stats.candidate_diversification_enabled =
      problem_.search.diversify_candidates;
  solution.stats.aabb_broadphase_enabled =
      problem_.search.use_aabb_broadphase;
  solution.stats.conflict_cache_enabled =
      problem_.search.use_conflict_cache;
  solution.stats.lazy_successors_enabled =
      problem_.search.use_lazy_successors;
  solution.stats.progressive_widening_enabled =
      problem_.search.use_progressive_widening;
  solution.stats.initial_candidate_width =
      problem_.search.initial_candidate_width;
  solution.stats.initial_solution_max_branching =
      problem_.search.initial_solution_max_branching;
  solution.stats.per_primitive_intervals_enabled =
      problem_.search.use_per_primitive_intervals;
  solution.stats.multiple_rotation_amounts_enabled =
      problem_.primitive_config.use_multiple_rotation_amounts;
  solution.stats.acceleration_constraints_enabled =
      problem_.primitive_config.use_acceleration_constraints;
  solution.stats.dynamics_aware_pibt_enabled =
      problem_.search.use_dynamics_aware_pibt;
  solution.stats.interval_dominance_enabled =
      problem_.search.use_interval_dominance;
  solution.stats.kinodynamic_lookahead_enabled =
      problem_.search.use_kinodynamic_lookahead;
  solution.stats.kinodynamic_lookahead_depth =
      problem_.search.kinodynamic_lookahead_depth;
  solution.stats.kinodynamic_lookahead_entry_count =
      kinodynamic_lookahead_entry_count_;
  solution.stats.pivot_anchor_lattice_enabled =
      problem_.primitive_config.use_pivot_anchor_lattice;
  solution.stats.active_rotation_amount_count =
      primitives_.active_rotation_amount_count();
  solution.stats.collision_mode = problem_.search.collision_mode;
  solution.stats.max_boundary_travel_per_interval_m =
      problem_.search.max_boundary_travel_per_interval_m;
  solution.stats.collision_interval_count = primitives_.interval_count();
  solution.stats.collision_interval_polygon_count =
      primitives_.total_variant_intervals();

  solution.stats.transition_lookups = transitions_.lookups();
  solution.stats.transition_cache_hits = transitions_.cache_hits();
  solution.stats.transition_on_demand_computations =
      transitions_.on_demand_computations();

  solution.stats.candidate_lookups = candidates_.lookups();
  solution.stats.candidate_cache_hits = candidates_.cache_hits();
  solution.stats.candidate_on_demand_computations =
      candidates_.on_demand_computations();
  solution.stats.expanded_nodes = expanded_nodes;
  solution.stats.low_level_constraint_nodes =
      instrumentation_.low_level_constraint_nodes;
  solution.stats.pibt_plan_calls = instrumentation_.pibt_plan_calls;
  solution.stats.pibt_assign_calls = instrumentation_.pibt_assign_calls;
  solution.stats.pibt_candidate_attempts =
      instrumentation_.pibt_candidate_attempts;
  solution.stats.pibt_kinematic_candidate_rejects =
      instrumentation_.pibt_kinematic_candidate_rejects;
  solution.stats.pibt_backtracks = instrumentation_.pibt_backtracks;
  solution.stats.kinematic_dominance_rejects =
      instrumentation_.kinematic_dominance_rejects;
  solution.stats.dynamic_prefilter_ms =
      instrumentation_.dynamic_prefilter_ms;
  solution.stats.dynamic_prefilter_calls =
      instrumentation_.dynamic_prefilter_calls;
  solution.stats.dynamic_candidate_evaluations =
      instrumentation_.dynamic_candidate_evaluations;
  solution.stats.geometry_candidate_count =
      instrumentation_.geometry_candidate_count;
  solution.stats.dynamic_candidate_count =
      instrumentation_.dynamic_candidate_count;
  solution.stats.post_pibt_kinematic_rejects =
      instrumentation_.post_pibt_kinematic_rejects;
  solution.stats.dominance_check_count =
      instrumentation_.dominance_check_count;
  solution.stats.cubic_relation_queries =
      instrumentation_.cubic_relation_queries;
  solution.stats.connection_rule_queries =
      instrumentation_.connection_rule_queries;
  solution.stats.kinodynamic_lookahead_score_queries =
      instrumentation_.kinodynamic_lookahead_score_queries;
  solution.stats.kinodynamic_lookahead_sequences_evaluated =
      instrumentation_.kinodynamic_lookahead_sequences_evaluated;
  solution.stats.kinodynamic_lookahead_feasible_sequences =
      instrumentation_.kinodynamic_lookahead_feasible_sequences;
  solution.stats.joint_moves_generated =
      instrumentation_.joint_moves_generated;
  solution.stats.joint_move_duplicates =
      instrumentation_.joint_move_duplicates;
  solution.stats.max_open_size = instrumentation_.max_open_size;
  solution.stats.lazy_successor_requests =
      instrumentation_.lazy_successor_requests;
  solution.stats.successor_continuations =
      instrumentation_.successor_continuations;
  solution.stats.progressive_widening_stages =
      instrumentation_.progressive_widening_stages;
  solution.stats.max_candidate_width =
      instrumentation_.max_candidate_width;

  solution.stats.conflict_calls = collision_checker_.conflict_calls();
  solution.stats.conflict_cache_hits =
      collision_checker_.conflict_cache_hits();
  solution.stats.conflict_cache_canonical_swaps =
      collision_checker_.conflict_cache_canonical_swaps();
  solution.stats.conflict_cache_entries =
      collision_checker_.conflict_cache_entries();
  solution.stats.whole_step_aabb_tests =
      collision_checker_.whole_step_aabb_tests();
  solution.stats.whole_step_aabb_rejects =
      collision_checker_.whole_step_aabb_rejects();
  solution.stats.interval_aabb_tests =
      collision_checker_.interval_aabb_tests();
  solution.stats.interval_aabb_rejects =
      collision_checker_.interval_aabb_rejects();
  solution.stats.polygon_sat_tests = collision_checker_.polygon_sat_tests();
  solution.stats.kinematic_validation_calls = kinematic_validation_calls_;
  solution.stats.kinematic_validation_failures =
      kinematic_validation_failures_;
  solution.stats.kinematic_search_restarts = kinematic_search_restarts_;
  solution.stats.kinematic_no_good_count = kinematic_no_goods_.size();

  return solution;
}

}  // namespace lacam_primitive
