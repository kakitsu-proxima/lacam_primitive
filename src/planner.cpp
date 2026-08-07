#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

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

}  // namespace

CollisionChecker::CollisionChecker(
    const Problem& problem,
    const PrimitiveTable& primitives)
    : problem_(problem),
      primitives_(primitives),
      static_grid_(problem.grid.width_cells, problem.grid.height_cells) {
  for (const ObstacleRect& obstacle : problem.obstacles) {
    static_grid_.add_rect(obstacle);
  }
  static_grid_.finalize();
}

bool CollisionChecker::statically_valid(
    const State& start,
    PrimitiveId primitive_id) const {
  const PrimitiveVariant& variant = primitives_.variant(primitive_id, start.heading);
  for (const CellMask& mask : variant.interval_masks) {
    if (static_grid_.intersects(mask, start.x, start.y)) return false;
  }
  return true;
}

bool CollisionChecker::conflict(
    const State& left_start,
    PrimitiveId left_primitive,
    const State& right_start,
    PrimitiveId right_primitive) const {
  const PrimitiveVariant& left =
      primitives_.variant(left_primitive, left_start.heading);
  const PrimitiveVariant& right =
      primitives_.variant(right_primitive, right_start.heading);
  if (left.interval_masks.size() != right.interval_masks.size()) {
    throw std::logic_error("primitive interval counts differ");
  }
  for (std::size_t interval = 0; interval < left.interval_masks.size(); ++interval) {
    if (shifted_intersects(
            left.interval_masks[interval], left_start.x, left_start.y,
            right.interval_masks[interval], right_start.x, right_start.y)) {
      return true;
    }
  }
  return false;
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
  const State& goal = problem_.agents.at(agent).goal;
  const int manhattan =
      std::abs(state.x - goal.x) + std::abs(state.y - goal.y);
  const int translation_steps =
      (manhattan + primitives_.max_translation_cells() - 1) /
      primitives_.max_translation_cells();
  const int heading_distance = circular_heading_distance(
      state.heading, goal.heading, problem_.grid.heading_bins);
  const int rotation_steps =
      (heading_distance + primitives_.max_rotation_bins() - 1) /
      primitives_.max_rotation_bins();
  return static_cast<double>(translation_steps + rotation_steps);
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
    const TransitionEntry transition = transitions_.lookup(state, primitive.id);
    if (!transition.valid) continue;
    const int wait_penalty =
        primitive.id == primitives_.wait_id() ? (at_goal ? 0 : 1) : 0;
    ranked.push_back(Ranked{
        primitive.id,
        agent_heuristic(agent, transition.next),
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
  if (!cache_enabled_) {
    precompute_ms_ = 0.0;
    return;
  }

  const auto start = std::chrono::steady_clock::now();
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
    std::mt19937& random)
    : problem_(problem),
      primitives_(primitives),
      collision_checker_(collision_checker),
      transitions_(transitions),
      candidates_(candidates),
      random_(random) {}

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
    std::vector<PrimitiveId>& selected,
    std::vector<State>& next) {
  const std::size_t count = current.size();
  selected.assign(count, primitives_.wait_id());
  next = current;
  std::vector<bool> assigned(count, false);
  std::vector<bool> visiting(count, false);

  for (int agent : priority_order) {
    if (!assigned[static_cast<std::size_t>(agent)] &&
        !assign_agent(
            agent, current, forced, selected, next, assigned, visiting)) {
      return false;
    }
  }
  return true;
}

bool PIBT::assign_agent(
    int agent,
    const std::vector<State>& current,
    const std::vector<std::optional<PrimitiveId>>& forced,
    std::vector<PrimitiveId>& selected,
    std::vector<State>& next,
    std::vector<bool>& assigned,
    std::vector<bool>& visiting) {
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
  } else {
    candidates = &ordered_candidates(id, current[id], scratch);
  }

  for (PrimitiveId candidate : *candidates) {
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
  }

  visiting[id] = false;
  return false;
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
}

double Planner::single_agent_steps(
    std::size_t agent,
    const State& state) const {
  const State& goal = problem_.agents[agent].goal;
  const int manhattan =
      std::abs(state.x - goal.x) + std::abs(state.y - goal.y);
  const int translation_steps =
      (manhattan + primitives_.max_translation_cells() - 1) /
      primitives_.max_translation_cells();
  const int heading_distance = circular_heading_distance(
      state.heading, goal.heading, problem_.grid.heading_bins);
  const int rotation_steps =
      (heading_distance + primitives_.max_rotation_bins() - 1) /
      primitives_.max_rotation_bins();
  return static_cast<double>(translation_steps + rotation_steps);
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

std::vector<std::pair<std::vector<PrimitiveId>, std::vector<State>>>
Planner::generate_joint_moves(
    const std::vector<State>& configuration,
    const std::vector<int>& order,
    PIBT& pibt,
    Deadline& deadline) {
  struct LowLevelNode {
    std::size_t depth = 0;
    std::vector<std::optional<PrimitiveId>> forced;
  };

  std::queue<LowLevelNode> open;
  open.push(LowLevelNode{
      0,
      std::vector<std::optional<PrimitiveId>>(configuration.size())});
  std::unordered_set<std::string> seen_constraint_nodes;
  std::unordered_set<std::string> seen_joint_moves;
  std::vector<std::pair<std::vector<PrimitiveId>, std::vector<State>>> results;

  while (!open.empty() &&
         results.size() < problem_.search.max_branching &&
         !deadline.expired()) {
    LowLevelNode node = std::move(open.front());
    open.pop();

    std::vector<PrimitiveId> selected;
    std::vector<State> next;
    if (pibt.plan(configuration, order, node.forced, selected, next)) {
      const std::string combo = primitive_combo_key(selected);
      if (seen_joint_moves.insert(combo).second) {
        results.push_back({selected, next});
      }
    }

    if (node.depth >= order.size()) continue;
    const int agent = order[node.depth];

    LowLevelNode skip = node;
    ++skip.depth;
    open.push(std::move(skip));

    std::vector<PrimitiveId> candidate_scratch;
    const std::vector<PrimitiveId>& candidate_list =
        pibt.ordered_candidates(
            static_cast<std::size_t>(agent),
            configuration[static_cast<std::size_t>(agent)],
            candidate_scratch);
    const std::size_t limit = std::min(
        problem_.search.alternatives_per_agent,
        candidate_list.size());

    for (std::size_t i = 0; i < limit; ++i) {
      LowLevelNode child = node;
      child.depth = node.depth + 1;
      child.forced[static_cast<std::size_t>(agent)] = candidate_list[i];

      std::ostringstream key;
      key << child.depth << ':';
      for (const auto& value : child.forced) {
        key << (value.has_value() ? static_cast<int>(*value) : -1) << ',';
      }
      if (seen_constraint_nodes.insert(key.str()).second) {
        open.push(std::move(child));
      }
    }
  }
  return results;
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

  for (std::size_t agent = 0; agent < problem_.agents.size(); ++agent) {
    result.states[agent].push_back(
        nodes[static_cast<std::size_t>(chain.front())].configuration[agent]);
  }
  for (std::size_t c = 1; c < chain.size(); ++c) {
    const SearchNode& node = nodes[static_cast<std::size_t>(chain[c])];
    for (std::size_t agent = 0; agent < problem_.agents.size(); ++agent) {
      result.primitives[agent].push_back(node.incoming[agent]);
      result.states[agent].push_back(node.configuration[agent]);
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
  nodes.push_back(std::move(start_node));

  std::priority_queue<QueueEntry> open;
  std::uint64_t serial = 0;
  open.push(QueueEntry{
      weight * nodes[0].h,
      nodes[0].h,
      serial++,
      0,
      0});

  std::unordered_map<std::vector<State>, double, JointStateHash> best_g;
  best_g[start] = 0.0;

  PIBT pibt(
      problem_, primitives_, collision_checker_, transitions_, candidates_, random_);
  std::size_t attempt_expansions = 0;

  while (!open.empty() &&
         !deadline.expired() &&
         attempt_expansions < problem_.search.max_expansions) {
    const QueueEntry entry = open.top();
    open.pop();
    const SearchNode& current_ref =
        nodes[static_cast<std::size_t>(entry.node_index)];
    const auto best = best_g.find(current_ref.configuration);
    if (best == best_g.end() || current_ref.g > best->second + kEpsilon) {
      continue;
    }
    if (current_ref.g + current_ref.h >= incumbent_cost - kEpsilon) {
      continue;
    }
    if (is_goal(current_ref.configuration)) {
      return reconstruct(nodes, entry.node_index);
    }

    ++attempt_expansions;
    ++expanded_nodes;
    const std::vector<State> configuration = current_ref.configuration;
    const double current_g = current_ref.g;
    const int current_depth = current_ref.depth;
    const std::vector<int> order = priority_order(configuration);
    const auto joint_moves =
        generate_joint_moves(configuration, order, pibt, deadline);
    const double step_cost = edge_cost(configuration);

    for (const auto& move : joint_moves) {
      const std::vector<PrimitiveId>& incoming = move.first;
      const std::vector<State>& next = move.second;
      if (next == configuration) continue;

      const double next_g = current_g + step_cost;
      const double next_h = heuristic(next);
      if (next_g + next_h >= incumbent_cost - kEpsilon) continue;

      const auto found = best_g.find(next);
      if (found != best_g.end() && found->second <= next_g + kEpsilon) {
        continue;
      }
      best_g[next] = next_g;

      SearchNode child;
      child.configuration = next;
      child.parent = entry.node_index;
      child.incoming = incoming;
      child.g = next_g;
      child.h = next_h;
      child.depth = current_depth + 1;
      nodes.push_back(std::move(child));

      const int index = static_cast<int>(nodes.size() - 1);
      open.push(QueueEntry{
          next_g + weight * next_h,
          next_h,
          serial++,
          0,
          index});
    }
  }
  return SearchAttempt{};
}

void Planner::publish_solution(
    const SearchAttempt& attempt,
    double weight,
    Deadline& deadline,
    Solution& solution) const {
  if (!attempt.success || attempt.cost >= solution.cost - kEpsilon) return;

  solution.success = true;
  solution.cost = attempt.cost;
  solution.plans.resize(problem_.agents.size());
  for (std::size_t i = 0; i < problem_.agents.size(); ++i) {
    solution.plans[i].states = attempt.states[i];
    solution.plans[i].primitive_ids = attempt.primitives[i];
  }
  solution.improvements.push_back(
      Improvement{deadline.elapsed_ms(), attempt.cost, weight});

  std::cout << "solution improved: cost=" << attempt.cost
            << " weight=" << weight
            << " search_ms=" << deadline.elapsed_ms() << '\n';
}

void Planner::publish_solution_from_node(
    const std::vector<SearchNode>& nodes,
    int goal_index,
    double weight,
    Deadline& deadline,
    Solution& solution) const {
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
  nodes.push_back(std::move(start_node));

  std::unordered_map<std::vector<State>, int, JointStateHash> node_index;
  node_index.emplace(start, 0);

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
  }

  PIBT pibt(
      problem_, primitives_, collision_checker_, transitions_, candidates_, random_);

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
      current_node.closed = true;

      ++expanded_nodes;
      const std::vector<State> configuration = current_node.configuration;
      const double current_g = current_node.g;
      const int current_depth = current_node.depth;
      const std::vector<int> order = priority_order(configuration);
      const auto joint_moves =
          generate_joint_moves(configuration, order, pibt, deadline);
      const double step_cost = edge_cost(configuration);

      for (const auto& move : joint_moves) {
        const std::vector<PrimitiveId>& incoming = move.first;
        const std::vector<State>& next = move.second;
        if (next == configuration) continue;

        const double next_g = current_g + step_cost;
        const double next_h = heuristic(next);
        if (next_g + next_h >= solution.cost - kEpsilon) continue;

        int next_index = -1;
        const auto found = node_index.find(next);
        if (found == node_index.end()) {
          SearchNode child;
          child.configuration = next;
          child.h = next_h;
          nodes.push_back(std::move(child));
          next_index = static_cast<int>(nodes.size() - 1);
          node_index.emplace(next, next_index);
        } else {
          next_index = found->second;
        }

        SearchNode& child = nodes[static_cast<std::size_t>(next_index)];
        if (next_g + kEpsilon >= child.g) continue;

        child.g = next_g;
        child.parent = entry.node_index;
        child.incoming = incoming;
        child.depth = current_depth + 1;

        if (!child.closed) {
          push_open(next_index);
        } else {
          child.in_incons = true;
        }

        if (is_goal(next)) {
          goal_index = next_index;
          publish_solution_from_node(
              nodes, goal_index, weight, deadline, solution);
        }
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

  Deadline deadline(problem_.search.time_limit_ms);
  Solution solution;
  solution.objective = problem_.search.objective;

  std::uint64_t expanded_nodes = 0;
  if (problem_.search.use_ara_star) {
    solve_ara_star(deadline, solution, expanded_nodes);
  } else {
    solve_repeated_weighted(deadline, solution, expanded_nodes);
  }

  solution.elapsed_ms = deadline.elapsed_ms();
  solution.stats.primitive_collision_precompute_ms =
      primitive_collision_precompute_ms_;
  solution.stats.transition_cache_precompute_ms =
      transitions_.precompute_ms();
  solution.stats.static_precompute_ms = static_precompute_ms_;
  solution.stats.candidate_cache_precompute_ms =
      candidates_.precompute_ms();
  solution.stats.query_precompute_ms = query_precompute_ms_;
  solution.stats.search_ms = solution.elapsed_ms;
  solution.stats.cold_total_ms =
      static_precompute_ms_ + query_precompute_ms_ + solution.elapsed_ms;
  solution.stats.warm_request_ms =
      query_precompute_ms_ + solution.elapsed_ms;

  solution.stats.transition_cache_enabled = transitions_.cache_enabled();
  solution.stats.candidate_cache_enabled = candidates_.cache_enabled();
  solution.stats.ara_star_enabled = problem_.search.use_ara_star;

  solution.stats.transition_lookups = transitions_.lookups();
  solution.stats.transition_cache_hits = transitions_.cache_hits();
  solution.stats.transition_on_demand_computations =
      transitions_.on_demand_computations();

  solution.stats.candidate_lookups = candidates_.lookups();
  solution.stats.candidate_cache_hits = candidates_.cache_hits();
  solution.stats.candidate_on_demand_computations =
      candidates_.on_demand_computations();
  solution.stats.expanded_nodes = expanded_nodes;

  return solution;
}

}  // namespace lacam_primitive
