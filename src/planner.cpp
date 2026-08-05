#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>

namespace lacam_primitive {
namespace {

std::string primitive_combo_key(const std::vector<PrimitiveId>& primitives) {
  std::string key;
  key.reserve(primitives.size() * 4);
  for (PrimitiveId id : primitives) {
    key.append(reinterpret_cast<const char*>(&id), sizeof(id));
  }
  return key;
}

}  // namespace

CollisionChecker::CollisionChecker(const Problem& problem, const PrimitiveTable& primitives)
    : problem_(problem),
      primitives_(primitives),
      static_grid_(problem.grid.width_cells, problem.grid.height_cells) {
  for (const ObstacleRect& obstacle : problem.obstacles) static_grid_.add_rect(obstacle);
  static_grid_.finalize();
}

bool CollisionChecker::statically_valid(const State& start, PrimitiveId primitive_id) const {
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
  const PrimitiveVariant& left = primitives_.variant(left_primitive, left_start.heading);
  const PrimitiveVariant& right = primitives_.variant(right_primitive, right_start.heading);
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

PIBT::PIBT(
    const Problem& problem,
    const PrimitiveTable& primitives,
    const CollisionChecker& collision_checker,
    std::mt19937& random)
    : problem_(problem),
      primitives_(primitives),
      collision_checker_(collision_checker),
      random_(random) {}

double PIBT::agent_heuristic(std::size_t agent, const State& state) const {
  const State& goal = problem_.agents.at(agent).goal;
  const int manhattan = std::abs(state.x - goal.x) + std::abs(state.y - goal.y);
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

std::vector<PrimitiveId> PIBT::ordered_candidates(std::size_t agent, const State& state) const {
  struct Ranked {
    PrimitiveId id;
    double score;
    int wait_penalty;
  };
  std::vector<Ranked> ranked;
  ranked.reserve(primitives_.primitives().size());
  const bool at_goal = state == problem_.agents.at(agent).goal;
  for (const Primitive& primitive : primitives_.primitives()) {
    if (!collision_checker_.statically_valid(state, primitive.id)) continue;
    const State next = primitives_.apply(state, primitive.id);
    const int wait_penalty = primitive.id == primitives_.wait_id() ? (at_goal ? 0 : 1) : 0;
    ranked.push_back(Ranked{primitive.id, agent_heuristic(agent, next), wait_penalty});
  }
  std::stable_sort(ranked.begin(), ranked.end(), [](const Ranked& left, const Ranked& right) {
    if (left.score != right.score) return left.score < right.score;
    if (left.wait_penalty != right.wait_penalty) return left.wait_penalty < right.wait_penalty;
    return left.id < right.id;
  });
  std::vector<PrimitiveId> result;
  result.reserve(ranked.size());
  for (const Ranked& item : ranked) result.push_back(item.id);
  return result;
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
        !assign_agent(agent, current, forced, selected, next, assigned, visiting)) {
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

  std::vector<PrimitiveId> candidates;
  if (forced[id].has_value()) {
    candidates.push_back(*forced[id]);
  } else {
    candidates = ordered_candidates(id, current[id]);
  }

  for (PrimitiveId candidate : candidates) {
    if (!collision_checker_.statically_valid(current[id], candidate)) continue;

    const auto selected_snapshot = selected;
    const auto next_snapshot = next;
    const auto assigned_snapshot = assigned;
    const auto visiting_snapshot = visiting;

    selected[id] = candidate;
    next[id] = primitives_.apply(current[id], candidate);
    assigned[id] = true;

    bool valid = true;
    for (std::size_t other = 0; other < current.size() && valid; ++other) {
      if (other == id || !assigned[other]) continue;
      if (collision_checker_.conflict(current[id], candidate, current[other], selected[other])) {
        valid = false;
      }
    }

    for (std::size_t other = 0; other < current.size() && valid; ++other) {
      if (other == id || assigned[other]) continue;
      if (!collision_checker_.conflict(
              current[id], candidate, current[other], primitives_.wait_id())) {
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
        if (collision_checker_.conflict(current[id], candidate, current[other], selected[other])) {
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
    : problem_(std::move(problem)),
      primitives_(problem_),
      collision_checker_(problem_, primitives_),
      random_(problem_.search.random_seed) {
  if (problem_.agents.empty()) throw std::invalid_argument("at least one agent is required");
  if (problem_.search.objective != "sum_of_costs" && problem_.search.objective != "makespan") {
    throw std::invalid_argument("objective must be sum_of_costs or makespan");
  }

  for (std::size_t i = 0; i < problem_.agents.size(); ++i) {
    const Agent& agent = problem_.agents[i];
    if (!collision_checker_.statically_valid(agent.start, primitives_.wait_id())) {
      throw std::invalid_argument("agent " + std::to_string(i) + " start footprint is invalid");
    }
    if (!collision_checker_.statically_valid(agent.goal, primitives_.wait_id())) {
      throw std::invalid_argument("agent " + std::to_string(i) + " goal footprint is invalid");
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
}

double Planner::single_agent_steps(std::size_t agent, const State& state) const {
  const State& goal = problem_.agents[agent].goal;
  const int manhattan = std::abs(state.x - goal.x) + std::abs(state.y - goal.y);
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

double Planner::heuristic(const std::vector<State>& configuration) const {
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

double Planner::edge_cost(const std::vector<State>& configuration) const {
  if (problem_.search.objective == "makespan") return problem_.grid.macro_dt;
  std::size_t unfinished = 0;
  for (std::size_t i = 0; i < configuration.size(); ++i) {
    if (!(configuration[i] == problem_.agents[i].goal)) ++unfinished;
  }
  return static_cast<double>(unfinished) * problem_.grid.macro_dt;
}

bool Planner::is_goal(const std::vector<State>& configuration) const {
  for (std::size_t i = 0; i < configuration.size(); ++i) {
    if (!(configuration[i] == problem_.agents[i].goal)) return false;
  }
  return true;
}

std::vector<int> Planner::priority_order(const std::vector<State>& configuration) const {
  std::vector<int> order(configuration.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int left, int right) {
    const double left_h = single_agent_steps(static_cast<std::size_t>(left), configuration[static_cast<std::size_t>(left)]);
    const double right_h = single_agent_steps(static_cast<std::size_t>(right), configuration[static_cast<std::size_t>(right)]);
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
  open.push(LowLevelNode{0, std::vector<std::optional<PrimitiveId>>(configuration.size())});
  std::unordered_set<std::string> seen_constraint_nodes;
  std::unordered_set<std::string> seen_joint_moves;
  std::vector<std::pair<std::vector<PrimitiveId>, std::vector<State>>> results;

  while (!open.empty() && results.size() < problem_.search.max_branching && !deadline.expired()) {
    LowLevelNode node = std::move(open.front());
    open.pop();

    std::vector<PrimitiveId> selected;
    std::vector<State> next;
    if (pibt.plan(configuration, order, node.forced, selected, next)) {
      const std::string combo = primitive_combo_key(selected);
      if (seen_joint_moves.insert(combo).second) results.push_back({selected, next});
    }

    if (node.depth >= order.size()) continue;
    const int agent = order[node.depth];

    LowLevelNode skip = node;
    ++skip.depth;
    open.push(std::move(skip));

    const std::vector<PrimitiveId> candidates =
        pibt.ordered_candidates(static_cast<std::size_t>(agent), configuration[static_cast<std::size_t>(agent)]);
    const std::size_t limit = std::min(problem_.search.alternatives_per_agent, candidates.size());
    for (std::size_t i = 0; i < limit; ++i) {
      LowLevelNode child = node;
      child.depth = node.depth + 1;
      child.forced[static_cast<std::size_t>(agent)] = candidates[i];
      std::ostringstream key;
      key << child.depth << ':';
      for (const auto& value : child.forced) {
        key << (value.has_value() ? static_cast<int>(*value) : -1) << ',';
      }
      if (seen_constraint_nodes.insert(key.str()).second) open.push(std::move(child));
    }
  }
  return results;
}

Planner::SearchAttempt Planner::reconstruct(
    const std::vector<SearchNode>& nodes,
    int goal_index) const {
  std::vector<int> chain;
  for (int index = goal_index; index >= 0; index = nodes[static_cast<std::size_t>(index)].parent) {
    chain.push_back(index);
  }
  std::reverse(chain.begin(), chain.end());

  SearchAttempt result;
  result.success = true;
  result.cost = nodes[static_cast<std::size_t>(goal_index)].g;
  result.states.resize(problem_.agents.size());
  result.primitives.resize(problem_.agents.size());

  for (std::size_t agent = 0; agent < problem_.agents.size(); ++agent) {
    result.states[agent].push_back(nodes[static_cast<std::size_t>(chain.front())].configuration[agent]);
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
    Deadline& deadline) {
  std::vector<State> start;
  start.reserve(problem_.agents.size());
  for (const Agent& agent : problem_.agents) start.push_back(agent.start);

  std::vector<SearchNode> nodes;
  nodes.push_back(SearchNode{start, -1, {}, 0.0, heuristic(start), 0});
  std::priority_queue<QueueEntry> open;
  std::uint64_t serial = 0;
  open.push(QueueEntry{weight * nodes[0].h, nodes[0].h, serial++, 0});
  std::unordered_map<std::vector<State>, double, JointStateHash> best_g;
  best_g[start] = 0.0;
  PIBT pibt(problem_, primitives_, collision_checker_, random_);
  std::size_t expansions = 0;

  while (!open.empty() && !deadline.expired() && expansions < problem_.search.max_expansions) {
    const QueueEntry entry = open.top();
    open.pop();
    const SearchNode& current_ref = nodes[static_cast<std::size_t>(entry.node_index)];
    const auto best = best_g.find(current_ref.configuration);
    if (best == best_g.end() || current_ref.g > best->second + 1e-9) continue;
    if (current_ref.g + current_ref.h >= incumbent_cost - 1e-9) continue;
    if (is_goal(current_ref.configuration)) return reconstruct(nodes, entry.node_index);

    ++expansions;
    const std::vector<State> configuration = current_ref.configuration;
    const double current_g = current_ref.g;
    const int current_depth = current_ref.depth;
    const std::vector<int> order = priority_order(configuration);
    const auto joint_moves = generate_joint_moves(configuration, order, pibt, deadline);
    const double step_cost = edge_cost(configuration);

    for (const auto& [incoming, next] : joint_moves) {
      if (next == configuration) continue;
      const double next_g = current_g + step_cost;
      const double next_h = heuristic(next);
      if (next_g + next_h >= incumbent_cost - 1e-9) continue;
      const auto found = best_g.find(next);
      if (found != best_g.end() && found->second <= next_g + 1e-9) continue;
      best_g[next] = next_g;
      nodes.push_back(SearchNode{next, entry.node_index, incoming, next_g, next_h, current_depth + 1});
      const int index = static_cast<int>(nodes.size() - 1);
      open.push(QueueEntry{next_g + weight * next_h, next_h, serial++, index});
    }
  }
  return SearchAttempt{};
}

Solution Planner::solve() {
  Deadline deadline(problem_.search.time_limit_ms);
  Solution solution;
  solution.objective = problem_.search.objective;

  double incumbent = std::numeric_limits<double>::infinity();
  double weight = std::max(problem_.search.minimum_weight, problem_.search.initial_weight);
  bool first_iteration = true;

  while (!deadline.expired()) {
    SearchAttempt attempt = weighted_search(weight, incumbent, deadline);
    if (attempt.success && attempt.cost < incumbent - 1e-9) {
      incumbent = attempt.cost;
      solution.success = true;
      solution.cost = attempt.cost;
      solution.plans.resize(problem_.agents.size());
      for (std::size_t i = 0; i < problem_.agents.size(); ++i) {
        solution.plans[i].states = std::move(attempt.states[i]);
        solution.plans[i].primitive_ids = std::move(attempt.primitives[i]);
      }
      solution.improvements.push_back(Improvement{deadline.elapsed_ms(), incumbent, weight});
      std::cout << "solution improved: cost=" << incumbent
                << " weight=" << weight
                << " elapsed_ms=" << deadline.elapsed_ms() << '\n';
    }

    if (!problem_.search.anytime) break;
    if (weight <= problem_.search.minimum_weight + 1e-9) {
      if (!attempt.success) break;
      // At weight 1, another strict-bound search proves no cheaper solution or finds one.
      continue;
    }
    weight = std::max(problem_.search.minimum_weight, weight - problem_.search.weight_step);
    first_iteration = false;
    (void)first_iteration;
  }

  solution.elapsed_ms = deadline.elapsed_ms();
  return solution;
}

}  // namespace lacam_primitive
