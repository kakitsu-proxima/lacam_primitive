#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "primitives.hpp"
#include "types.hpp"

namespace lacam_primitive {

class CollisionChecker {
 public:
  CollisionChecker(const Problem& problem, const PrimitiveTable& primitives);

  [[nodiscard]] bool statically_valid(
      const State& start,
      PrimitiveId primitive_id) const;

  [[nodiscard]] bool conflict(
      const State& left_start,
      PrimitiveId left_primitive,
      const State& right_start,
      PrimitiveId right_primitive) const;

  void reset_runtime_stats() const;
  [[nodiscard]] std::uint64_t conflict_calls() const { return conflict_calls_; }
  [[nodiscard]] std::uint64_t conflict_cache_hits() const {
    return conflict_cache_hits_;
  }
  [[nodiscard]] std::uint64_t conflict_cache_canonical_swaps() const {
    return conflict_cache_canonical_swaps_;
  }
  [[nodiscard]] std::size_t conflict_cache_entries() const {
    return conflict_cache_.size();
  }
  [[nodiscard]] std::uint64_t whole_step_aabb_tests() const {
    return whole_step_aabb_tests_;
  }
  [[nodiscard]] std::uint64_t whole_step_aabb_rejects() const {
    return whole_step_aabb_rejects_;
  }
  [[nodiscard]] std::uint64_t interval_aabb_tests() const {
    return interval_aabb_tests_;
  }
  [[nodiscard]] std::uint64_t interval_aabb_rejects() const {
    return interval_aabb_rejects_;
  }
  [[nodiscard]] std::uint64_t polygon_sat_tests() const {
    return polygon_sat_tests_;
  }

 private:
  struct ConflictKey {
    int dx = 0;
    int dy = 0;
    int left_heading = 0;
    int right_heading = 0;
    PrimitiveId left_primitive = 0;
    PrimitiveId right_primitive = 0;

    bool operator==(const ConflictKey& other) const noexcept {
      return dx == other.dx && dy == other.dy &&
             left_heading == other.left_heading &&
             right_heading == other.right_heading &&
             left_primitive == other.left_primitive &&
             right_primitive == other.right_primitive;
    }
  };

  struct ConflictKeyHash {
    std::size_t operator()(const ConflictKey& key) const noexcept;
  };

  const Problem& problem_;
  const PrimitiveTable& primitives_;
  std::vector<ConvexPolygon> obstacle_polygons_;
  mutable std::unordered_map<ConflictKey, bool, ConflictKeyHash> conflict_cache_;
  mutable std::uint64_t conflict_calls_ = 0;
  mutable std::uint64_t conflict_cache_hits_ = 0;
  mutable std::uint64_t conflict_cache_canonical_swaps_ = 0;
  mutable std::uint64_t whole_step_aabb_tests_ = 0;
  mutable std::uint64_t whole_step_aabb_rejects_ = 0;
  mutable std::uint64_t interval_aabb_tests_ = 0;
  mutable std::uint64_t interval_aabb_rejects_ = 0;
  mutable std::uint64_t polygon_sat_tests_ = 0;
};

struct SearchInstrumentation {
  std::uint64_t low_level_constraint_nodes = 0;
  std::uint64_t pibt_plan_calls = 0;
  std::uint64_t pibt_assign_calls = 0;
  std::uint64_t pibt_candidate_attempts = 0;
  std::uint64_t pibt_backtracks = 0;
  std::uint64_t joint_moves_generated = 0;
  std::uint64_t joint_move_duplicates = 0;
  std::uint64_t max_open_size = 0;
  std::uint64_t lazy_successor_requests = 0;
  std::uint64_t successor_continuations = 0;
  std::uint64_t progressive_widening_stages = 0;
  std::uint64_t max_candidate_width = 0;
};

struct TransitionEntry {
  State next;
  bool valid = false;
};

class TransitionProvider {
 public:
  TransitionProvider(
      const Problem& problem,
      const PrimitiveTable& primitives,
      const CollisionChecker& collision_checker,
      bool cache_enabled);

  void build_cache();

  [[nodiscard]] TransitionEntry lookup(
      const State& state,
      PrimitiveId primitive_id) const;

  [[nodiscard]] bool cache_enabled() const { return cache_enabled_; }
  [[nodiscard]] double precompute_ms() const { return precompute_ms_; }
  [[nodiscard]] std::size_t state_count() const { return state_count_; }

  [[nodiscard]] std::uint64_t lookups() const { return lookups_; }
  [[nodiscard]] std::uint64_t cache_hits() const { return cache_hits_; }
  [[nodiscard]] std::uint64_t on_demand_computations() const {
    return on_demand_computations_;
  }

  void reset_runtime_stats() const;

 private:
  const Problem& problem_;
  const PrimitiveTable& primitives_;
  const CollisionChecker& collision_checker_;
  bool cache_enabled_ = false;
  std::size_t state_count_ = 0;
  std::size_t primitive_count_ = 0;
  std::vector<TransitionEntry> cache_;
  double precompute_ms_ = 0.0;

  mutable std::uint64_t lookups_ = 0;
  mutable std::uint64_t cache_hits_ = 0;
  mutable std::uint64_t on_demand_computations_ = 0;

  [[nodiscard]] std::size_t state_index(const State& state) const;
  [[nodiscard]] std::size_t cache_index(
      const State& state,
      PrimitiveId primitive_id) const;
  [[nodiscard]] TransitionEntry compute(
      const State& state,
      PrimitiveId primitive_id) const;
};

class CandidateProvider {
 public:
  CandidateProvider(
      const Problem& problem,
      const PrimitiveTable& primitives,
      const TransitionProvider& transitions,
      bool cache_enabled);

  void build_cache();

  // If the cache is enabled, the returned reference points into the cache.
  // Otherwise scratch is overwritten and returned.
  [[nodiscard]] const std::vector<PrimitiveId>& ordered_candidates(
      std::size_t agent,
      const State& state,
      std::vector<PrimitiveId>& scratch) const;

  [[nodiscard]] double agent_distance(
      std::size_t agent,
      const State& state) const;

  [[nodiscard]] bool cache_enabled() const { return cache_enabled_; }
  [[nodiscard]] double precompute_ms() const { return precompute_ms_; }

  [[nodiscard]] std::uint64_t lookups() const { return lookups_; }
  [[nodiscard]] std::uint64_t cache_hits() const { return cache_hits_; }
  [[nodiscard]] std::uint64_t on_demand_computations() const {
    return on_demand_computations_;
  }

  void reset_runtime_stats() const;

 private:
  const Problem& problem_;
  const PrimitiveTable& primitives_;
  const TransitionProvider& transitions_;
  bool cache_enabled_ = false;
  std::size_t state_count_ = 0;
  std::vector<std::vector<PrimitiveId>> cache_;
  std::vector<std::vector<int>> reverse_edges_;
  std::vector<std::vector<int>> distance_to_goal_;
  double precompute_ms_ = 0.0;

  mutable std::uint64_t lookups_ = 0;
  mutable std::uint64_t cache_hits_ = 0;
  mutable std::uint64_t on_demand_computations_ = 0;

  [[nodiscard]] std::size_t state_index(const State& state) const;
  [[nodiscard]] std::size_t cache_index(
      std::size_t agent,
      const State& state) const;
  [[nodiscard]] double agent_heuristic(
      std::size_t agent,
      const State& state) const;
  [[nodiscard]] std::vector<PrimitiveId> compute(
      std::size_t agent,
      const State& state) const;
  [[nodiscard]] State state_from_index(std::size_t index) const;
  void build_reverse_distances();
};

class PIBT {
 public:
  PIBT(
      const Problem& problem,
      const PrimitiveTable& primitives,
      const CollisionChecker& collision_checker,
      const TransitionProvider& transitions,
      const CandidateProvider& candidates,
      std::mt19937& random,
      SearchInstrumentation& instrumentation);

  [[nodiscard]] const std::vector<PrimitiveId>& ordered_candidates(
      std::size_t agent,
      const State& state,
      std::vector<PrimitiveId>& scratch) const;

  bool plan(
      const std::vector<State>& current,
      const std::vector<int>& priority_order,
      const std::vector<std::optional<PrimitiveId>>& forced,
      std::vector<PrimitiveId>& selected,
      std::vector<State>& next);

 private:
  const Problem& problem_;
  const PrimitiveTable& primitives_;
  const CollisionChecker& collision_checker_;
  const TransitionProvider& transitions_;
  const CandidateProvider& candidates_;
  std::mt19937& random_;
  SearchInstrumentation& instrumentation_;

  bool assign_agent(
      int agent,
      const std::vector<State>& current,
      const std::vector<std::optional<PrimitiveId>>& forced,
      std::vector<PrimitiveId>& selected,
      std::vector<State>& next,
      std::vector<bool>& assigned,
      std::vector<bool>& visiting);
};

class Planner {
 public:
  explicit Planner(Problem problem);

  [[nodiscard]] Solution solve();

 private:
  using JointMove =
      std::pair<std::vector<PrimitiveId>, std::vector<State>>;

  struct LowLevelConstraintNode {
    std::size_t depth = 0;
    std::vector<std::optional<PrimitiveId>> forced;
  };

  struct JointMoveGeneratorState {
    std::queue<LowLevelConstraintNode> open;
    std::unordered_set<std::string> seen_constraint_nodes;
    std::unordered_set<std::string> seen_joint_moves;
    std::size_t candidate_width = 0;
    std::size_t yielded = 0;
    bool initialized = false;
    bool exhausted = false;
  };

  struct SearchNode {
    std::vector<State> configuration;
    int parent = -1;
    std::vector<PrimitiveId> incoming;
    double g = std::numeric_limits<double>::infinity();
    double h = 0.0;
    int depth = 0;

    // Used by the ARA*-style search.
    bool closed = false;
    bool in_open = false;
    bool in_incons = false;
    std::uint64_t open_stamp = 0;
    bool expansion_counted = false;
    std::shared_ptr<JointMoveGeneratorState> successor_generator;
  };

  struct QueueEntry {
    double f = 0.0;
    double h = 0.0;
    std::uint64_t serial = 0;
    std::uint64_t open_stamp = 0;
    int node_index = -1;

    bool operator<(const QueueEntry& other) const {
      if (f != other.f) return f > other.f;
      if (h != other.h) return h > other.h;
      return serial > other.serial;
    }
  };

  struct SearchAttempt {
    bool success = false;
    double cost = std::numeric_limits<double>::infinity();
    std::vector<std::vector<State>> states;
    std::vector<std::vector<PrimitiveId>> primitives;
  };

  // Declared first so construction time also includes PrimitiveTable and CollisionChecker construction.
  std::chrono::steady_clock::time_point construction_start_;
  Problem problem_;
  PrimitiveTable primitives_;
  CollisionChecker collision_checker_;
  TransitionProvider transitions_;
  CandidateProvider candidates_;
  std::mt19937 random_;
  SearchInstrumentation instrumentation_;

  double primitive_collision_precompute_ms_ = 0.0;
  double static_precompute_ms_ = 0.0;
  double query_precompute_ms_ = 0.0;

  [[nodiscard]] SearchAttempt weighted_search(
      double weight,
      double incumbent_cost,
      Deadline& deadline,
      std::uint64_t& expanded_nodes);

  void solve_repeated_weighted(
      Deadline& deadline,
      Solution& solution,
      std::uint64_t& expanded_nodes);

  void solve_ara_star(
      Deadline& deadline,
      Solution& solution,
      std::uint64_t& expanded_nodes);

  void publish_solution(
      const SearchAttempt& attempt,
      double weight,
      Deadline& deadline,
      Solution& solution) const;

  void publish_solution_from_node(
      const std::vector<SearchNode>& nodes,
      int goal_index,
      double weight,
      Deadline& deadline,
      Solution& solution) const;

  [[nodiscard]] std::vector<int> priority_order(
      const std::vector<State>& configuration) const;
  [[nodiscard]] double heuristic(
      const std::vector<State>& configuration) const;
  [[nodiscard]] double single_agent_steps(
      std::size_t agent,
      const State& state) const;
  [[nodiscard]] double edge_cost(
      const std::vector<State>& configuration) const;
  [[nodiscard]] bool is_goal(
      const std::vector<State>& configuration) const;

  [[nodiscard]] std::optional<JointMove> next_joint_move(
      const std::vector<State>& configuration,
      const std::vector<int>& order,
      PIBT& pibt,
      Deadline& deadline,
      JointMoveGeneratorState& generator);

  [[nodiscard]] bool generator_has_more(
      const JointMoveGeneratorState& generator) const;

  [[nodiscard]] SearchAttempt reconstruct(
      const std::vector<SearchNode>& nodes,
      int goal_index) const;
};

}  // namespace lacam_primitive
