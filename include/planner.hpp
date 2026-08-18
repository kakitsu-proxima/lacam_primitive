#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
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

 private:
  const Problem& problem_;
  const PrimitiveTable& primitives_;
  std::vector<ConvexPolygon> obstacle_polygons_;
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
};

class PIBT {
 public:
  PIBT(
      const Problem& problem,
      const PrimitiveTable& primitives,
      const CollisionChecker& collision_checker,
      const TransitionProvider& transitions,
      const CandidateProvider& candidates,
      std::mt19937& random);

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

  [[nodiscard]] std::vector<std::pair<std::vector<PrimitiveId>, std::vector<State>>>
  generate_joint_moves(
      const std::vector<State>& configuration,
      const std::vector<int>& order,
      PIBT& pibt,
      Deadline& deadline);

  [[nodiscard]] SearchAttempt reconstruct(
      const std::vector<SearchNode>& nodes,
      int goal_index) const;
};

}  // namespace lacam_primitive
