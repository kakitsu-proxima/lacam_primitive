#pragma once

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

  [[nodiscard]] bool statically_valid(const State& start, PrimitiveId primitive_id) const;
  [[nodiscard]] bool conflict(
      const State& left_start,
      PrimitiveId left_primitive,
      const State& right_start,
      PrimitiveId right_primitive) const;

 private:
  const Problem& problem_;
  const PrimitiveTable& primitives_;
  StaticGrid static_grid_;
};

class PIBT {
 public:
  PIBT(
      const Problem& problem,
      const PrimitiveTable& primitives,
      const CollisionChecker& collision_checker,
      std::mt19937& random);

  [[nodiscard]] std::vector<PrimitiveId> ordered_candidates(
      std::size_t agent,
      const State& state) const;

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
  std::mt19937& random_;

  bool assign_agent(
      int agent,
      const std::vector<State>& current,
      const std::vector<std::optional<PrimitiveId>>& forced,
      std::vector<PrimitiveId>& selected,
      std::vector<State>& next,
      std::vector<bool>& assigned,
      std::vector<bool>& visiting);

  [[nodiscard]] double agent_heuristic(std::size_t agent, const State& state) const;
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
    double g = 0.0;
    double h = 0.0;
    int depth = 0;
  };

  struct QueueEntry {
    double f = 0.0;
    double h = 0.0;
    std::uint64_t serial = 0;
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

  Problem problem_;
  PrimitiveTable primitives_;
  CollisionChecker collision_checker_;
  std::mt19937 random_;

  [[nodiscard]] SearchAttempt weighted_search(
      double weight,
      double incumbent_cost,
      Deadline& deadline);

  [[nodiscard]] std::vector<int> priority_order(const std::vector<State>& configuration) const;
  [[nodiscard]] double heuristic(const std::vector<State>& configuration) const;
  [[nodiscard]] double single_agent_steps(std::size_t agent, const State& state) const;
  [[nodiscard]] double edge_cost(const std::vector<State>& configuration) const;
  [[nodiscard]] bool is_goal(const std::vector<State>& configuration) const;

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
