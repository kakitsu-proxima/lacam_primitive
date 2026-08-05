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

namespace lacam_primitive {

constexpr double kPi = 3.141592653589793238462643383279502884;

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
    seed ^= static_cast<std::size_t>(state.y) + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
    seed ^= static_cast<std::size_t>(state.heading) + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
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
  double cell_size = 0.1;
  double origin_x = 0.0;
  double origin_y = 0.0;
  int width_cells = 1;
  int height_cells = 1;
  int heading_bins = 4;
  double macro_dt = 0.2;

  [[nodiscard]] double heading_step() const {
    return 2.0 * kPi / static_cast<double>(heading_bins);
  }

  [[nodiscard]] double world_x(int x) const {
    return origin_x + static_cast<double>(x) * cell_size;
  }

  [[nodiscard]] double world_y(int y) const {
    return origin_y + static_cast<double>(y) * cell_size;
  }
};

struct RobotSpec {
  double length = 0.2;
  double width = 0.2;
  double max_linear_velocity = 1.0;
  double max_angular_velocity = 8.0;
  double collision_padding = 0.0;
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
  std::size_t alternatives_per_agent = 5;
  std::uint32_t random_seed = 7;
};

struct PrimitiveConfig {
  std::vector<int> translation_cells{1};
  int rotation_bins = 1;
  bool include_wait = true;
};

struct ObstacleRect {
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
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
  std::vector<Agent> agents;
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
};

struct Solution {
  bool success = false;
  double elapsed_ms = 0.0;
  double cost = std::numeric_limits<double>::infinity();
  std::string objective;
  std::vector<AgentPlan> plans;
  std::vector<Improvement> improvements;
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
