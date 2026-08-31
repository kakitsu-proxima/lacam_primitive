#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include "io.hpp"
#include "planner.hpp"

namespace {

void print_usage(const char* executable) {
  std::cerr
      << "Usage: " << executable
      << " --input problem.yaml --output solution.yaml\n"
      << "       [--time-limit-ms 100]\n"
      << "       [--transition-cache on|off]\n"
      << "       [--candidate-cache on|off]\n"
      << "       [--ara-star on|off]\n"
      << "       [--reverse-bfs on|off]\n"
      << "       [--candidate-diversification on|off]\n"
      << "       [--aabb-broadphase on|off]\n"
      << "       [--conflict-cache on|off]\n"
      << "       [--lazy-successors on|off]\n"
      << "       [--progressive-widening on|off]\n"
      << "       [--initial-candidate-width 1]\n"
      << "       [--per-primitive-intervals on|off]\n"
      << "       [--multiple-rotation-amounts on|off]\n"
      << "       [--acceleration-constraints on|off]\n"
      << "       [--collision-mode time_indexed|whole_step]\n"
      << "       [--max-boundary-travel-per-interval-m 0.005]\n";
}

bool parse_on_off(const std::string& value, const std::string& option_name) {
  if (value == "on" || value == "true" || value == "1") return true;
  if (value == "off" || value == "false" || value == "0") return false;
  throw std::invalid_argument(
      option_name + " expects on/off, true/false, or 1/0");
}

const char* on_off(bool value) {
  return value ? "on" : "off";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string input;
    std::string output;
    double override_time_limit = -1.0;
    std::optional<bool> override_transition_cache;
    std::optional<bool> override_candidate_cache;
    std::optional<bool> override_ara_star;
    std::optional<bool> override_reverse_bfs;
    std::optional<bool> override_candidate_diversification;
    std::optional<bool> override_aabb_broadphase;
    std::optional<bool> override_conflict_cache;
    std::optional<bool> override_lazy_successors;
    std::optional<bool> override_progressive_widening;
    std::optional<std::size_t> override_initial_candidate_width;
    std::optional<bool> override_per_primitive_intervals;
    std::optional<bool> override_multiple_rotation_amounts;
    std::optional<bool> override_acceleration_constraints;
    std::optional<std::string> override_collision_mode;
    std::optional<double> override_max_boundary_travel;

    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--input" && i + 1 < argc) {
        input = argv[++i];
      } else if (argument == "--output" && i + 1 < argc) {
        output = argv[++i];
      } else if (argument == "--time-limit-ms" && i + 1 < argc) {
        override_time_limit = std::stod(argv[++i]);
      } else if (argument == "--transition-cache" && i + 1 < argc) {
        override_transition_cache =
            parse_on_off(argv[++i], argument);
      } else if (argument == "--candidate-cache" && i + 1 < argc) {
        override_candidate_cache =
            parse_on_off(argv[++i], argument);
      } else if (argument == "--ara-star" && i + 1 < argc) {
        override_ara_star = parse_on_off(argv[++i], argument);
      } else if (argument == "--reverse-bfs" && i + 1 < argc) {
        override_reverse_bfs = parse_on_off(argv[++i], argument);
      } else if (argument == "--candidate-diversification" && i + 1 < argc) {
        override_candidate_diversification = parse_on_off(argv[++i], argument);
      } else if (argument == "--aabb-broadphase" && i + 1 < argc) {
        override_aabb_broadphase = parse_on_off(argv[++i], argument);
      } else if (argument == "--conflict-cache" && i + 1 < argc) {
        override_conflict_cache = parse_on_off(argv[++i], argument);
      } else if (argument == "--lazy-successors" && i + 1 < argc) {
        override_lazy_successors = parse_on_off(argv[++i], argument);
      } else if (argument == "--progressive-widening" && i + 1 < argc) {
        override_progressive_widening = parse_on_off(argv[++i], argument);
      } else if (argument == "--initial-candidate-width" && i + 1 < argc) {
        override_initial_candidate_width =
            static_cast<std::size_t>(std::stoull(argv[++i]));
      } else if (argument == "--per-primitive-intervals" && i + 1 < argc) {
        override_per_primitive_intervals = parse_on_off(argv[++i], argument);
      } else if (argument == "--multiple-rotation-amounts" && i + 1 < argc) {
        override_multiple_rotation_amounts = parse_on_off(argv[++i], argument);
      } else if (argument == "--acceleration-constraints" && i + 1 < argc) {
        override_acceleration_constraints = parse_on_off(argv[++i], argument);
      } else if (argument == "--collision-mode" && i + 1 < argc) {
        override_collision_mode = argv[++i];
      } else if (argument ==
                     "--max-boundary-travel-per-interval-m" &&
                 i + 1 < argc) {
        override_max_boundary_travel = std::stod(argv[++i]);
      } else if (argument == "--help") {
        print_usage(argv[0]);
        return 0;
      } else {
        print_usage(argv[0]);
        return 2;
      }
    }

    if (input.empty() || output.empty()) {
      print_usage(argv[0]);
      return 2;
    }

    lacam_primitive::Problem problem = lacam_primitive::load_problem(input);
    if (override_time_limit > 0.0) {
      problem.search.time_limit_ms = override_time_limit;
    }
    if (override_transition_cache.has_value()) {
      problem.search.use_transition_cache = *override_transition_cache;
    }
    if (override_candidate_cache.has_value()) {
      problem.search.use_candidate_cache = *override_candidate_cache;
    }
    if (override_ara_star.has_value()) {
      problem.search.use_ara_star = *override_ara_star;
    }
    if (override_reverse_bfs.has_value()) {
      problem.search.use_reverse_bfs_heuristic = *override_reverse_bfs;
    }
    if (override_candidate_diversification.has_value()) {
      problem.search.diversify_candidates =
          *override_candidate_diversification;
    }
    if (override_aabb_broadphase.has_value()) {
      problem.search.use_aabb_broadphase = *override_aabb_broadphase;
    }
    if (override_conflict_cache.has_value()) {
      problem.search.use_conflict_cache = *override_conflict_cache;
    }
    if (override_lazy_successors.has_value()) {
      problem.search.use_lazy_successors = *override_lazy_successors;
    }
    if (override_progressive_widening.has_value()) {
      problem.search.use_progressive_widening =
          *override_progressive_widening;
    }
    if (override_initial_candidate_width.has_value()) {
      if (*override_initial_candidate_width == 0) {
        throw std::invalid_argument(
            "--initial-candidate-width must be positive");
      }
      problem.search.initial_candidate_width =
          *override_initial_candidate_width;
    }
    if (override_per_primitive_intervals.has_value()) {
      problem.search.use_per_primitive_intervals =
          *override_per_primitive_intervals;
    }
    if (override_multiple_rotation_amounts.has_value()) {
      problem.primitive_config.use_multiple_rotation_amounts =
          *override_multiple_rotation_amounts;
    }
    if (override_acceleration_constraints.has_value()) {
      problem.primitive_config.use_acceleration_constraints =
          *override_acceleration_constraints;
    }
    if (override_collision_mode.has_value()) {
      if (*override_collision_mode != "time_indexed" &&
          *override_collision_mode != "whole_step") {
        throw std::invalid_argument(
            "--collision-mode expects time_indexed or whole_step");
      }
      problem.search.collision_mode = *override_collision_mode;
    }
    if (override_max_boundary_travel.has_value()) {
      if (*override_max_boundary_travel <= 0.0) {
        throw std::invalid_argument(
            "--max-boundary-travel-per-interval-m must be positive");
      }
      problem.search.max_boundary_travel_per_interval_m =
          *override_max_boundary_travel;
    }

    lacam_primitive::Planner planner(problem);
    const lacam_primitive::Solution solution = planner.solve();
    lacam_primitive::write_solution(output, problem, solution);

    std::cout
        << "precompute static: " << solution.stats.static_precompute_ms
        << " ms"
        << " (primitive/collision="
        << solution.stats.primitive_collision_precompute_ms
        << ", transition_cache="
        << solution.stats.transition_cache_precompute_ms << ")\n"
        << "precompute query: " << solution.stats.query_precompute_ms
        << " ms"
        << " (candidate_cache="
        << solution.stats.candidate_cache_precompute_ms << ")\n"
        << "search: " << solution.stats.search_ms << " ms\n"
        << "first solution: "
        << (solution.success ? solution.stats.first_solution_ms : -1.0)
        << " ms, cost="
        << (solution.success ? solution.stats.first_solution_cost : -1.0)
        << "\n"
        << "warm request: " << solution.stats.warm_request_ms << " ms\n"
        << "cold total: " << solution.stats.cold_total_ms << " ms\n"
        << "modes: transition_cache="
        << on_off(solution.stats.transition_cache_enabled)
        << " candidate_cache="
        << on_off(solution.stats.candidate_cache_enabled)
        << " ara_star=" << on_off(solution.stats.ara_star_enabled)
        << " reverse_bfs="
        << on_off(solution.stats.reverse_bfs_heuristic_enabled)
        << " diversification="
        << on_off(solution.stats.candidate_diversification_enabled)
        << " aabb=" << on_off(solution.stats.aabb_broadphase_enabled)
        << " conflict_cache=" << on_off(solution.stats.conflict_cache_enabled)
        << " lazy=" << on_off(solution.stats.lazy_successors_enabled)
        << " widening="
        << on_off(solution.stats.progressive_widening_enabled)
        << " initial_width=" << solution.stats.initial_candidate_width
        << " per_primitive_intervals="
        << on_off(solution.stats.per_primitive_intervals_enabled)
        << " multiple_rotations="
        << on_off(solution.stats.multiple_rotation_amounts_enabled)
        << " acceleration_constraints="
        << on_off(solution.stats.acceleration_constraints_enabled)
        << " pivot_anchor_lattice="
        << on_off(solution.stats.pivot_anchor_lattice_enabled)
        << " active_rotation_amounts="
        << solution.stats.active_rotation_amount_count
        << " collision_mode=" << solution.stats.collision_mode
        << " max_boundary_travel_per_interval_m="
        << solution.stats.max_boundary_travel_per_interval_m
        << " collision_intervals="
        << solution.stats.collision_interval_count
        << " interval_polygons="
        << solution.stats.collision_interval_polygon_count
        << "\n"
        << "cache stats: transition "
        << solution.stats.transition_cache_hits << "/"
        << solution.stats.transition_lookups
        << " hits, candidate "
        << solution.stats.candidate_cache_hits << "/"
        << solution.stats.candidate_lookups
        << " hits, expanded_nodes="
        << solution.stats.expanded_nodes
        << ", low_level_nodes=" << solution.stats.low_level_constraint_nodes
        << ", kinematic_validations="
        << solution.stats.kinematic_validation_calls
        << ", kinematic_failures="
        << solution.stats.kinematic_validation_failures
        << ", kinematic_restarts="
        << solution.stats.kinematic_search_restarts
        << ", kinematic_no_goods="
        << solution.stats.kinematic_no_good_count
        << ", pibt_backtracks=" << solution.stats.pibt_backtracks
        << ", continuations=" << solution.stats.successor_continuations
        << ", widening_stages="
        << solution.stats.progressive_widening_stages
        << ", max_open=" << solution.stats.max_open_size << "\n"
        << "collision stats: calls=" << solution.stats.conflict_calls
        << ", cache_hits=" << solution.stats.conflict_cache_hits
        << ", cache_entries=" << solution.stats.conflict_cache_entries
        << ", whole_aabb_rejects=" << solution.stats.whole_step_aabb_rejects
        << "/" << solution.stats.whole_step_aabb_tests
        << ", interval_aabb_rejects=" << solution.stats.interval_aabb_rejects
        << "/" << solution.stats.interval_aabb_tests
        << ", SAT=" << solution.stats.polygon_sat_tests << "\n";

    if (!solution.success) {
      std::cerr << "No solution found within "
                << solution.stats.search_ms << " search ms\n";
      return 1;
    }

    std::cout << "Final solution cost: " << solution.cost
              << ", search: " << solution.stats.search_ms << " ms\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 2;
  }
}
