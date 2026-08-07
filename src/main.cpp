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
      << "       [--ara-star on|off]\n";
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
        << "warm request: " << solution.stats.warm_request_ms << " ms\n"
        << "cold total: " << solution.stats.cold_total_ms << " ms\n"
        << "modes: transition_cache="
        << on_off(solution.stats.transition_cache_enabled)
        << " candidate_cache="
        << on_off(solution.stats.candidate_cache_enabled)
        << " ara_star=" << on_off(solution.stats.ara_star_enabled)
        << "\n"
        << "cache stats: transition "
        << solution.stats.transition_cache_hits << "/"
        << solution.stats.transition_lookups
        << " hits, candidate "
        << solution.stats.candidate_cache_hits << "/"
        << solution.stats.candidate_lookups
        << " hits, expanded_nodes="
        << solution.stats.expanded_nodes << "\n";

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
