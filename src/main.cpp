#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "io.hpp"
#include "planner.hpp"

namespace {

void print_usage(const char* executable) {
  std::cerr << "Usage: " << executable
            << " --input problem.yaml --output solution.yaml [--time-limit-ms 100]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string input;
    std::string output;
    double override_time_limit = -1.0;

    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--input" && i + 1 < argc) {
        input = argv[++i];
      } else if (argument == "--output" && i + 1 < argc) {
        output = argv[++i];
      } else if (argument == "--time-limit-ms" && i + 1 < argc) {
        override_time_limit = std::stod(argv[++i]);
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
    if (override_time_limit > 0.0) problem.search.time_limit_ms = override_time_limit;
    lacam_primitive::Planner planner(problem);
    const lacam_primitive::Solution solution = planner.solve();
    lacam_primitive::write_solution(output, problem, solution);

    if (!solution.success) {
      std::cerr << "No solution found within " << solution.elapsed_ms << " ms\n";
      return 1;
    }
    std::cout << "Final solution cost: " << solution.cost
              << ", elapsed: " << solution.elapsed_ms << " ms\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 2;
  }
}
