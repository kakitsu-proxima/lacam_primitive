#pragma once

#include <string>

#include "types.hpp"

namespace lacam_primitive {

Problem load_problem(const std::string& path);
void write_solution(const std::string& path, const Problem& problem, const Solution& solution);

}  // namespace lacam_primitive
