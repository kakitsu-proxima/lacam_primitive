cmake_minimum_required(VERSION 3.20)
project(lacam_primitive VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(
  LACAM_PRIMITIVE_DEFAULT_TRANSITION_CACHE
  "Enable the static transition cache by default"
  ON
)
option(
  LACAM_PRIMITIVE_DEFAULT_CANDIDATE_CACHE
  "Enable the candidate ordering cache by default"
  ON
)
option(
  LACAM_PRIMITIVE_DEFAULT_ARA_STAR
  "Use ARA*-style anytime search by default"
  ON
)

add_library(lacam_primitive_core
  src/yaml.cpp
  src/geometry.cpp
  src/primitives.cpp
  src/planner.cpp
  src/io.cpp
)

target_include_directories(
  lacam_primitive_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(
  lacam_primitive_core
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_compile_definitions(
  lacam_primitive_core
  PUBLIC
    LACAM_PRIMITIVE_DEFAULT_TRANSITION_CACHE=$<BOOL:${LACAM_PRIMITIVE_DEFAULT_TRANSITION_CACHE}>
    LACAM_PRIMITIVE_DEFAULT_CANDIDATE_CACHE=$<BOOL:${LACAM_PRIMITIVE_DEFAULT_CANDIDATE_CACHE}>
    LACAM_PRIMITIVE_DEFAULT_ARA_STAR=$<BOOL:${LACAM_PRIMITIVE_DEFAULT_ARA_STAR}>
)

add_executable(lacam_primitive src/main.cpp)
target_link_libraries(lacam_primitive PRIVATE lacam_primitive_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(lacam_primitive_tests tests/test_main.cpp)
  target_link_libraries(lacam_primitive_tests PRIVATE lacam_primitive_core)
  target_compile_definitions(
    lacam_primitive_tests
    PRIVATE LACAM_PRIMITIVE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(NAME lacam_primitive_tests COMMAND lacam_primitive_tests)
endif()
