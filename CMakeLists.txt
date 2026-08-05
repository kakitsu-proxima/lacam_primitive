cmake_minimum_required(VERSION 3.20)
project(minilacam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(lacam_primitive_core
  src/yaml.cpp
  src/geometry.cpp
  src/primitives.cpp
  src/planner.cpp
  src/io.cpp
)

target_include_directories(lacam_primitive_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(lacam_primitive_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(lacam_primitive src/main.cpp)
target_link_libraries(lacam_primitive PRIVATE lacam_primitive_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(lacam_primitive_tests tests/test_main.cpp)
  target_link_libraries(lacam_primitive_tests PRIVATE lacam_primitive_core)
  add_test(NAME lacam_primitive_tests COMMAND lacam_primitive_tests)
endif()
