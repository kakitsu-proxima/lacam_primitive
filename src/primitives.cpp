#include "primitives.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

namespace lacam_primitive {
namespace {

std::string pivot_suffix(int offset) {
  if (offset == 0) {
    return "center";
  }
  if (offset > 0) {
    return "front_" + std::to_string(offset);
  }
  return "rear_" + std::to_string(-offset);
}

bool nearly_integer(double value) {
  return std::abs(value - std::round(value)) < 1e-8;
}

}  // namespace

PrimitiveTable::PrimitiveTable(const Problem& problem)
    : problem_(problem) {
  generate_primitives();
  build_variants();
}

const Primitive& PrimitiveTable::primitive(PrimitiveId id) const {
  if (static_cast<std::size_t>(id) >= primitives_.size()) {
    throw std::out_of_range("primitive id");
  }
  return primitives_[id];
}

const PrimitiveVariant& PrimitiveTable::variant(
    PrimitiveId id,
    int start_heading) const {
  if (static_cast<std::size_t>(id) >= variants_.size()) {
    throw std::out_of_range("primitive id");
  }

  const int heading =
      positive_mod(start_heading, problem_.grid.heading_bins);

  return variants_[id][static_cast<std::size_t>(heading)];
}

State PrimitiveTable::apply(
    const State& state,
    PrimitiveId id) const {
  const PrimitiveVariant& selected =
      variant(id, state.heading);

  return State{
      state.x + selected.delta.x,
      state.y + selected.delta.y,
      positive_mod(
          state.heading + selected.delta.heading,
          problem_.grid.heading_bins)};
}

void PrimitiveTable::generate_primitives() {
  const GridSpec& grid = problem_.grid;
  const RobotSpec& robot = problem_.robot;

  if (grid.cell_size <= 0.0 ||
      grid.macro_dt <= 0.0 ||
      grid.heading_bins <= 0) {
    throw std::invalid_argument(
        "invalid grid metric or time settings");
  }

  if (robot.max_linear_velocity <= 0.0 ||
      robot.max_angular_velocity <= 0.0) {
    throw std::invalid_argument(
        "velocity limits must be positive");
  }

  std::set<int> distances;

  for (int distance :
       problem_.primitive_config.translation_cells) {
    if (distance <= 0) {
      throw std::invalid_argument(
          "translation_cells must contain positive integers");
    }

    const double velocity =
        static_cast<double>(distance) *
        grid.cell_size /
        grid.macro_dt;

    if (velocity >
        robot.max_linear_velocity + 1e-9) {
      throw std::invalid_argument(
          "translation primitive of " +
          std::to_string(distance) +
          " cells exceeds max_linear_velocity");
    }

    distances.insert(distance);

    max_translation_cells_ =
        std::max(max_translation_cells_, distance);

    max_position_delta_cells_ =
        std::max(max_position_delta_cells_, distance);
  }

  const int rotation_bins =
      problem_.primitive_config.rotation_bins;

  if (rotation_bins <= 0) {
    throw std::invalid_argument(
        "rotation_bins must be positive");
  }

  const double angular_velocity =
      static_cast<double>(rotation_bins) *
      grid.heading_step() /
      grid.macro_dt;

  if (angular_velocity >
      robot.max_angular_velocity + 1e-9) {
    throw std::invalid_argument(
        "rotation primitive exceeds max_angular_velocity");
  }

  max_rotation_bins_ = rotation_bins;

  std::set<int> pivot_offsets(
      problem_.primitive_config
          .rotation_pivot_offsets_cells.begin(),
      problem_.primitive_config
          .rotation_pivot_offsets_cells.end());

  if (pivot_offsets.empty()) {
    pivot_offsets.insert(0);
  }

  // The center of the rectangle travels around the selected pivot.
  // Check its tangential velocity as a conservative interpretation of
  // max_linear_velocity.
  for (int offset : pivot_offsets) {
    const double center_linear_velocity =
        std::abs(static_cast<double>(offset)) *
        grid.cell_size *
        angular_velocity;

    if (center_linear_velocity >
        robot.max_linear_velocity + 1e-9) {
      throw std::invalid_argument(
          "pivot rotation with offset " +
          std::to_string(offset) +
          " cells exceeds max_linear_velocity");
    }
  }

  auto add =
      [&](std::string name,
          int dx,
          int dy,
          int dh,
          bool pivot_rotation,
          int pivot_offset_cells) {
        if (primitives_.size() >=
            std::numeric_limits<PrimitiveId>::max()) {
          throw std::runtime_error(
              "too many primitives");
        }

        const PrimitiveId id =
            static_cast<PrimitiveId>(
                primitives_.size());

        primitives_.push_back(
            Primitive{
                id,
                std::move(name),
                dx,
                dy,
                dh,
                pivot_rotation,
                pivot_offset_cells});

        return id;
      };

  if (problem_.primitive_config.include_wait) {
    wait_id_ =
        add(
            "wait",
            0,
            0,
            0,
            false,
            0);
  } else {
    throw std::invalid_argument(
        "the minimal PIBT implementation "
        "requires a wait primitive");
  }

  for (int distance : distances) {
    add(
        "east_" + std::to_string(distance),
        distance,
        0,
        0,
        false,
        0);

    add(
        "west_" + std::to_string(distance),
        -distance,
        0,
        0,
        false,
        0);

    add(
        "north_" + std::to_string(distance),
        0,
        distance,
        0,
        false,
        0);

    add(
        "south_" + std::to_string(distance),
        0,
        -distance,
        0,
        false,
        0);
  }

  for (int pivot_offset : pivot_offsets) {
    const std::string suffix =
        pivot_suffix(pivot_offset);

    add(
        "rotate_ccw_" + suffix,
        0,
        0,
        rotation_bins,
        true,
        pivot_offset);

    add(
        "rotate_cw_" + suffix,
        0,
        0,
        -rotation_bins,
        true,
        pivot_offset);
  }
}

void PrimitiveTable::build_variants() {
  const GridSpec& grid = problem_.grid;
  const RobotSpec& robot = problem_.robot;

  const double radius_cells =
      0.5 *
      std::hypot(robot.length, robot.width) /
      grid.cell_size;

  double maximum_boundary_travel_cells = 0.0;

  for (const Primitive& primitive : primitives_) {
    if (primitive.pivot_rotation) {
      const double rotation =
          std::abs(
              static_cast<double>(
                  primitive.d_heading)) *
          grid.heading_step();

      // Every point of the robot can move at most approximately
      // (pivot distance + robot radius) * angle.
      const double pivot_radius =
          std::abs(
              static_cast<double>(
                  primitive.pivot_offset_cells));

      maximum_boundary_travel_cells =
          std::max(
              maximum_boundary_travel_cells,
              (pivot_radius + radius_cells) *
                  rotation);
    } else {
      const double translation =
          std::hypot(
              static_cast<double>(primitive.dx),
              static_cast<double>(primitive.dy));

      const double rotation =
          std::abs(
              static_cast<double>(
                  primitive.d_heading)) *
          grid.heading_step();

      maximum_boundary_travel_cells =
          std::max(
              maximum_boundary_travel_cells,
              translation +
                  radius_cells * rotation);
    }
  }

  constexpr double
      maximum_boundary_travel_per_interval = 0.25;

  interval_count_ =
      std::max<std::size_t>(
          1,
          static_cast<std::size_t>(
              std::ceil(
                  maximum_boundary_travel_cells /
                  maximum_boundary_travel_per_interval)));

  interval_count_ =
      std::min<std::size_t>(
          interval_count_,
          256);

  const double actual_guard_cells =
      0.5 *
      maximum_boundary_travel_cells /
      static_cast<double>(interval_count_);

  const double half_length_cells =
      0.5 *
          robot.length /
          grid.cell_size +
      robot.collision_padding /
          grid.cell_size +
      actual_guard_cells;

  const double half_width_cells =
      0.5 *
          robot.width /
          grid.cell_size +
      robot.collision_padding /
          grid.cell_size +
      actual_guard_cells;

  variants_.resize(primitives_.size());

  for (const Primitive& primitive :
       primitives_) {
    variants_[primitive.id].resize(
        static_cast<std::size_t>(
            grid.heading_bins));

    for (int heading = 0;
         heading < grid.heading_bins;
         ++heading) {
      PrimitiveVariant variant;

      variant.interval_masks.reserve(
          interval_count_);

      const double start_yaw =
          static_cast<double>(heading) *
          grid.heading_step();

      const double yaw_delta =
          static_cast<double>(
              primitive.d_heading) *
          grid.heading_step();

      double final_center_x =
          static_cast<double>(primitive.dx);

      double final_center_y =
          static_cast<double>(primitive.dy);

      if (primitive.pivot_rotation) {
        const double pivot_offset =
            static_cast<double>(
                primitive.pivot_offset_cells);

        const double pivot_x =
            pivot_offset *
            std::cos(start_yaw);

        const double pivot_y =
            pivot_offset *
            std::sin(start_yaw);

        const double end_yaw =
            start_yaw + yaw_delta;

        final_center_x =
            pivot_x -
            pivot_offset *
                std::cos(end_yaw);

        final_center_y =
            pivot_y -
            pivot_offset *
                std::sin(end_yaw);

        // State only supports integer grid coordinates.
        if (!nearly_integer(final_center_x) ||
            !nearly_integer(final_center_y)) {
          throw std::invalid_argument(
              "pivot rotation produces a non-integer "
              "grid endpoint. Use a finer cell_size or "
              "an integer-compatible pivot offset.");
        }
      }

      variant.delta =
          State{
              static_cast<int>(
                  std::llround(
                      final_center_x)),
              static_cast<int>(
                  std::llround(
                      final_center_y)),
              primitive.d_heading};

      if (primitive.d_heading != 0 &&
          (variant.delta.x != 0 ||
           variant.delta.y != 0)) {
        has_coupled_rotation_translation_ =
            true;
      }

      max_position_delta_cells_ =
          std::max(
              max_position_delta_cells_,
              std::abs(variant.delta.x) +
                  std::abs(variant.delta.y));

      for (std::size_t interval = 0;
           interval < interval_count_;
           ++interval) {
        const double alpha0 =
            static_cast<double>(interval) /
            static_cast<double>(
                interval_count_);

        const double alpha1 =
            static_cast<double>(interval + 1) /
            static_cast<double>(
                interval_count_);

        const double alpha_mid =
            0.5 * (alpha0 + alpha1);

        std::vector<CellMask> samples;
        samples.reserve(3);

        for (double alpha :
             {alpha0, alpha_mid, alpha1}) {
          const double yaw =
              start_yaw +
              alpha * yaw_delta;

          double center_x = 0.0;
          double center_y = 0.0;

          if (primitive.pivot_rotation) {
            const double pivot_offset =
                static_cast<double>(
                    primitive.pivot_offset_cells);

            // Fixed pivot in world coordinates.
            const double pivot_x =
                pivot_offset *
                std::cos(start_yaw);

            const double pivot_y =
                pivot_offset *
                std::sin(start_yaw);

            // Rectangle center follows a circular arc
            // around the fixed pivot.
            center_x =
                pivot_x -
                pivot_offset *
                    std::cos(yaw);

            center_y =
                pivot_y -
                pivot_offset *
                    std::sin(yaw);
          } else {
            center_x =
                alpha *
                static_cast<double>(
                    primitive.dx);

            center_y =
                alpha *
                static_cast<double>(
                    primitive.dy);
          }

          samples.push_back(
              rasterize_oriented_rectangle(
                  center_x,
                  center_y,
                  yaw,
                  half_length_cells,
                  half_width_cells));
        }

        variant.interval_masks.push_back(
            union_masks(samples));
      }

      variants_[primitive.id]
               [static_cast<std::size_t>(
                    heading)] =
          std::move(variant);
    }
  }
}

}  // namespace lacam_primitive