#pragma once

#include <string>
#include <vector>

#include "geometry.hpp"
#include "types.hpp"

namespace lacam_primitive {

struct Primitive {
  PrimitiveId id = 0;
  std::string name;

  // Global translation primitives use these directly.
  int dx = 0;
  int dy = 0;

  int d_heading = 0;

  // True for rotation around a point on the robot longitudinal axis.
  bool pivot_rotation = false;

  // Body-frame longitudinal offset from the rectangle center.
  // Positive = front (+body-x), negative = rear.
  int pivot_offset_cells = 0;
};

struct PrimitiveVariant {
  // Important:
  // for pivot rotations, delta.x / delta.y depend on start heading.
  State delta;

  // A conservative metric swept region for each common time interval.
  // Coordinates are relative to the primitive's integer start anchor and are
  // measured in grid cells; the construction tolerance itself is specified in
  // metres so its physical accuracy is independent of planning cell_size.
  std::vector<ConvexPolygon> interval_polygons;

  // Convex hull of every interval polygon. Comparing this once per pair is
  // faster, but deliberately ignores when within the step an area is used.
  ConvexPolygon whole_step_polygon;
};

class PrimitiveTable {
 public:
  explicit PrimitiveTable(const Problem& problem);

  [[nodiscard]] const Primitive& primitive(PrimitiveId id) const;

  [[nodiscard]] const PrimitiveVariant& variant(
      PrimitiveId id,
      int start_heading) const;

  [[nodiscard]] State apply(
      const State& state,
      PrimitiveId id) const;

  [[nodiscard]] const std::vector<Primitive>& primitives() const {
    return primitives_;
  }

  [[nodiscard]] PrimitiveId wait_id() const {
    return wait_id_;
  }

  [[nodiscard]] int max_translation_cells() const {
    return max_translation_cells_;
  }

  // Maximum Manhattan position change produced by one primitive.
  // Includes off-center rotations.
  [[nodiscard]] int max_position_delta_cells() const {
    return max_position_delta_cells_;
  }

  [[nodiscard]] int max_rotation_bins() const {
    return max_rotation_bins_;
  }

  [[nodiscard]] bool has_coupled_rotation_translation() const {
    return has_coupled_rotation_translation_;
  }

  [[nodiscard]] std::size_t interval_count() const {
    return interval_count_;
  }

 private:
  Problem problem_;

  std::vector<Primitive> primitives_;
  std::vector<std::vector<PrimitiveVariant>> variants_;

  PrimitiveId wait_id_ = 0;

  int max_translation_cells_ = 1;
  int max_position_delta_cells_ = 1;
  int max_rotation_bins_ = 1;

  bool has_coupled_rotation_translation_ = false;

  std::size_t interval_count_ = 1;

  void generate_primitives();
  void build_variants();
};

}  // namespace lacam_primitive
