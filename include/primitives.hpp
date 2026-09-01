#pragma once

#include <string>
#include <vector>

#include "geometry.hpp"
#include "kinematics.hpp"
#include "types.hpp"

namespace lacam_primitive {

enum class ProgressCoordinate {
  kStationary,
  kTranslationMetres,
  kRotationRadians,
};

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

  // Feasible boundary-rate range for a constant-acceleration time law. The
  // planner does not select one exact rate yet; this envelope is intentionally
  // retained for the downstream continuous trajectory layer.
  ProgressCoordinate progress_coordinate = ProgressCoordinate::kStationary;
  ConstantAccelerationEnvelope progress_envelope;

  // Limits used by the sequence-level cubic time-scaling validator. For a
  // fixed-pivot rotation these already include the conservative rigid-body
  // point-acceleration cap.
  double kinematic_max_rate = 0.0;
  double kinematic_max_acceleration =
      std::numeric_limits<double>::infinity();
  CubicBoundaryRateRelation kinematic_relation;
  bool kinematically_feasible = true;
};

struct PrimitiveVariant {
  // Important:
  // for pivot rotations, delta.x / delta.y depend on start heading.
  State delta;

  // Metric boundary twist produced by one unit of the scalar progress rate
  // stored in Primitive::progress_envelope. A continuous layer can choose
  // rates from the envelope and enforce
  // previous.end_basis * rate == next.start_basis * rate without the graph
  // committing to one exact speed.
  Twist2D start_twist_per_progress_rate;
  Twist2D end_twist_per_progress_rate;

  // A conservative metric swept region for each common time interval.
  // Coordinates are relative to the primitive's integer start anchor and are
  // measured in grid cells; the construction tolerance itself is specified in
  // metres so its physical accuracy is independent of planning cell_size.
  std::vector<ConvexPolygon> interval_polygons;
  std::vector<AxisAlignedBounds> interval_bounds;
  std::size_t interval_count = 1;

  // Convex hull of every interval polygon. Comparing this once per pair is
  // faster, but deliberately ignores when within the step an area is used.
  ConvexPolygon whole_step_polygon;
  AxisAlignedBounds whole_step_bounds;
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

  [[nodiscard]] std::size_t active_rotation_amount_count() const {
    return active_rotation_amount_count_;
  }

  [[nodiscard]] bool has_coupled_rotation_translation() const {
    return has_coupled_rotation_translation_;
  }

  [[nodiscard]] bool has_off_center_pivots() const {
    return has_off_center_pivots_;
  }

  [[nodiscard]] std::size_t interval_count() const {
    return interval_count_;
  }

  [[nodiscard]] std::size_t total_variant_intervals() const {
    return total_variant_intervals_;
  }

 private:
  Problem problem_;

  std::vector<Primitive> primitives_;
  std::vector<std::vector<PrimitiveVariant>> variants_;

  PrimitiveId wait_id_ = 0;

  int max_translation_cells_ = 1;
  int max_position_delta_cells_ = 1;
  int max_rotation_bins_ = 1;
  std::size_t active_rotation_amount_count_ = 1;

  bool has_coupled_rotation_translation_ = false;
  bool has_off_center_pivots_ = false;

  std::size_t interval_count_ = 1;
  std::size_t total_variant_intervals_ = 0;

  void generate_primitives();
  void build_variants();
};

}  // namespace lacam_primitive
