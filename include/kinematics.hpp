#pragma once

#include <limits>

#include "geometry.hpp"

namespace lacam_primitive {

struct Twist2D {
  double vx = 0.0;       // m/s
  double vy = 0.0;       // m/s
  double omega = 0.0;    // rad/s
};

// Closed scalar interval. An interval with lower > upper is empty. Keeping
// this as a value type makes it suitable for compact reachable-set labels in
// either LaCAM/PIBT or CBS/A*.
struct ScalarInterval {
  double lower = 0.0;
  double upper = -1.0;

  [[nodiscard]] bool empty() const noexcept { return lower > upper; }
  [[nodiscard]] bool contains(double value, double tolerance = 1e-9) const noexcept {
    return !empty() && value >= lower - tolerance && value <= upper + tolerance;
  }
};

[[nodiscard]] ScalarInterval intersect(
    const ScalarInterval& left,
    const ScalarInterval& right) noexcept;

// Exact boundary-velocity relation for one-dimensional constant-acceleration
// motion over a fixed displacement and duration:
//
//   displacement = (start_velocity + end_velocity) * duration / 2
//
// The intervals intentionally leave the boundary velocities undecided. A
// lower-level continuous trajectory solver can select any consistent value.
struct ConstantAccelerationEnvelope {
  double displacement = 0.0;
  double duration = 0.0;
  double max_abs_velocity = 0.0;
  double max_abs_acceleration = std::numeric_limits<double>::infinity();
  ScalarInterval start_velocity;
  ScalarInterval end_velocity;

  [[nodiscard]] bool feasible() const noexcept {
    return !start_velocity.empty() && !end_velocity.empty();
  }

  [[nodiscard]] double end_velocity_for(double start) const;
};

[[nodiscard]] ConstantAccelerationEnvelope constant_acceleration_envelope(
    double displacement,
    double duration,
    double max_abs_velocity,
    double max_abs_acceleration = std::numeric_limits<double>::infinity());

// Further restrict a rotational envelope so every point of a rectangle
// rotating about a fixed body-frame pivot stays below a physical acceleration
// limit. The result remains an interval because the worst acceleration is
// monotone in the distance of the start rate from the average rate.
[[nodiscard]] ConstantAccelerationEnvelope
apply_fixed_pivot_body_acceleration_limit(
    ConstantAccelerationEnvelope envelope,
    double pivot_offset_m,
    double length,
    double width,
    double max_body_point_acceleration);

// Propagate an incoming boundary-velocity set through an envelope. For the
// constant-acceleration model this affine interval propagation is exact.
[[nodiscard]] ScalarInterval propagate(
    const ConstantAccelerationEnvelope& envelope,
    const ScalarInterval& incoming_start_velocity) noexcept;

// Project the feasible (start_rate, end_rate) polygon of a cubic Hermite
// progress law onto its end-rate axis. Acceleration is linear within the step,
// so checking both endpoints exactly enforces its absolute bound. Rates are
// non-negative to preserve the primitive's monotone swept-path assumption.
[[nodiscard]] ScalarInterval propagate_cubic_boundary_rates(
    double displacement,
    double duration,
    double max_abs_velocity,
    double max_abs_acceleration,
    const ScalarInterval& incoming_start_velocity);

// Acceleration of a point fixed to a planar rigid body:
//   a_point = a_center + alpha J R r - omega^2 R r
[[nodiscard]] Point2 body_point_acceleration(
    const Point2& center_acceleration,
    double yaw,
    double angular_velocity,
    double angular_acceleration,
    const Point2& body_point);

// Exact maximum over the four physical rectangle corners at one instant.
// collision_padding is deliberately not part of the physical body.
[[nodiscard]] double max_rectangle_corner_acceleration(
    const Point2& center_acceleration,
    double yaw,
    double angular_velocity,
    double angular_acceleration,
    double length,
    double width);

// Fast conservative bound over every point in the rectangle.
[[nodiscard]] double conservative_rectangle_acceleration_bound(
    const Point2& center_acceleration,
    double angular_velocity,
    double angular_acceleration,
    double length,
    double width);

}  // namespace lacam_primitive
