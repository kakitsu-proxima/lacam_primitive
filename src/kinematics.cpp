#include "kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace lacam_primitive {

ScalarInterval intersect(
    const ScalarInterval& left,
    const ScalarInterval& right) noexcept {
  return ScalarInterval{
      std::max(left.lower, right.lower),
      std::min(left.upper, right.upper)};
}

double ConstantAccelerationEnvelope::end_velocity_for(double start) const {
  if (duration <= 0.0) {
    throw std::logic_error("constant-acceleration envelope has no duration");
  }
  return 2.0 * displacement / duration - start;
}

ConstantAccelerationEnvelope constant_acceleration_envelope(
    double displacement,
    double duration,
    double max_abs_velocity,
    double max_abs_acceleration) {
  if (!std::isfinite(displacement) || !std::isfinite(duration) ||
      !std::isfinite(max_abs_velocity) || duration <= 0.0 ||
      max_abs_velocity <= 0.0 || std::isnan(max_abs_acceleration) ||
      max_abs_acceleration <= 0.0) {
    throw std::invalid_argument("invalid constant-acceleration limits");
  }

  const double twice_average = 2.0 * displacement / duration;
  ScalarInterval starts{-max_abs_velocity, max_abs_velocity};

  // end = twice_average - start must also satisfy the velocity bound.
  starts = intersect(
      starts,
      ScalarInterval{
          twice_average - max_abs_velocity,
          twice_average + max_abs_velocity});

  if (std::isfinite(max_abs_acceleration)) {
    // acceleration = 2 * (average_velocity - start) / duration.
    const double average = displacement / duration;
    const double half_delta = 0.5 * max_abs_acceleration * duration;
    starts = intersect(
        starts,
        ScalarInterval{average - half_delta, average + half_delta});
  }

  ConstantAccelerationEnvelope result;
  result.displacement = displacement;
  result.duration = duration;
  result.max_abs_velocity = max_abs_velocity;
  result.max_abs_acceleration = max_abs_acceleration;
  result.start_velocity = starts;
  if (starts.empty()) return result;
  result.end_velocity = ScalarInterval{
      twice_average - starts.upper,
      twice_average - starts.lower};
  return result;
}

ConstantAccelerationEnvelope apply_fixed_pivot_body_acceleration_limit(
    ConstantAccelerationEnvelope envelope,
    double pivot_offset_m,
    double length,
    double width,
    double max_body_point_acceleration) {
  if (length <= 0.0 || width <= 0.0 ||
      std::isnan(max_body_point_acceleration) ||
      max_body_point_acceleration <= 0.0) {
    throw std::invalid_argument("invalid rigid-body acceleration limit");
  }
  if (!envelope.feasible() ||
      !std::isfinite(max_body_point_acceleration)) {
    return envelope;
  }

  const double half_length = 0.5 * length;
  const double half_width = 0.5 * width;
  double maximum_radius = 0.0;
  for (double x : {-half_length, half_length}) {
    for (double y : {-half_width, half_width}) {
      maximum_radius = std::max(
          maximum_radius, std::hypot(x - pivot_offset_m, y));
    }
  }
  if (maximum_radius <= 0.0) return envelope;

  const double average = envelope.displacement / envelope.duration;
  const auto maximum_point_acceleration = [&](double start_rate) {
    const double end_rate = envelope.end_velocity_for(start_rate);
    const double angular_acceleration =
        (end_rate - start_rate) / envelope.duration;
    const double maximum_angular_speed =
        std::max(std::abs(start_rate), std::abs(end_rate));
    return maximum_radius * std::hypot(
        angular_acceleration,
        maximum_angular_speed * maximum_angular_speed);
  };

  if (maximum_point_acceleration(average) >
      max_body_point_acceleration) {
    envelope.start_velocity = ScalarInterval{};
    envelope.end_velocity = ScalarInterval{};
    return envelope;
  }

  const double available_half_width = std::max(
      std::abs(envelope.start_velocity.lower - average),
      std::abs(envelope.start_velocity.upper - average));
  double low = 0.0;
  double high = available_half_width;
  if (maximum_point_acceleration(average + high) <=
      max_body_point_acceleration) {
    low = high;
  } else {
    for (int iteration = 0; iteration < 64; ++iteration) {
      const double middle = 0.5 * (low + high);
      if (maximum_point_acceleration(average + middle) <=
          max_body_point_acceleration) {
        low = middle;
      } else {
        high = middle;
      }
    }
  }

  envelope.start_velocity = intersect(
      envelope.start_velocity,
      ScalarInterval{average - low, average + low});
  if (envelope.start_velocity.empty()) {
    envelope.end_velocity = ScalarInterval{};
  } else {
    const double twice_average = 2.0 * average;
    envelope.end_velocity = ScalarInterval{
        twice_average - envelope.start_velocity.upper,
        twice_average - envelope.start_velocity.lower};
  }
  return envelope;
}

ScalarInterval propagate(
    const ConstantAccelerationEnvelope& envelope,
    const ScalarInterval& incoming_start_velocity) noexcept {
  const ScalarInterval starts = intersect(
      envelope.start_velocity, incoming_start_velocity);
  if (starts.empty() || envelope.duration <= 0.0) return ScalarInterval{};
  const double twice_average =
      2.0 * envelope.displacement / envelope.duration;
  return ScalarInterval{
      twice_average - starts.upper,
      twice_average - starts.lower};
}

namespace {

void clip_rate_relation(
    CubicBoundaryRateRelation& relation,
    double a,
    double b,
    double c) {
  if (relation.vertex_count == 0) return;
  std::array<
      CubicBoundaryRateRelation::Point,
      CubicBoundaryRateRelation::kMaximumVertices> result{};
  std::size_t result_count = 0;
  const auto value = [&](const CubicBoundaryRateRelation::Point& point) {
    return a * point.start + b * point.end - c;
  };
  const auto append = [&](const CubicBoundaryRateRelation::Point& point) {
    if (result_count >= result.size()) {
      throw std::logic_error("cubic boundary-rate polygon is too complex");
    }
    result[result_count++] = point;
  };

  for (std::size_t i = 0; i < relation.vertex_count; ++i) {
    const auto& current = relation.vertices[i];
    const auto& next = relation.vertices[(i + 1) % relation.vertex_count];
    const double current_value = value(current);
    const double next_value = value(next);
    const bool current_inside = current_value <= 1e-12;
    const bool next_inside = next_value <= 1e-12;
    if (current_inside) append(current);
    if (current_inside != next_inside) {
      const double fraction =
          current_value / (current_value - next_value);
      append(CubicBoundaryRateRelation::Point{
          current.start + fraction * (next.start - current.start),
          current.end + fraction * (next.end - current.end)});
    }
  }
  relation.vertices = result;
  relation.vertex_count = result_count;
}

}  // namespace

CubicBoundaryRateRelation make_cubic_boundary_rate_relation(
    double displacement,
    double duration,
    double max_abs_velocity,
    double max_abs_acceleration) {
  if (!std::isfinite(displacement) || displacement < 0.0 ||
      !std::isfinite(duration) || duration <= 0.0 ||
      !std::isfinite(max_abs_velocity) || max_abs_velocity <= 0.0 ||
      std::isnan(max_abs_acceleration) || max_abs_acceleration <= 0.0) {
    throw std::invalid_argument("invalid cubic boundary-rate limits");
  }

  CubicBoundaryRateRelation relation;
  relation.max_abs_velocity = max_abs_velocity;
  relation.vertex_count = 4;
  relation.vertices[0] = {0.0, 0.0};
  relation.vertices[1] = {max_abs_velocity, 0.0};
  relation.vertices[2] = {max_abs_velocity, max_abs_velocity};
  relation.vertices[3] = {0.0, max_abs_velocity};

  // The velocity is a quadratic Bezier curve with control values
  // v0, 3*d/T-v0-v1, v1. Keeping all three in [0, vmax] is conservative and
  // guarantees both monotone progress and the continuous-time speed bound.
  const double middle_sum = 3.0 * displacement / duration;
  clip_rate_relation(relation, 1.0, 1.0, middle_sum);
  clip_rate_relation(
      relation, -1.0, -1.0, max_abs_velocity - middle_sum);
  if (!relation.feasible()) return relation;

  if (std::isfinite(max_abs_acceleration)) {
    const double inverse_duration = 1.0 / duration;
    const double offset = 6.0 * displacement /
                          (duration * duration);
    // a(0) = offset - (4 v0 + 2 v1) / T
    // a(T) = -offset + (2 v0 + 4 v1) / T
    const std::array<std::array<double, 3>, 4> constraints{{
        {{-4.0 * inverse_duration, -2.0 * inverse_duration,
          max_abs_acceleration - offset}},
        {{4.0 * inverse_duration, 2.0 * inverse_duration,
          max_abs_acceleration + offset}},
        {{2.0 * inverse_duration, 4.0 * inverse_duration,
          max_abs_acceleration + offset}},
        {{-2.0 * inverse_duration, -4.0 * inverse_duration,
          max_abs_acceleration - offset}},
    }};
    for (const auto& constraint : constraints) {
      clip_rate_relation(
          relation, constraint[0], constraint[1], constraint[2]);
      if (!relation.feasible()) return relation;
    }
  }

  return relation;
}

ScalarInterval CubicBoundaryRateRelation::propagate(
    const ScalarInterval& incoming_start_velocity) const noexcept {
  const ScalarInterval starts = intersect(
      incoming_start_velocity, ScalarInterval{0.0, max_abs_velocity});
  if (starts.empty() || !feasible()) return ScalarInterval{};

  ScalarInterval result{
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
  const auto include_end = [&](double end_rate) {
    result.lower = std::min(result.lower, end_rate);
    result.upper = std::max(result.upper, end_rate);
  };
  for (std::size_t i = 0; i < vertex_count; ++i) {
    const Point& current = vertices[i];
    const Point& next = vertices[(i + 1) % vertex_count];
    if (current.start >= starts.lower - 1e-12 &&
        current.start <= starts.upper + 1e-12) {
      include_end(current.end);
    }
    const double delta_start = next.start - current.start;
    if (std::abs(delta_start) <= 1e-15) continue;
    for (double boundary : {starts.lower, starts.upper}) {
      const double fraction = (boundary - current.start) / delta_start;
      if (fraction >= -1e-12 && fraction <= 1.0 + 1e-12) {
        include_end(
            current.end + fraction * (next.end - current.end));
      }
    }
  }
  return result;
}

ScalarInterval propagate_cubic_boundary_rates(
    double displacement,
    double duration,
    double max_abs_velocity,
    double max_abs_acceleration,
    const ScalarInterval& incoming_start_velocity) {
  return make_cubic_boundary_rate_relation(
             displacement,
             duration,
             max_abs_velocity,
             max_abs_acceleration)
      .propagate(incoming_start_velocity);
}

Point2 body_point_acceleration(
    const Point2& center_acceleration,
    double yaw,
    double angular_velocity,
    double angular_acceleration,
    const Point2& body_point) {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const Point2 world_offset{
      cosine * body_point.x - sine * body_point.y,
      sine * body_point.x + cosine * body_point.y};
  return Point2{
      center_acceleration.x -
          angular_acceleration * world_offset.y -
          angular_velocity * angular_velocity * world_offset.x,
      center_acceleration.y +
          angular_acceleration * world_offset.x -
          angular_velocity * angular_velocity * world_offset.y};
}

double max_rectangle_corner_acceleration(
    const Point2& center_acceleration,
    double yaw,
    double angular_velocity,
    double angular_acceleration,
    double length,
    double width) {
  if (length <= 0.0 || width <= 0.0) {
    throw std::invalid_argument("rectangle dimensions must be positive");
  }
  const double half_length = 0.5 * length;
  const double half_width = 0.5 * width;
  const std::array<Point2, 4> corners{
      Point2{half_length, half_width},
      Point2{half_length, -half_width},
      Point2{-half_length, half_width},
      Point2{-half_length, -half_width}};
  double maximum = 0.0;
  for (const Point2& corner : corners) {
    const Point2 acceleration = body_point_acceleration(
        center_acceleration, yaw, angular_velocity,
        angular_acceleration, corner);
    maximum = std::max(maximum, std::hypot(acceleration.x, acceleration.y));
  }
  return maximum;
}

double conservative_rectangle_acceleration_bound(
    const Point2& center_acceleration,
    double angular_velocity,
    double angular_acceleration,
    double length,
    double width) {
  if (length <= 0.0 || width <= 0.0) {
    throw std::invalid_argument("rectangle dimensions must be positive");
  }
  const double radius = 0.5 * std::hypot(length, width);
  return std::hypot(center_acceleration.x, center_acceleration.y) +
         radius * (std::abs(angular_acceleration) +
                   angular_velocity * angular_velocity);
}

}  // namespace lacam_primitive
