#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <stdexcept>

namespace lacam_primitive {

StaticGrid::StaticGrid(int width, int height)
    : width_(width), height_(height), occupied_(static_cast<std::size_t>(width * height), 0) {
  if (width <= 0 || height <= 0) throw std::invalid_argument("grid dimensions must be positive");
}

void StaticGrid::add_rect(const ObstacleRect& rect) {
  if (rect.width <= 0 || rect.height <= 0) throw std::invalid_argument("obstacle rectangle dimensions must be positive");
  for (int y = rect.y; y < rect.y + rect.height; ++y) {
    for (int x = rect.x; x < rect.x + rect.width; ++x) {
      if (x < 0 || y < 0 || x >= width_ || y >= height_) {
        throw std::invalid_argument("obstacle rectangle lies outside the grid");
      }
      occupied_[static_cast<std::size_t>(y * width_ + x)] = 1;
    }
  }
}

void StaticGrid::finalize() {
  prefix_.assign(static_cast<std::size_t>(height_ * (width_ + 1)), 0);
  for (int y = 0; y < height_; ++y) {
    const std::size_t row = static_cast<std::size_t>(y * (width_ + 1));
    for (int x = 0; x < width_; ++x) {
      prefix_[row + static_cast<std::size_t>(x + 1)] =
          prefix_[row + static_cast<std::size_t>(x)] +
          static_cast<int>(occupied_[static_cast<std::size_t>(y * width_ + x)]);
    }
  }
}

bool StaticGrid::occupied(int x, int y) const {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return true;
  return occupied_[static_cast<std::size_t>(y * width_ + x)] != 0;
}

bool StaticGrid::intersects(const CellMask& mask, int anchor_x, int anchor_y) const {
  if (prefix_.empty()) throw std::logic_error("StaticGrid::finalize() must be called before collision checks");
  for (const RowSpan& span : mask.rows) {
    const int y = anchor_y + span.y;
    const int x0 = anchor_x + span.x_begin;
    const int x1 = anchor_x + span.x_end;
    if (y < 0 || y >= height_ || x0 < 0 || x1 >= width_) return true;
    const std::size_t row = static_cast<std::size_t>(y * (width_ + 1));
    if (prefix_[row + static_cast<std::size_t>(x1 + 1)] -
            prefix_[row + static_cast<std::size_t>(x0)] >
        0) {
      return true;
    }
  }
  return false;
}

CellMask rasterize_oriented_rectangle(
    double center_x_cells,
    double center_y_cells,
    double yaw,
    double half_length_cells,
    double half_width_cells) {
  if (half_length_cells <= 0.0 || half_width_cells <= 0.0) {
    throw std::invalid_argument("rectangle half extents must be positive");
  }

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  // basis unit vectors
  const double u_x = c;
  const double u_y = s;
  const double v_x = -s;
  const double v_y = c;
  const double extent_x = std::abs(u_x) * half_length_cells + std::abs(v_x) * half_width_cells;
  const double extent_y = std::abs(u_y) * half_length_cells + std::abs(v_y) * half_width_cells;

  const int min_x = static_cast<int>(std::floor(center_x_cells - extent_x - 0.5));
  const int max_x = static_cast<int>(std::ceil(center_x_cells + extent_x + 0.5));
  const int min_y = static_cast<int>(std::floor(center_y_cells - extent_y - 0.5));
  const int max_y = static_cast<int>(std::ceil(center_y_cells + extent_y + 0.5));

  std::map<int, std::vector<int>> cells_by_row;
  constexpr double cell_half = 0.5;
  constexpr double epsilon = 1e-12;
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const double dx = static_cast<double>(x) - center_x_cells;
      const double dy = static_cast<double>(y) - center_y_cells;

      const bool world_x_ok = std::abs(dx) <=
          half_length_cells * std::abs(u_x) + half_width_cells * std::abs(v_x) + cell_half + epsilon;
      const bool world_y_ok = std::abs(dy) <=
          half_length_cells * std::abs(u_y) + half_width_cells * std::abs(v_y) + cell_half + epsilon;
      const bool rect_u_ok = std::abs(dx * u_x + dy * u_y) <=
          half_length_cells + cell_half * (std::abs(u_x) + std::abs(u_y)) + epsilon;
      const bool rect_v_ok = std::abs(dx * v_x + dy * v_y) <=
          half_width_cells + cell_half * (std::abs(v_x) + std::abs(v_y)) + epsilon;

      if (world_x_ok && world_y_ok && rect_u_ok && rect_v_ok) {
        cells_by_row[y].push_back(x);
      }
    }
  }

  CellMask result;
  for (auto& [y, xs] : cells_by_row) {
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    int begin = xs.front();
    int previous = xs.front();
    for (std::size_t i = 1; i < xs.size(); ++i) {
      if (xs[i] == previous + 1) {
        previous = xs[i];
        continue;
      }
      result.rows.push_back(RowSpan{y, begin, previous});
      begin = previous = xs[i];
    }
    result.rows.push_back(RowSpan{y, begin, previous});
  }
  return result;
}

CellMask union_masks(const std::vector<CellMask>& masks) {
  std::map<int, std::vector<std::pair<int, int>>> rows;
  for (const CellMask& mask : masks) {
    for (const RowSpan& span : mask.rows) rows[span.y].push_back({span.x_begin, span.x_end});
  }

  CellMask result;
  for (auto& [y, intervals] : rows) {
    std::sort(intervals.begin(), intervals.end());
    int begin = intervals.front().first;
    int end = intervals.front().second;
    for (std::size_t i = 1; i < intervals.size(); ++i) {
      if (intervals[i].first <= end + 1) {
        end = std::max(end, intervals[i].second);
      } else {
        result.rows.push_back(RowSpan{y, begin, end});
        begin = intervals[i].first;
        end = intervals[i].second;
      }
    }
    result.rows.push_back(RowSpan{y, begin, end});
  }
  return result;
}

bool shifted_intersects(
    const CellMask& left,
    int left_x,
    int left_y,
    const CellMask& right,
    int right_x,
    int right_y) {
  std::size_t i = 0;
  std::size_t j = 0;
  while (i < left.rows.size() && j < right.rows.size()) {
    const int left_row = left.rows[i].y + left_y;
    const int right_row = right.rows[j].y + right_y;
    if (left_row < right_row) {
      ++i;
      continue;
    }
    if (right_row < left_row) {
      ++j;
      continue;
    }

    const int left_begin = left.rows[i].x_begin + left_x;
    const int left_end = left.rows[i].x_end + left_x;
    const int right_begin = right.rows[j].x_begin + right_x;
    const int right_end = right.rows[j].x_end + right_x;
    if (std::max(left_begin, right_begin) <= std::min(left_end, right_end)) return true;
    if (left_end < right_end) {
      ++i;
    } else {
      ++j;
    }
  }
  return false;
}

std::vector<Point2> oriented_rectangle_corners(
    double center_x_cells,
    double center_y_cells,
    double yaw,
    double half_length_cells,
    double half_width_cells) {
  if (half_length_cells <= 0.0 || half_width_cells <= 0.0) {
    throw std::invalid_argument("rectangle half extents must be positive");
  }

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const Point2 longitudinal{c * half_length_cells, s * half_length_cells};
  const Point2 lateral{-s * half_width_cells, c * half_width_cells};

  return {
      Point2{center_x_cells + longitudinal.x + lateral.x,
             center_y_cells + longitudinal.y + lateral.y},
      Point2{center_x_cells - longitudinal.x + lateral.x,
             center_y_cells - longitudinal.y + lateral.y},
      Point2{center_x_cells - longitudinal.x - lateral.x,
             center_y_cells - longitudinal.y - lateral.y},
      Point2{center_x_cells + longitudinal.x - lateral.x,
             center_y_cells + longitudinal.y - lateral.y},
  };
}

namespace {

double cross(const Point2& origin, const Point2& a, const Point2& b) {
  return (a.x - origin.x) * (b.y - origin.y) -
         (a.y - origin.y) * (b.x - origin.x);
}

bool separated_on_polygon_axes(
    const ConvexPolygon& axes_from,
    double axes_shift_x,
    double axes_shift_y,
    const ConvexPolygon& left,
    double left_shift_x,
    double left_shift_y,
    const ConvexPolygon& right,
    double right_shift_x,
    double right_shift_y) {
  constexpr double epsilon = 1e-12;
  for (std::size_t i = 0; i < axes_from.vertices.size(); ++i) {
    const Point2& a = axes_from.vertices[i];
    const Point2& b =
        axes_from.vertices[(i + 1) % axes_from.vertices.size()];
    const double edge_x = b.x - a.x;
    const double edge_y = b.y - a.y;
    const double axis_x = -edge_y;
    const double axis_y = edge_x;

    double left_min = std::numeric_limits<double>::infinity();
    double left_max = -std::numeric_limits<double>::infinity();
    for (const Point2& point : left.vertices) {
      const double projection =
          (point.x + left_shift_x) * axis_x +
          (point.y + left_shift_y) * axis_y;
      left_min = std::min(left_min, projection);
      left_max = std::max(left_max, projection);
    }

    double right_min = std::numeric_limits<double>::infinity();
    double right_max = -std::numeric_limits<double>::infinity();
    for (const Point2& point : right.vertices) {
      const double projection =
          (point.x + right_shift_x) * axis_x +
          (point.y + right_shift_y) * axis_y;
      right_min = std::min(right_min, projection);
      right_max = std::max(right_max, projection);
    }

    // Touching footprints count as a collision. axes_shift is intentionally
    // unused in the projection: translating the edge does not change its
    // normal direction.
    (void)axes_shift_x;
    (void)axes_shift_y;
    if (left_max < right_min - epsilon ||
        right_max < left_min - epsilon) {
      return true;
    }
  }
  return false;
}

ConvexPolygon rectangle_polygon(
    double min_x,
    double min_y,
    double max_x,
    double max_y) {
  return ConvexPolygon{{
      Point2{min_x, min_y},
      Point2{max_x, min_y},
      Point2{max_x, max_y},
      Point2{min_x, max_y},
  }};
}

}  // namespace

AxisAlignedBounds polygon_bounds(const ConvexPolygon& polygon) {
  if (polygon.vertices.empty()) {
    throw std::invalid_argument("cannot bound an empty polygon");
  }
  AxisAlignedBounds result{
      polygon.vertices.front().x,
      polygon.vertices.front().y,
      polygon.vertices.front().x,
      polygon.vertices.front().y};
  for (const Point2& point : polygon.vertices) {
    result.min_x = std::min(result.min_x, point.x);
    result.min_y = std::min(result.min_y, point.y);
    result.max_x = std::max(result.max_x, point.x);
    result.max_y = std::max(result.max_y, point.y);
  }
  return result;
}

bool shifted_bounds_intersect(
    const AxisAlignedBounds& left,
    double left_x,
    double left_y,
    const AxisAlignedBounds& right,
    double right_x,
    double right_y) {
  constexpr double epsilon = 1e-12;
  return left.min_x + left_x <= right.max_x + right_x + epsilon &&
         right.min_x + right_x <= left.max_x + left_x + epsilon &&
         left.min_y + left_y <= right.max_y + right_y + epsilon &&
         right.min_y + right_y <= left.max_y + left_y + epsilon;
}

ConvexPolygon convex_hull(std::vector<Point2> points) {
  if (points.size() < 3) {
    throw std::invalid_argument("convex hull needs at least three points");
  }

  std::sort(points.begin(), points.end(), [](const Point2& left, const Point2& right) {
    if (left.x != right.x) return left.x < right.x;
    return left.y < right.y;
  });
  points.erase(
      std::unique(points.begin(), points.end(), [](const Point2& left, const Point2& right) {
        return left.x == right.x && left.y == right.y;
      }),
      points.end());

  std::vector<Point2> hull;
  hull.reserve(points.size() * 2);
  for (const Point2& point : points) {
    while (hull.size() >= 2 &&
           cross(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(point);
  }

  const std::size_t lower_size = hull.size();
  for (auto iterator = points.rbegin() + 1; iterator != points.rend(); ++iterator) {
    while (hull.size() > lower_size &&
           cross(hull[hull.size() - 2], hull.back(), *iterator) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(*iterator);
  }
  hull.pop_back();
  return ConvexPolygon{std::move(hull)};
}

bool shifted_polygons_intersect(
    const ConvexPolygon& left,
    double left_x,
    double left_y,
    const ConvexPolygon& right,
    double right_x,
    double right_y) {
  if (left.vertices.size() < 3 || right.vertices.size() < 3) {
    throw std::invalid_argument("polygon needs at least three vertices");
  }
  return !separated_on_polygon_axes(
             left, left_x, left_y,
             left, left_x, left_y,
             right, right_x, right_y) &&
         !separated_on_polygon_axes(
             right, right_x, right_y,
             left, left_x, left_y,
             right, right_x, right_y);
}

bool shifted_polygon_inside_bounds(
    const ConvexPolygon& polygon,
    double shift_x,
    double shift_y,
    double min_x,
    double min_y,
    double max_x,
    double max_y) {
  constexpr double epsilon = 1e-12;
  for (const Point2& point : polygon.vertices) {
    const double x = point.x + shift_x;
    const double y = point.y + shift_y;
    if (x < min_x - epsilon || x > max_x + epsilon ||
        y < min_y - epsilon || y > max_y + epsilon) {
      return false;
    }
  }
  return true;
}

bool shifted_polygon_intersects_rect(
    const ConvexPolygon& polygon,
    double shift_x,
    double shift_y,
    double min_x,
    double min_y,
    double max_x,
    double max_y) {
  const ConvexPolygon rectangle =
      rectangle_polygon(min_x, min_y, max_x, max_y);
  return shifted_polygons_intersect(
      polygon, shift_x, shift_y, rectangle, 0.0, 0.0);
}

}  // namespace lacam_primitive
