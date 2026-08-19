#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "types.hpp"

namespace lacam_primitive {

struct RowSpan {
  int y = 0;
  int x_begin = 0;
  int x_end = 0;
};

struct CellMask {
  std::vector<RowSpan> rows;
};

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

// A counter-clockwise convex polygon in grid-cell coordinates. Unlike a
// CellMask, its geometric accuracy does not change when the planning grid is
// made coarser.
struct ConvexPolygon {
  std::vector<Point2> vertices;
};

struct AxisAlignedBounds {
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
};

[[nodiscard]] AxisAlignedBounds polygon_bounds(
    const ConvexPolygon& polygon);

[[nodiscard]] bool shifted_bounds_intersect(
    const AxisAlignedBounds& left,
    double left_x,
    double left_y,
    const AxisAlignedBounds& right,
    double right_x,
    double right_y);

class StaticGrid {
 public:
  StaticGrid() = default;
  StaticGrid(int width, int height);

  void add_rect(const ObstacleRect& rect);
  void finalize();

  [[nodiscard]] bool occupied(int x, int y) const;
  [[nodiscard]] bool intersects(const CellMask& mask, int anchor_x, int anchor_y) const;
  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }

 private:
  int width_ = 0;
  int height_ = 0;
  std::vector<std::uint8_t> occupied_;
  std::vector<int> prefix_;
};

[[nodiscard]] CellMask rasterize_oriented_rectangle(
    double center_x_cells,
    double center_y_cells,
    double yaw,
    double half_length_cells,
    double half_width_cells);

[[nodiscard]] CellMask union_masks(const std::vector<CellMask>& masks);

[[nodiscard]] bool shifted_intersects(
    const CellMask& left,
    int left_x,
    int left_y,
    const CellMask& right,
    int right_x,
    int right_y);

[[nodiscard]] std::vector<Point2> oriented_rectangle_corners(
    double center_x_cells,
    double center_y_cells,
    double yaw,
    double half_length_cells,
    double half_width_cells);

[[nodiscard]] ConvexPolygon convex_hull(std::vector<Point2> points);

[[nodiscard]] bool shifted_polygons_intersect(
    const ConvexPolygon& left,
    double left_x,
    double left_y,
    const ConvexPolygon& right,
    double right_x,
    double right_y);

[[nodiscard]] bool shifted_polygon_inside_bounds(
    const ConvexPolygon& polygon,
    double shift_x,
    double shift_y,
    double min_x,
    double min_y,
    double max_x,
    double max_y);

[[nodiscard]] bool shifted_polygon_intersects_rect(
    const ConvexPolygon& polygon,
    double shift_x,
    double shift_y,
    double min_x,
    double min_y,
    double max_x,
    double max_y);

}  // namespace lacam_primitive
