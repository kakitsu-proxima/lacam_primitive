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

}  // namespace lacam_primitive
