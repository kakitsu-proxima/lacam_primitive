#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <map>
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

}  // namespace lacam_primitive
