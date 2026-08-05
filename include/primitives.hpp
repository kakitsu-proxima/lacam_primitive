#pragma once

#include <string>
#include <vector>

#include "geometry.hpp"
#include "types.hpp"

namespace lacam_primitive {

struct Primitive {
  PrimitiveId id = 0;
  std::string name;
  int dx = 0;
  int dy = 0;
  int d_heading = 0;
};

struct PrimitiveVariant {
  State delta;
  std::vector<CellMask> interval_masks;
};

class PrimitiveTable {
 public:
  explicit PrimitiveTable(const Problem& problem);

  [[nodiscard]] const Primitive& primitive(PrimitiveId id) const;
  [[nodiscard]] const PrimitiveVariant& variant(PrimitiveId id, int start_heading) const;
  [[nodiscard]] State apply(const State& state, PrimitiveId id) const;
  [[nodiscard]] const std::vector<Primitive>& primitives() const { return primitives_; }
  [[nodiscard]] PrimitiveId wait_id() const { return wait_id_; }
  [[nodiscard]] int max_translation_cells() const { return max_translation_cells_; }
  [[nodiscard]] int max_rotation_bins() const { return max_rotation_bins_; }
  [[nodiscard]] std::size_t interval_count() const { return interval_count_; }

 private:
  Problem problem_;
  std::vector<Primitive> primitives_;
  std::vector<std::vector<PrimitiveVariant>> variants_;
  PrimitiveId wait_id_ = 0;
  int max_translation_cells_ = 1;
  int max_rotation_bins_ = 1;
  std::size_t interval_count_ = 1;

  void generate_primitives();
  void build_variants();
};

}  // namespace lacam_primitive
