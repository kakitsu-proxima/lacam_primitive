#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace lacam_primitive::yaml {

class Node {
 public:
  enum class Type { Null, Scalar, Map, Sequence };

  Node() = default;
  explicit Node(std::string scalar);
  static Node map();
  static Node sequence();

  [[nodiscard]] Type type() const { return type_; }
  [[nodiscard]] bool is_null() const { return type_ == Type::Null; }
  [[nodiscard]] bool is_scalar() const { return type_ == Type::Scalar; }
  [[nodiscard]] bool is_map() const { return type_ == Type::Map; }
  [[nodiscard]] bool is_sequence() const { return type_ == Type::Sequence; }

  [[nodiscard]] bool contains(const std::string& key) const;
  [[nodiscard]] const Node& at(const std::string& key) const;
  [[nodiscard]] Node& at(const std::string& key);
  [[nodiscard]] const Node& at(std::size_t index) const;
  [[nodiscard]] std::size_t size() const;

  void set(const std::string& key, Node value);
  void push(Node value);

  [[nodiscard]] std::string as_string() const;
  [[nodiscard]] int as_int() const;
  [[nodiscard]] std::size_t as_size() const;
  [[nodiscard]] double as_double() const;
  [[nodiscard]] bool as_bool() const;

 private:
  Type type_ = Type::Null;
  std::string scalar_;
  std::map<std::string, Node> map_;
  std::vector<Node> sequence_;
};

Node parse_file(const std::string& path);
Node parse(const std::string& text);

}  // namespace lacam_primitive::yaml
