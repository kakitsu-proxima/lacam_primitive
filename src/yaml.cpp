#include "yaml.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace lacam_primitive::yaml {
namespace {

struct Line {
  int indent = 0;
  std::string text;
  int number = 0;
};

std::string trim(const std::string& input) {
  const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();
  if (first >= last) return {};
  return std::string(first, last);
}

std::string strip_comment(const std::string& input) {
  bool single = false;
  bool dual = false;
  int bracket_depth = 0;
  for (std::size_t i = 0; i < input.size(); ++i) {
    const char c = input[i];
    if (c == '\'' && !dual) single = !single;
    if (c == '"' && !single) dual = !dual;
    if (!single && !dual) {
      if (c == '[') ++bracket_depth;
      if (c == ']') --bracket_depth;
      if (c == '#' && bracket_depth == 0) return input.substr(0, i);
    }
  }
  return input;
}

std::size_t find_mapping_colon(const std::string& text) {
  bool single = false;
  bool dual = false;
  int bracket_depth = 0;
  for (std::size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (c == '\'' && !dual) single = !single;
    if (c == '"' && !single) dual = !dual;
    if (!single && !dual) {
      if (c == '[') ++bracket_depth;
      if (c == ']') --bracket_depth;
      if (c == ':' && bracket_depth == 0) return i;
    }
  }
  return std::string::npos;
}

Node parse_inline(std::string text) {
  text = trim(text);
  if (text.empty()) return Node();
  if (text.front() == '[') {
    if (text.back() != ']') throw std::runtime_error("unterminated inline sequence: " + text);
    Node result = Node::sequence();
    std::string body = text.substr(1, text.size() - 2);
    bool single = false;
    bool dual = false;
    int depth = 0;
    std::size_t begin = 0;
    for (std::size_t i = 0; i <= body.size(); ++i) {
      const bool at_end = i == body.size();
      const char c = at_end ? ',' : body[i];
      if (!at_end) {
        if (c == '\'' && !dual) single = !single;
        if (c == '"' && !single) dual = !dual;
        if (!single && !dual) {
          if (c == '[') ++depth;
          if (c == ']') --depth;
        }
      }
      if ((at_end || c == ',') && !single && !dual && depth == 0) {
        const std::string item = trim(body.substr(begin, i - begin));
        if (!item.empty()) result.push(parse_inline(item));
        begin = i + 1;
      }
    }
    return result;
  }
  if ((text.front() == '"' && text.back() == '"') ||
      (text.front() == '\'' && text.back() == '\'')) {
    text = text.substr(1, text.size() - 2);
  }
  return Node(text);
}

std::vector<Line> preprocess(const std::string& text) {
  std::vector<Line> lines;
  std::istringstream stream(text);
  std::string raw;
  int number = 0;
  while (std::getline(stream, raw)) {
    ++number;
    if (!raw.empty() && raw.back() == '\r') raw.pop_back();
    raw = strip_comment(raw);
    if (trim(raw).empty()) continue;
    int indent = 0;
    while (indent < static_cast<int>(raw.size()) && raw[static_cast<std::size_t>(indent)] == ' ') {
      ++indent;
    }
    if (indent < static_cast<int>(raw.size()) && raw[static_cast<std::size_t>(indent)] == '\t') {
      throw std::runtime_error("tabs are not supported in YAML indentation at line " + std::to_string(number));
    }
    lines.push_back(Line{indent, trim(raw), number});
  }
  return lines;
}

Node parse_block(const std::vector<Line>& lines, std::size_t& index, int indent);

void parse_map_entry(
    Node& map,
    const std::vector<Line>& lines,
    std::size_t& index,
    int indent,
    const std::string& entry_text) {
  const std::size_t colon = find_mapping_colon(entry_text);
  if (colon == std::string::npos) {
    throw std::runtime_error("expected key: value at line " + std::to_string(lines[index].number));
  }
  const std::string key = trim(entry_text.substr(0, colon));
  const std::string value = trim(entry_text.substr(colon + 1));
  if (key.empty()) throw std::runtime_error("empty key at line " + std::to_string(lines[index].number));

  ++index;
  if (!value.empty()) {
    map.set(key, parse_inline(value));
    return;
  }
  if (index >= lines.size() || lines[index].indent <= indent) {
    map.set(key, Node());
    return;
  }
  map.set(key, parse_block(lines, index, lines[index].indent));
}

Node parse_map(const std::vector<Line>& lines, std::size_t& index, int indent) {
  Node result = Node::map();
  while (index < lines.size() && lines[index].indent == indent &&
         lines[index].text.rfind("- ", 0) != 0 && lines[index].text != "-") {
    const std::string text = lines[index].text;
    parse_map_entry(result, lines, index, indent, text);
  }
  return result;
}

Node parse_sequence(const std::vector<Line>& lines, std::size_t& index, int indent) {
  Node result = Node::sequence();
  while (index < lines.size() && lines[index].indent == indent &&
         (lines[index].text == "-" || lines[index].text.rfind("- ", 0) == 0)) {
    const std::string remainder = trim(lines[index].text.substr(1));
    if (remainder.empty()) {
      ++index;
      if (index >= lines.size() || lines[index].indent <= indent) {
        result.push(Node());
      } else {
        result.push(parse_block(lines, index, lines[index].indent));
      }
      continue;
    }

    if (find_mapping_colon(remainder) != std::string::npos) {
      Node item = Node::map();
      parse_map_entry(item, lines, index, indent, remainder);
      while (index < lines.size() && lines[index].indent > indent) {
        const int child_indent = lines[index].indent;
        if (lines[index].text.rfind("- ", 0) == 0 || lines[index].text == "-") break;
        parse_map_entry(item, lines, index, child_indent, lines[index].text);
      }
      result.push(std::move(item));
    } else {
      result.push(parse_inline(remainder));
      ++index;
    }
  }
  return result;
}

Node parse_block(const std::vector<Line>& lines, std::size_t& index, int indent) {
  if (index >= lines.size()) return Node();
  if (lines[index].indent != indent) {
    throw std::runtime_error("inconsistent indentation at line " + std::to_string(lines[index].number));
  }
  if (lines[index].text == "-" || lines[index].text.rfind("- ", 0) == 0) {
    return parse_sequence(lines, index, indent);
  }
  return parse_map(lines, index, indent);
}

}  // namespace

Node::Node(std::string scalar) : type_(Type::Scalar), scalar_(std::move(scalar)) {}

Node Node::map() {
  Node result;
  result.type_ = Type::Map;
  return result;
}

Node Node::sequence() {
  Node result;
  result.type_ = Type::Sequence;
  return result;
}

bool Node::contains(const std::string& key) const {
  return type_ == Type::Map && map_.find(key) != map_.end();
}

const Node& Node::at(const std::string& key) const {
  if (type_ != Type::Map) throw std::runtime_error("YAML node is not a map");
  const auto found = map_.find(key);
  if (found == map_.end()) throw std::runtime_error("missing YAML key: " + key);
  return found->second;
}

Node& Node::at(const std::string& key) {
  if (type_ != Type::Map) throw std::runtime_error("YAML node is not a map");
  const auto found = map_.find(key);
  if (found == map_.end()) throw std::runtime_error("missing YAML key: " + key);
  return found->second;
}

const Node& Node::at(std::size_t index) const {
  if (type_ != Type::Sequence || index >= sequence_.size()) {
    throw std::runtime_error("YAML sequence index out of range");
  }
  return sequence_[index];
}

std::size_t Node::size() const {
  if (type_ == Type::Sequence) return sequence_.size();
  if (type_ == Type::Map) return map_.size();
  return 0;
}

void Node::set(const std::string& key, Node value) {
  if (type_ != Type::Map) throw std::runtime_error("YAML node is not a map");
  map_[key] = std::move(value);
}

void Node::push(Node value) {
  if (type_ != Type::Sequence) throw std::runtime_error("YAML node is not a sequence");
  sequence_.push_back(std::move(value));
}

std::string Node::as_string() const {
  if (type_ != Type::Scalar) throw std::runtime_error("YAML node is not a scalar");
  return scalar_;
}

int Node::as_int() const {
  const std::string value = as_string();
  std::size_t consumed = 0;
  const int result = std::stoi(value, &consumed);
  if (consumed != value.size()) throw std::runtime_error("invalid integer: " + value);
  return result;
}

std::size_t Node::as_size() const {
  const int value = as_int();
  if (value < 0) throw std::runtime_error("expected non-negative integer");
  return static_cast<std::size_t>(value);
}

double Node::as_double() const {
  const std::string value = as_string();
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size()) throw std::runtime_error("invalid number: " + value);
  return result;
}

bool Node::as_bool() const {
  std::string value = as_string();
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (value == "true" || value == "yes" || value == "on" || value == "1") {
    return true;
  }
  if (value == "false" || value == "no" || value == "off" || value == "0") {
    return false;
  }
  throw std::runtime_error("invalid boolean: " + value);
}

Node parse(const std::string& text) {
  const std::vector<Line> lines = preprocess(text);
  if (lines.empty()) return Node::map();
  std::size_t index = 0;
  Node root = parse_block(lines, index, lines.front().indent);
  if (index != lines.size()) {
    throw std::runtime_error("could not parse YAML near line " + std::to_string(lines[index].number));
  }
  return root;
}

Node parse_file(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open input file: " + path);
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return parse(buffer.str());
}

}  // namespace lacam_primitive::yaml
