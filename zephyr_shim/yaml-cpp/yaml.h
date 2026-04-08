#ifndef FASTCAT_ZEPHYR_SHIM_YAML_H_
#define FASTCAT_ZEPHYR_SHIM_YAML_H_

#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace YAML
{

class Node
{
 public:
  using sequence_type = std::vector<Node>;
  using iterator = sequence_type::iterator;
  using const_iterator = sequence_type::const_iterator;

  Node() = default;
  Node(const Node&) = default;
  Node(Node&&) = default;
  Node& operator=(const Node&) = default;
  Node& operator=(Node&&) = default;

  Node(const char* value) { set_scalar_string(value != nullptr ? value : ""); }
  Node(const std::string& value) { set_scalar_string(value); }
  Node(bool value) { set_scalar_string(value ? "true" : "false"); }

  template <typename T,
            typename std::enable_if<std::is_integral<T>::value &&
                                        !std::is_same<T, bool>::value,
                                    int>::type = 0>
  Node(T value)
  {
    set_scalar_string(std::to_string(static_cast<long long>(value)));
  }

  template <typename T,
            typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
  Node(T value)
  {
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    set_scalar_string(stream.str());
  }

  Node& operator=(const char* value)
  {
    set_scalar_string(value != nullptr ? value : "");
    return *this;
  }

  Node& operator=(const std::string& value)
  {
    set_scalar_string(value);
    return *this;
  }

  Node& operator=(bool value)
  {
    set_scalar_string(value ? "true" : "false");
    return *this;
  }

  template <typename T,
            typename std::enable_if<std::is_integral<T>::value &&
                                        !std::is_same<T, bool>::value,
                                    int>::type = 0>
  Node& operator=(T value)
  {
    set_scalar_string(std::to_string(static_cast<long long>(value)));
    return *this;
  }

  template <typename T,
            typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
  Node& operator=(T value)
  {
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    set_scalar_string(stream.str());
    return *this;
  }

  explicit operator bool() const
  {
    return data_ != nullptr && data_->kind != Kind::Undefined;
  }

  bool IsSequence() const
  {
    return data_ != nullptr && data_->kind == Kind::Sequence;
  }

  std::size_t size() const
  {
    if (data_ == nullptr) {
      return 0U;
    }

    if (data_->kind == Kind::Sequence) {
      return data_->sequence.size();
    }

    if (data_->kind == Kind::Map) {
      return data_->mapping.size();
    }

    return data_->kind == Kind::Undefined ? 0U : 1U;
  }

  Node operator[](const std::string& key) const
  {
    if (data_ == nullptr || data_->kind != Kind::Map) {
      return Node();
    }

    auto found = data_->mapping.find(key);
    if (found == data_->mapping.end()) {
      return Node();
    }

    return found->second;
  }

  Node operator[](const char* key) const
  {
    return operator[](std::string(key != nullptr ? key : ""));
  }

  Node operator[](std::size_t index) const
  {
    if (data_ == nullptr || data_->kind != Kind::Sequence ||
        index >= data_->sequence.size()) {
      return Node();
    }

    return data_->sequence[index];
  }

  Node operator[](int index) const
  {
    if (index < 0) {
      return Node();
    }

    return operator[](static_cast<std::size_t>(index));
  }

  Node& operator[](const std::string& key)
  {
    ensure_map();
    return data_->mapping[key];
  }

  Node& operator[](const char* key)
  {
    return operator[](std::string(key != nullptr ? key : ""));
  }

  Node& operator[](std::size_t index)
  {
    ensure_sequence();
    if (index >= data_->sequence.size()) {
      data_->sequence.resize(index + 1U);
    }
    return data_->sequence[index];
  }

  Node& operator[](int index)
  {
    if (index < 0) {
      throw std::out_of_range("YAML sequence index is negative");
    }

    return operator[](static_cast<std::size_t>(index));
  }

  void push_back(const Node& value)
  {
    ensure_sequence();
    data_->sequence.push_back(value);
  }

  template <typename T>
  void push_back(const T& value)
  {
    push_back(Node(value));
  }

  iterator begin()
  {
    return mutable_sequence().begin();
  }

  iterator end()
  {
    return mutable_sequence().end();
  }

  const_iterator begin() const
  {
    return const_sequence().begin();
  }

  const_iterator end() const
  {
    return const_sequence().end();
  }

  template <typename T>
  T as() const;

 private:
  enum class Kind
  {
    Undefined,
    Scalar,
    Sequence,
    Map,
  };

  struct Storage
  {
    Kind kind = Kind::Undefined;
    std::string scalar;
    sequence_type sequence;
    std::map<std::string, Node> mapping;
  };

  void ensure_map()
  {
    if (data_ == nullptr) {
      data_ = std::make_shared<Storage>();
    }

    if (data_->kind == Kind::Undefined) {
      data_->kind = Kind::Map;
    }

    if (data_->kind != Kind::Map) {
      throw std::runtime_error("YAML node is not a map");
    }
  }

  void ensure_sequence()
  {
    if (data_ == nullptr) {
      data_ = std::make_shared<Storage>();
    }

    if (data_->kind == Kind::Undefined) {
      data_->kind = Kind::Sequence;
    }

    if (data_->kind != Kind::Sequence) {
      throw std::runtime_error("YAML node is not a sequence");
    }
  }

  void set_scalar_string(const std::string& value)
  {
    if (data_ == nullptr) {
      data_ = std::make_shared<Storage>();
    }

    data_->kind = Kind::Scalar;
    data_->scalar = value;
    data_->sequence.clear();
    data_->mapping.clear();
  }

  const std::string& scalar() const
  {
    if (data_ == nullptr || data_->kind != Kind::Scalar) {
      throw std::runtime_error("YAML node does not hold a scalar");
    }

    return data_->scalar;
  }

  sequence_type& mutable_sequence()
  {
    static sequence_type empty_sequence;

    if (data_ == nullptr || data_->kind != Kind::Sequence) {
      return empty_sequence;
    }

    return data_->sequence;
  }

  const sequence_type& const_sequence() const
  {
    static const sequence_type empty_sequence;

    if (data_ == nullptr || data_->kind != Kind::Sequence) {
      return empty_sequence;
    }

    return data_->sequence;
  }

  std::shared_ptr<Storage> data_;
};

inline Node Load(const std::string&)
{
  return Node();
}

Node LoadFile(const std::string& path);

template <>
inline std::string Node::as<std::string>() const
{
  return scalar();
}

template <>
inline bool Node::as<bool>() const
{
  const std::string& value = scalar();

  if (value == "true" || value == "True" || value == "1") {
    return true;
  }

  if (value == "false" || value == "False" || value == "0") {
    return false;
  }

  throw std::runtime_error("Failed to parse YAML bool scalar: " + value);
}

template <>
inline double Node::as<double>() const
{
  return std::stod(scalar());
}

template <>
inline float Node::as<float>() const
{
  return std::stof(scalar());
}

template <>
inline int64_t Node::as<int64_t>() const
{
  return std::stoll(scalar());
}

template <>
inline uint64_t Node::as<uint64_t>() const
{
  return static_cast<uint64_t>(std::stoull(scalar()));
}

template <>
inline int32_t Node::as<int32_t>() const
{
  return static_cast<int32_t>(std::stol(scalar()));
}

template <>
inline uint32_t Node::as<uint32_t>() const
{
  return static_cast<uint32_t>(std::stoul(scalar()));
}

template <>
inline int16_t Node::as<int16_t>() const
{
  return static_cast<int16_t>(std::stoi(scalar()));
}

template <>
inline uint16_t Node::as<uint16_t>() const
{
  return static_cast<uint16_t>(std::stoul(scalar()));
}

template <>
inline int8_t Node::as<int8_t>() const
{
  return static_cast<int8_t>(std::stoi(scalar()));
}

template <>
inline uint8_t Node::as<uint8_t>() const
{
  return static_cast<uint8_t>(std::stoul(scalar()));
}

}  // namespace YAML

#endif
