#ifndef FASTCAT_CONDITIONAL_H_
#define FASTCAT_CONDITIONAL_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
enum ConditionalType {
  LT,  // <
  LE,  // <=
  GT,  // >
  GE,  // >=
  EQ,  // ==, limited use for double types
  NE,  // !=, limited use for double types
  BAD_CONDITIONAL_TYPE
};

ConditionalType ConditionalTypeFromString(std::string cond_type);

class Conditional : public DeviceBase
{
 public:
  Conditional();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;

 protected:
  std::string          conditional_type_string_;
  enum ConditionalType conditional_type_  = BAD_CONDITIONAL_TYPE;
  double               compare_rhs_value_ = 0.0;
};

}  // namespace fastcat

#endif
