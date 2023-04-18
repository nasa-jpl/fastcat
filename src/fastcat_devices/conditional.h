#ifndef FASTCAT_CONDITIONAL_H_
#define FASTCAT_CONDITIONAL_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{

ConditionalOperatorType ConditionalTypeFromString(std::string cond_type);

class Conditional : public DeviceBase
{
 public:
  Conditional();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;

 protected:
  std::string             conditional_type_string_;
  ConditionalOperatorType conditional_type_;
  double                  compare_rhs_value_{0};
};

}  // namespace fastcat

#endif
