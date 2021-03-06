#ifndef FASTCAT_FUNCTION_H_
#define FASTCAT_FUNCTION_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
enum FunctionType { POLYNOMIAL, BAD_FUNCTION_TYPE };

class Function : public DeviceBase
{
 public:
  Function();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;

 protected:
  std::string         function_type_string_;
  enum FunctionType   function_type_;
  int                 order_{0};
  std::vector<double> coefficients_;
};

}  // namespace fastcat

#endif
