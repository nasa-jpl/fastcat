#ifndef FASTCAT_FUNCTION_H_
#define FASTCAT_FUNCTION_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
enum FunctionType { POLYNOMIAL, SUMMATION, MULTIPLICATION, INVERSION, BAD_FUNCTION_TYPE };
FunctionType FunctionTypeFromString(const std::string&);

class Function : public DeviceBase
{
 public:
  Function();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
 protected:
  std::string         function_type_string_;
  enum FunctionType   function_type_;
  
  // POLYNOMIAL member variables
  int                 order_{0};
  std::vector<double> coefficients_;

  // SUMMATION member variabls
  std::vector<std::string> signals_;
};

}  // namespace fastcat

#endif
