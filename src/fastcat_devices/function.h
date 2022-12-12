#ifndef FASTCAT_FUNCTION_H_
#define FASTCAT_FUNCTION_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
enum FunctionType { 
  POLYNOMIAL, 
  SUMMATION, 
  MULTIPLICATION, 
  POWER, 
  EXPONENTIAL,
  SIGMOID,
  BAD_FUNCTION_TYPE 
};
FunctionType FunctionTypeFromString(const std::string&);

typedef struct {
  int order = 0;
} PolynomialParams;

typedef struct {
  double exponent;
} PowerParams;

typedef struct {
  double base;
} ExponentialParams;

class Function : public DeviceBase
{
 public:
  Function();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
 protected:
  std::string         function_type_string_;
  enum FunctionType   function_type_;

  std::vector<double> polynomial_coefficients_;
  union {
    PolynomialParams polynomial_params_;
    PowerParams power_params_;
    ExponentialParams exponential_params_;
  };  
};

}  // namespace fastcat

#endif
