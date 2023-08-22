#ifndef FASTCAT_LINEAR_INTERPOLATION_H_
#define FASTCAT_LINEAR_INTERPOLATION_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{

class LinearInterpolation : public DeviceBase
{
 public:
  LinearInterpolation();
  bool ConfigFromYaml(YAML::Node node, double external_time = -1) override;
  bool Read() override;

 protected:
  // Config paramters
  std::vector<std::pair<double, double>> domain_range_;
  bool                                   enable_out_of_bounds_fault_ = false;

  std::vector<double> slope_;  // computed during initialization
};

}  // namespace fastcat

#endif
