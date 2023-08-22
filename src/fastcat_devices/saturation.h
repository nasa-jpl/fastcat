#ifndef FASTCAT_SATURATION_H_
#define FASTCAT_SATURATION_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
class Saturation : public DeviceBase
{
 public:
  Saturation();
  bool ConfigFromYaml(YAML::Node node, double external_time = -1) override;
  bool Read() override;

 protected:
  double lower_limit_ = 0.0;
  double upper_limit_ = 0.0;
};

}  // namespace fastcat

#endif
