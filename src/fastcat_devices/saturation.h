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
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;

 protected:
  double lower_limit_;
  double upper_limit_;
};

}  // namespace fastcat

#endif
