#ifndef FASTCAT_SCHMITT_TRIGGER_H_
#define FASTCAT_SCHMITT_TRIGGER_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
class SchmittTrigger : public DeviceBase
{
 public:
  SchmittTrigger();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;

 protected:
  double low_threshold_{0};
  double high_threshold_{0};
};

}  // namespace fastcat

#endif
