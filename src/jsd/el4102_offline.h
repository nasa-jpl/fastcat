#ifndef FASTCAT_EL4102_OFFLINE_H_
#define FASTCAT_EL4102_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el4102.h"

namespace fastcat
{
class El4102Offline : public El4102
{
 public:
  bool      ConfigFromYaml(const YAML::Node& node, double external_time = -1) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif