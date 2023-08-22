#ifndef FASTCAT_EL2124_OFFLINE_H_
#define FASTCAT_EL2124_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el2124.h"

namespace fastcat
{
class El2124Offline : public El2124
{
 public:
  bool      ConfigFromYaml(YAML::Node node, double external_time = -1) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif
