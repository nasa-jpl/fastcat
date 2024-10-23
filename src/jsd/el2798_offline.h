#ifndef FASTCAT_EL2798_OFFLINE_H_
#define FASTCAT_EL2798_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el2798.h"

namespace fastcat
{
class El2798Offline : public El2798
{
 public:
  bool      ConfigFromYaml(const YAML::Node& node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif
