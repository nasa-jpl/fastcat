#ifndef FASTCAT_EL2828_OFFLINE_H_
#define FASTCAT_EL2828_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el2828.h"

namespace fastcat
{
class El2828Offline : public El2828
{
 public:
  bool      ConfigFromYaml(const YAML::Node& node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif
