#ifndef FASTCAT_EL2809_OFFLINE_H_
#define FASTCAT_EL2809_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el2809.h"

namespace fastcat
{
class El2809Offline : public El2809
{
 public:
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif
