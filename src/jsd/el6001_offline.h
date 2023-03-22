#ifndef FASTCAT_EL6001_OFFLINE_H_
#define FASTCAT_EL6001_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el6001.h"

namespace fastcat
{
class El6001Offline : public El6001
{
 public:
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif