#ifndef FASTCAT_JED0200_OFFLINE_H_
#define FASTCAT_JED0200_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jed0200.h"

namespace fastcat
{
class Jed0200Offline : public Jed0200
{
 public:
  bool      ConfigFromYaml(const YAML::Node& node, double external_time = -1) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif
