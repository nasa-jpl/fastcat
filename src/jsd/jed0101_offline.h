#ifndef FASTCAT_JED0101_OFFLINE_H_
#define FASTCAT_JED0101_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jed0101.h"

namespace fastcat
{
class Jed0101Offline : public Jed0101
{
 public:
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif
