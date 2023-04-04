#ifndef FASTCAT_FBC_OFFLINE_H_
#define FASTCAT_FBC_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/fbc.h"

namespace fastcat
{
class FbcOffline : public Fbc
{
 public:
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
};

}  // namespace fastcat

#endif