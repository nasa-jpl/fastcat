#ifndef FASTCAT_EGD_OFFLINE_H_
#define FASTCAT_EGD_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/egd.h"

namespace fastcat
{
class EgdOffline : public Egd
{
 public:
  EgdOffline();
  bool      ConfigFromYaml(const YAML::Node& node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
  void      Fault() override;
  void      Reset() override;

 private:
  bool ReadProfiledMode();
  bool ReadCSMode();
  bool WriteProfiledMode(DeviceCmd& cmd);
  bool WriteCSMode(DeviceCmd& cmd);

  jsd_egd_motion_command_t jsd_motion_cmd_ = {};
};

}  // namespace fastcat

#endif
