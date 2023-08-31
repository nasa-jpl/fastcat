#ifndef FASTCAT_EGW_H_
#define FASTCAT_EGW_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_egd_pub.h"

namespace fastcat
{
class Egd : public JsdDeviceBase
{
 public:
  Egd();
  bool      ConfigFromYaml(const YAML::Node& node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;
  void      Fault() override;
  void      Reset() override;

 protected:
  bool ConfigFromYamlCommon(const YAML::Node& node);
  bool DriveCmdModeFromString(std::string               dcm_string,
                              jsd_egd_drive_cmd_mode_t& dcm);

  std::string        drive_cmd_mode_string_;
  jsd_slave_config_t jsd_slave_config_ = {0};
  jsd_egd_state_t    jsd_egd_state_    = {0};

  double cs_cmd_freq_hz_ = 0.0;

 private:
  bool WriteProfiledMode(DeviceCmd& cmd);
  bool WriteCSMode(DeviceCmd& cmd);
  bool GSModeFromString(std::string                      gs_mode_string,
                        jsd_elmo_gain_scheduling_mode_t& gs_mode);
};

}  // namespace fastcat

#endif
