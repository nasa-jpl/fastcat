#ifndef FASTCAT_ATI_FTS_H_
#define FASTCAT_ATI_FTS_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_ati_fts_pub.h"

namespace fastcat
{
class AtiFts : public JsdDeviceBase
{
 public:
  AtiFts();
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);

 private:
  jsd_slave_config_t jsd_slave_config_{0};
  bool               ati_error_       = false;
  uint32_t           ati_status_code_ = 0;

  double bias_[6] = {0};

  bool   enable_fts_guard_fault_ = true;
  double max_force_[3]           = {0, 0, 0};
  double max_torque_[3]          = {0, 0, 0};
};

}  // namespace fastcat

#endif
