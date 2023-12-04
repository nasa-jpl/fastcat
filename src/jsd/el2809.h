#ifndef FASTCAT_EL2809_H_
#define FASTCAT_EL2809_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el2809_pub.h"

namespace fastcat
{
class El2809 : public JsdDeviceBase
{
 public:
  El2809();
  bool      ConfigFromYaml(const YAML::Node& node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;

 protected:
  bool ConfigFromYamlCommon(const YAML::Node& node);

 private:
  jsd_slave_config_t jsd_slave_config_ = {0};
};

}  // namespace fastcat

#endif
