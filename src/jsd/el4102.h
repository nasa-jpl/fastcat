#ifndef FASTCAT_EL4102_H_
#define FASTCAT_EL4102_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el4102_pub.h"

namespace fastcat
{
class El4102 : public JsdDeviceBase
{
 public:
  El4102();
  bool      ConfigFromYaml(const YAML::Node& node, double external_time = -1) override;
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
