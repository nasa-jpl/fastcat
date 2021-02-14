#ifndef FASTCAT_EL2124_H_
#define FASTCAT_EL2124_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"
#include "jsd/jsd_el2124_pub.h"

namespace fastcat
{
class El2124 : public DeviceBase
{
 public:
  El2124();
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);

 private:
  jsd_slave_config_t jsd_slave_config_ = {0};
};

}  // namespace fastcat

#endif
