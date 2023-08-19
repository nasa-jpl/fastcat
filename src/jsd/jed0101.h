#ifndef FASTCAT_JED0101_H_
#define FASTCAT_JED0101_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_jed0101_pub.h"

namespace fastcat
{
class Jed0101 : public JsdDeviceBase
{
 public:
  Jed0101();
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);

  uint16_t initial_cmd_ = 0;

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
