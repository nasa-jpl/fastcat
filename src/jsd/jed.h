#ifndef FASTCAT_JED_H_
#define FASTCAT_JED_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"
#include "jsd/jsd_jed_pub.h"

namespace fastcat
{
class Jed : public DeviceBase
{
 public:
  Jed();
  bool      ConfigFromYaml(YAML::Node node) override;
  bool      Read() override;
  FaultType Process() override;
  bool      Write(DeviceCmd& cmd) override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);

  uint16_t initial_cmd_ = 0;

 private:
  jsd_slave_config_t jsd_slave_config_;
};

}  // namespace fastcat

#endif
