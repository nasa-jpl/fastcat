#ifndef FASTCAT_EL6001_H_
#define FASTCAT_EL6001_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el6001_pub.h"

namespace fastcat
{
class El6001 : public JsdDeviceBase
{
 public:
  El6001();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
  bool Write(DeviceCmd& cmd) override;
  // FaultType Process() override;
  // void      Fault() override;
  // void      Reset() override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);

  int baud_rate_;

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
