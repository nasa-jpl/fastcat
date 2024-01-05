#ifndef FASTCAT_EL3104_H_
#define FASTCAT_EL3104_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el3104_pub.h"

namespace fastcat
{
class El3104 : public JsdDeviceBase
{
 public:
  El3104();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;

 protected:
  bool ConfigFromYamlCommon(const YAML::Node& node);

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
