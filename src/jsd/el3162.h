#ifndef FASTCAT_EL3162_H_
#define FASTCAT_EL3162_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el3162_pub.h"

namespace fastcat
{
class El3162 : public JsdDeviceBase
{
 public:
  El3162();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;

 protected:
  bool ConfigFromYamlCommon(const YAML::Node& node);

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
