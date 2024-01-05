#ifndef FASTCAT_EL1008_H_
#define FASTCAT_EL1008_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el1008_pub.h"

namespace fastcat
{
class El1008 : public JsdDeviceBase
{
 public:
  El1008();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;

 protected:
  bool ConfigFromYamlCommon(const YAML::Node& node);

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
