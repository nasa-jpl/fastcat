#ifndef FASTCAT_EL3318_H_
#define FASTCAT_EL3318_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el3318_pub.h"

namespace fastcat
{
class El3318 : public JsdDeviceBase
{
 public:
  El3318();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);
  bool ElementFromString(std::string element_string, jsd_el3318_element_t& element);
  std::string        element_string_;
  jsd_el3318_element_t element_{JSD_EL3318_ELEMENT_TYPE_K};

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
