#ifndef FASTCAT_EL3202_H_
#define FASTCAT_EL3202_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el3202_pub.h"

namespace fastcat
{
class El3202 : public JsdDeviceBase
{
 public:
  El3202();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);
  bool ElementFromString(std::string element_string, jsd_el3202_element_t& element);
  std::string        element_ch1_string_;
  std::string        element_ch2_string_;
  jsd_el3202_element_t element_ch1_{JSD_EL3202_ELEMENT_PT100};
  jsd_el3202_element_t element_ch2_{JSD_EL3202_ELEMENT_PT100};

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
