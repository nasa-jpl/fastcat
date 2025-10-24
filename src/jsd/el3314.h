#ifndef FASTCAT_EL3314_H_
#define FASTCAT_EL3314_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el3314_pub.h"

namespace fastcat
{
class El3314 : public JsdDeviceBase
{
 public:
  El3314();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;

 protected:
  bool                 ConfigFromYamlCommon(const YAML::Node& node);
  bool                 ElementFromString(std::string           element_string,
                                         jsd_el3314_element_t& element);
  std::string          element_string_;
  jsd_el3314_element_t element_{JSD_EL3314_ELEMENT_TYPE_K};

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
