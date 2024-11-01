#ifndef FASTCAT_EL5042_H_
#define FASTCAT_EL5042_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el5042_pub.h"

namespace fastcat
{
class El5042 : public JsdDeviceBase
{
 public:
  El5042();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;

 protected:
  bool ConfigFromYamlCommon(const YAML::Node& node);

  bool ClockFrequencyFromString(std::string           clock_frequency_string,
                         jsd_el5042_clock_t& clock_frequency);

  std::vector<std::string> clock_frequency_strings_;

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
