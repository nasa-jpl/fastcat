#ifndef FASTCAT_EL3602_H_
#define FASTCAT_EL3602_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el3602_pub.h"

namespace fastcat
{
class El3602 : public JsdDeviceBase
{
 public:
  El3602();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);
  bool RangeFromString(std::string range_string, jsd_el3602_range_t& range);
  std::string        range_ch1_string_;
  std::string        range_ch2_string_;
  jsd_el3602_range_t range_ch1_{JSD_EL3602_RANGE_10V};
  jsd_el3602_range_t range_ch2_{JSD_EL3602_RANGE_10V};

 private:
  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
