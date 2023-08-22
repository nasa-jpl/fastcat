#ifndef FASTCAT_EL3208_H_
#define FASTCAT_EL3208_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_el3208_pub.h"

namespace fastcat
{
class El3208 : public JsdDeviceBase
{
 public:
  El3208();
  bool      ConfigFromYaml(YAML::Node node, double external_time = -1) override;
  bool      Read() override;
  FaultType Process() override;

 protected:
  bool ConfigFromYamlCommon(YAML::Node node);

  bool ElementFromString(std::string           element_string,
                         jsd_el3208_element_t& element);
  bool ConnectionFromString(std::string              connection_string,
                            jsd_el3208_connection_t& connection);

  std::vector<std::string> element_strings_;
  std::vector<std::string> connection_strings_;
  jsd_slave_config_t       jsd_slave_config_ = {0};

  double* outputs_[JSD_EL3208_NUM_CHANNELS]         = {0};
  double  low_threshold_[JSD_EL3208_NUM_CHANNELS]   = {0};
  double  high_threshold_[JSD_EL3208_NUM_CHANNELS]  = {0};
  bool    check_threshold_[JSD_EL3208_NUM_CHANNELS] = {0};
};

}  // namespace fastcat

#endif
