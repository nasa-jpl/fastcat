#ifndef FASTCAT_ILD1900_H_
#define FASTCAT_ILD1900_H_

// Include related header (for cc files)

// Include C then C++ libraries

// Include external then project includes
#include "fastcat/jsd/jsd_device_base.h"
#include "jsd/jsd_ild1900_pub.h"

namespace fastcat
{
class Ild1900 : public JsdDeviceBase
{
 public:
  Ild1900();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;

 protected:
  bool ConfigFromYamlCommon(const YAML::Node& node);

 private:
  bool ModelFromString(std::string model_string, jsd_ild1900_model_t& model);
  bool AveragingTypeFromString(std::string              averaging_type_string,
                               jsd_ild1900_averaging_t& averaging_type);
  bool ExposureModeFromString(std::string                  exposure_mode_string,
                              jsd_ild1900_exposure_mode_t& exposure_mode);
  bool PeakSelectionFromString(std::string peak_selection_string,
                               jsd_ild1900_peak_selection_t& peak_selection);

  jsd_slave_config_t jsd_slave_config_{0};
};

}  // namespace fastcat

#endif
