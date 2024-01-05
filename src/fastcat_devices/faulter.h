#ifndef FASTCAT_FAULTER_H_
#define FASTCAT_FAULTER_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
/**
 * @brief Fastcat device class that can be used to trigger a fault based on a
 * signal.
 */
class Faulter : public DeviceBase
{
 public:
  Faulter();  ///< Faulter constructor.
  /**
   * @brief Parses input yaml file to set sensor parameters.
   * @param node The portion of the yaml file corresponding to this Faulter
   * device.
   * @return True if configuration completes without error; false otherwise.
   */
  bool ConfigFromYaml(const YAML::Node& node) override;
  /**
   * @brief Updates device state.
   * @return True if device state is read without error; false otherwise.
   */
  bool Read() override;
  /**
   * @brief Checks if fault-trigger signal is present.
   * @return FaultType enum value corresponding to appropriate fault state.
   */
  FaultType Process() override;
  /**
   * @brief Commands device. Only FaulterEnableCmd is accepted currently.
   * @param cmd Command provided to Faulter device, of a type that is a subclass
   * of DeviceCmd.
   * @return True if command is accepted; false otherwise.
   */
  bool Write(DeviceCmd& cmd) override;
  void Fault() override;  ///< Disables device.
  void Reset() override;  ///< Enables device if device starts enabled.

 protected:
  bool start_enabled_{true};  ///< YAML configurable, recommended value is true
};

}  // namespace fastcat

#endif
