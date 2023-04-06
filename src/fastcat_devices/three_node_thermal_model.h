#ifndef FASTCAT_THREE_NODE_THERMAL_MODEL_H_
#define FASTCAT_THREE_NODE_THERMAL_MODEL_H_

// Include external then project includes
#include "fastcat/device_base.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

namespace fastcat
{
/**
 * @brief Class implementing a Three-Node Thermal Model for estimating
 *         internal motor temperatures, through fastcat.
 */
class ThreeNodeThermalModel : public DeviceBase
{
 public:
  ThreeNodeThermalModel();  ///< ThreeNodeThermalModel constructor.

  /**
   * @brief Parses input yaml file to set model constants and temperature
   * limits.
   * @param node The portion of the yaml file corresponding to this device.
   * @return True if configuration completes without error; false otherwise.
   */
  bool ConfigFromYaml(YAML::Node node) override;

  /**
   * @brief Reads in most recent temperature and current signal values, and
   *        stores them for further calculations.
   * @return True if device state is read without error; false otherwise.
   */
  bool Read() override;

  /**
   * @brief Performs one update step of the thermal prediction model, tracking
   *        the most recently predicted temperature values at each node, and
   *        reporting a fault if any of the limits are exceeded
   * @return FaultType enum value corresponding to appropriate fault state.
   */
  FaultType Process() override;

  /**
   * @brief Commands device
   *        TODO: describe the available commands
   * @param cmd Command provided to the device, of a type that is a subclass of
   * DeviceCmd
   * @return boolean for if the command was accepted and successful or not
   */
  bool Write(DeviceCmd& cmd) override;

 protected:
  // TODO: add constants and variables parsed from config
};
}  // namespace fastcat

#endif
