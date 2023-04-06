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
   *        limits.
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
  // declare motor parameters
  double thermal_mass_node_1_{0.0};
  double thermal_mass_node_2_{0.0};
  double thermal_res_nodes_1_to_2_{
      0.0};  ///< thermal resistance from node 1 to 2 (deg C / W)
  double thermal_res_nodes_2_to_3_{
      0.0};  ///< thermal resistance from node 2 to 3 (deg C / W)
  double winding_res_{0.0};  ///< motor winding electrical resistance (ohms)
  double winding_thermal_cor_{0.0};  ///< coefficient of resistance
  double k1_{0.0}, k3_{0.0};         ///< weights for T4 estimate

  // declare fault protection parameters
  double max_allowable_temp_1_{0.0};
  double max_allowable_temp_2_{0.0};
  double max_allowable_temp_3_{0.0};
  double max_allowable_temp_4_{0.0};
  size_t persistance_limit_{
      0};  ///< represents how many time cycles a temperature limit is able to
           ///< be exceeded before throwing a fault

  // declare variables for storing signal data and estimates
  double motor_current_{
      0.0};  ///< this value is retrieved from a motor controller measurement
  double motor_resistance_{
      0.0};  ///< this value is estimated based on the temp 1 estimate and
             ///< represents the resistance of the motor, which is used for
             ///< calculated power
  double node_1_temp_{0.0};  ///< this value is estimated from the model and
                             ///< represents winding temperature
  double node_2_temp_{0.0};  ///< this value is estimated from the model and
                             ///< represents stator temperature
  double node_3_temp_{0.0};  ///< this value is measured directly using a sensor
  double node_4_temp_{
      0.0};  ///< this value is estimated from the model and generally
             ///< represents all other component temperatures
};
}  // namespace fastcat

#endif
