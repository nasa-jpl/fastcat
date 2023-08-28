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
  bool ConfigFromYaml(const YAML::Node& node, double external_time = -1) override;

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
   *        Currently, only the SEED_THERMAL_MODEL_TEMPERATURE_CMD, used to
   * reseed the model is accepted
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
  double winding_thermal_cor_{0.0};     ///< coefficient of resistance
  double k1_{0.0}, k2_{0.0}, k3_{0.0};  ///< weights for T4 estimate

  // declare fault protection parameters
  std::vector<double> max_allowable_temps_{0.0, 0.0, 0.0, 0.0};
  uint32_t            persistence_limit_{
      0};  ///< represents how many time cycles a temperature limit is able to
                      ///< be exceeded before throwing a fault
  double ref_temp_{0.0};  ///< the reference temperature for the winding
                          ///< resistance parameter, along with being used for
                          ///< calculating the dynamically varying resistance

  // declare variables for storing signal data and estimates
  double motor_current_{
      0.0};  ///< this value is retrieved from a motor controller measurement
  double motor_res_{0.0};  ///< this value is estimated based on the temp 1
                           ///< estimate and represents the resistance of the
                           ///< motor, which is used for calculated power
  std::vector<double> node_temps_{
      0.0, 0.0, 0.0, 0.0};  ///< this value is estimated from the model and
                            ///< represents winding temperature
  std::vector<size_t> node_overtemp_persistences_{
      0, 0, 0, 0};  ///< this is used as a counter for how many cycles node 1
                    ///< has been over the temperature limit

  // tracked variables
  double last_time_{0.0};

  // constants
  // the required number of signals for this device
  static constexpr size_t FC_TNTM_NUM_SIGNALS = 2;
  // signal index for node 3 temperature
  static constexpr size_t NODE_3_TEMP_IDX = 0;
  // signal index for motor current
  static constexpr size_t MOTOR_CURRENT_IDX = 1;
};
}  // namespace fastcat

#endif
