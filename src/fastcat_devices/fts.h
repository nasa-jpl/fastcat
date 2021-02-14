#ifndef FASTCAT_FTS_H_
#define FASTCAT_FTS_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

#define FC_FTS_N_DIMS 6

namespace fastcat
{
/**
 * @brief Class implementing force-torque sensor through fastcat.
 */
class Fts : public DeviceBase
{
 public:
  Fts();  ///< Fts constructor.
          /**
           * @brief Parses input yaml file to set sensor parameters.
           * @param node The portion of the yaml file corresponding to this FTS device.
           * @return True if configuration completes without error; false otherwise.
           */
  bool ConfigFromYaml(YAML::Node node) override;
  /**
   * @brief Calculates wrench (forces and torques) from input signals.
   * @return True if device state is read without error; false otherwise.
   */
  bool Read() override;
  /**
   * @brief Checks if forces and torques have exceeded acceptable maxima.
   * @return FaultType enum value corresponding to appropriate fault state.
   */
  FaultType Process() override;
  /**
   * @brief Commands device.
   * Only FtsTareCmd is accepted currently.
   * @param cmd Command provided to FTS device, of a type that is a subclass of
   * DeviceCmd.
   * @return True if command is accepted; false otherwise.
   */
  bool Write(DeviceCmd& cmd) override;

 protected:
  std::vector<std::vector<double>>
         calibration_;  ///< Calibration matrix of doubles for wrench calculation.
  double wrench_[FC_FTS_N_DIMS] = {
      0};  ///< Array where wrench values are stored.
  double sig_offset_[FC_FTS_N_DIMS] = {
      0};                 ///< Double array for storing tare offsets.
  double max_force_{0};   ///< If the L2 norm of the force components exceeds
                          ///< this value, the entire fastcat system will fault.
  double max_torque_{0};  ///< If the L2 norm of the torque components exceeds
                          ///< this value, the entire fastcat system will fault.
};

}  // namespace fastcat

#endif
