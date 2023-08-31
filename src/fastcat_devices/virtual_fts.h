#ifndef FASTCAT_VIRTUAL_FTS_H_
#define FASTCAT_VIRTUAL_FTS_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/fastcat_devices/fts.h"
#include "fastcat/transform_utils.h"

#define FC_FTS_N_DIMS 6

namespace fastcat
{
/**
 * @brief Class that takes in the output of an FTS device and applies a
 * transform to it.
 */
class VirtualFts : public Fts
{
 public:
  VirtualFts();  ///< Fts constructor.
  /**
   * @brief Parses input yaml file to set sensor parameters.
   * @param node The portion of the yaml file corresponding to this Virtual FTS
   * device.
   * @return True if configuration completes without error; false otherwise.
   */
  bool ConfigFromYaml(const YAML::Node& node);
  /**
   * @brief Calculates wrench (forces and torques) from input signals.
   * @return True if device state is read without error; false otherwise.
   */
  bool Read();

 private:
  transform tf_ = {};
};

}  // namespace fastcat

#endif
