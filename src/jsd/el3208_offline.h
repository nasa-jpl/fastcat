#ifndef FASTCAT_EL3208_OFFLINE_H_
#define FASTCAT_EL3208_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el3208.h"

namespace fastcat
{
class El3208Offline : public El3208
{
 public:
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;
};

}  // namespace fastcat

#endif
