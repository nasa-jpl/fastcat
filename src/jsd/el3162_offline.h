#ifndef FASTCAT_EL3162_OFFLINE_H_
#define FASTCAT_EL3162_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el3162.h"

namespace fastcat
{
class El3162Offline : public El3162
{
 public:
  bool ConfigFromYaml(const YAML::Node& node, double external_time = -1) override;
  bool Read() override;
};

}  // namespace fastcat

#endif