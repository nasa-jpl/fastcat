#ifndef FASTCAT_EL3602_OFFLINE_H_
#define FASTCAT_EL3602_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el3602.h"

namespace fastcat
{
class El3602Offline : public El3602
{
 public:
  bool ConfigFromYaml(YAML::Node node, double external_time = -1) override;
  bool Read() override;
};

}  // namespace fastcat

#endif
