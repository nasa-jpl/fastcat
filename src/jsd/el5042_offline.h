#ifndef FASTCAT_EL5042_OFFLINE_H_
#define FASTCAT_EL5042_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el5042.h"

namespace fastcat
{
class El5042Offline : public El5042
{
 public:
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;
};

}  // namespace fastcat

#endif