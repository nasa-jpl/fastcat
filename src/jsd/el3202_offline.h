#ifndef FASTCAT_EL3202_OFFLINE_H_
#define FASTCAT_EL3202_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el3202.h"

namespace fastcat
{
class El3202Offline : public El3202
{
 public:
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
};

}  // namespace fastcat

#endif
