#ifndef FASTCAT_EL1008_OFFLINE_H_
#define FASTCAT_EL1008_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el1008.h"

namespace fastcat
{
class El1008Offline : public El1008
{
 public:
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
};

}  // namespace fastcat

#endif