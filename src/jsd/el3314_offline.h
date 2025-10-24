#ifndef FASTCAT_EL3314_OFFLINE_H_
#define FASTCAT_EL3314_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el3314.h"

namespace fastcat
{
class El3314Offline : public El3314
{
 public:
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;
};

}  // namespace fastcat

#endif
