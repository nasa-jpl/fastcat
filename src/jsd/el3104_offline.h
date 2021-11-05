#ifndef FASTCAT_EL3104_OFFLINE_H_
#define FASTCAT_EL3104_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/el3104.h"

namespace fastcat
{
class El3104Offline : public El3104
{
 public:
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
};

}  // namespace fastcat

#endif
