#ifndef FASTCAT_ILD1900_OFFLINE_H_
#define FASTCAT_ILD1900_OFFLINE_H_

// Include related header (for cc files)

// Include C then C++ libraries

// Include external then project includes
#include "fastcat/jsd/ild1900.h"

namespace fastcat
{
class Ild1900Offline : public Ild1900
{
 public:
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
};

}  // namespace fascat

#endif