// Include related header (for cc files)
#include "fastcat/jsd/el1008_offline.h"

// Include c then c++ libraries

// Include external then project includes

bool fastcat::El1008Offline::ConfigFromYaml(YAML::Node node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El1008Offline::Read() { return true; }