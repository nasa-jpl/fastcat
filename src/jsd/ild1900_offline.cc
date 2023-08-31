// Include related header (for cc files)
#include "fastcat/jsd/ild1900_offline.h"

// Include C then C++ libraries

// Include external then project includes

bool fastcat::Ild1900Offline::ConfigFromYaml(const YAML::Node& node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::Ild1900Offline::Read() { return true; }
