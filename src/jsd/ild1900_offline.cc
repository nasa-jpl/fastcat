// Include related header (for cc files)
#include "fastcat/jsd/ild1900_offline.h"

// Include C then C++ libraries

// Include external then project includes

bool fastcat::Ild1900Offline::ConfigFromYaml(YAML::Node node, double /*external_time*/)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::Ild1900Offline::Read() { return true; }