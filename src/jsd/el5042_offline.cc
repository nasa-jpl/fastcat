// Include related header (for cc files)
#include "fastcat/jsd/el5042_offline.h"

// Include c then c++ libraries

// Include external then project includes

bool fastcat::El5042Offline::ConfigFromYaml(const YAML::Node& node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El5042Offline::Read() { return true; }
