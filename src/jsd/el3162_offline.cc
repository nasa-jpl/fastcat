// Include related header (for cc files)
#include "fastcat/jsd/el3162_offline.h"

// Include c then c++ libraries

// Include external then project includes

bool fastcat::El3162Offline::ConfigFromYaml(const YAML::Node& node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El3162Offline::Read() { return true; }
