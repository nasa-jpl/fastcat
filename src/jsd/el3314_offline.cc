// Include related header (for cc files)
#include "fastcat/jsd/el3314_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes

bool fastcat::El3314Offline::ConfigFromYaml(const YAML::Node& node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El3314Offline::Read() { return true; }
