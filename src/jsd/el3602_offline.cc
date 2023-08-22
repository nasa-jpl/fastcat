// Include related header (for cc files)
#include "fastcat/jsd/el3602_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes

bool fastcat::El3602Offline::ConfigFromYaml(YAML::Node node, double /*external_time*/)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El3602Offline::Read() { return true; }
