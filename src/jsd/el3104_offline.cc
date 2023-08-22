// Include related header (for cc files)
#include "fastcat/jsd/el3104_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes

bool fastcat::El3104Offline::ConfigFromYaml(YAML::Node node,
                                            double /*external_time*/)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El3104Offline::Read() { return true; }
