// Include related header (for cc files)
#include "fastcat/jsd/el3318_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes

bool fastcat::El3318Offline::ConfigFromYaml(YAML::Node node,
                                            double /*external_time*/)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El3318Offline::Read() { return true; }
