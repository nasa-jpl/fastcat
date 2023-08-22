// Include related header (for cc files)
#include "fastcat/jsd/el3208_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"

bool fastcat::El3208Offline::ConfigFromYaml(YAML::Node node, double external_time)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El3208Offline::Read()
{
  memset(&state_->el3208_state, 0, sizeof(El3208State));

  return true;
}
