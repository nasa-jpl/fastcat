// Include related header (for cc files)
#include "fastcat/jsd/el3202_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>
#include <iostream>

// Include external then project includes

bool fastcat::El3202Offline::ConfigFromYaml(YAML::Node node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::El3202Offline::Read()
{
  state_->time = std::chrono::steady_clock::now();
  return true;
}
