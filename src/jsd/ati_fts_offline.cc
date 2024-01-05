// Include related header (for cc files)
#include "fastcat/jsd/ati_fts_offline.h"

// Include c then c++ libraries

// Include external then project includes

bool fastcat::AtiFtsOffline::ConfigFromYaml(const YAML::Node& node)
{
  return ConfigFromYamlCommon(node);
}

bool fastcat::AtiFtsOffline::Read() { return true; }

fastcat::FaultType fastcat::AtiFtsOffline::Process() { return NO_FAULT; }
