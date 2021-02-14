#ifndef FASTCAT_SIGNAL_HANDLING_H_
#define FASTCAT_SIGNAL_HANDLING_H_

// Include related header (for cc files)

// Include c then c++ libraries
#include <string>

// Include external then project includes
#include <yaml-cpp/yaml.h>

#include "fastcat/types.h"

namespace fastcat
{
bool UpdateSignal(Signal& signal);

bool ConfigSignalsFromYaml(YAML::Node node, std::vector<Signal>& signals,
                           bool is_commander);

bool ConfigSignalByteIndexing(DeviceState* state, Signal& signal);

enum DeviceCmdType DeviceCmdTypeFromString(std::string str);

}  // namespace fastcat

#endif
