#ifndef FASTCAT_ACTUATOR_UTILS_H_
#define FASTCAT_ACTUATOR_UTILS_H_

// Include related header (for cc files)

// Include C then C++ libraries

// Include external then project includes
#include "fastcat/types.h"

namespace fastcat
{
namespace actuator_utils
{
std::string GetJSDFaultCodeAsString(const DeviceState& state);

std::string GetFastcatFaultCodeAsString(const DeviceState& state);

bool IsJsdFaultCodePresent(const DeviceState& state);

}  // namespace actuator_utils

}  // namespace fastcat

#endif