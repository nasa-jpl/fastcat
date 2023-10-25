// Include related header (for cc files)
#include "fastcat/jsd/actuator_utils.h"

// Include C then C++ libraries
#include <cassert>

// Include external then project includes
#include "fastcat/jsd/actuator.h"
#include "fastcat/jsd/platinum_sil_actuator.h"
#include "jsd/jsd_egd_pub.h"
#include "jsd/jsd_epd_nominal_pub.h"
#include "jsd/jsd_epd_sil_pub.h"

std::string fastcat::actuator_utils::GetJSDFaultCodeAsString(
    const DeviceState& state)
{
  std::string fault_str;

  if (state.type == GOLD_ACTUATOR_STATE) {
    auto fault = static_cast<jsd_egd_fault_code_t>(
        state.gold_actuator_state.jsd_fault_code);
    fault_str = std::string(jsd_egd_fault_code_to_string(fault));
  } else if (state.type == PLATINUM_ACTUATOR_STATE) {
    auto fault = static_cast<jsd_epd_fault_code_t>(
        state.platinum_actuator_state.jsd_fault_code);
    fault_str = std::string(jsd_epd_nominal_fault_code_to_string(fault));
  } else if (state.type == PLATINUM_SIL_ACTUATOR_STATE) {
    auto fault = static_cast<jsd_epd_fault_code_t>(
        state.platinum_sil_actuator_state.jsd_fault_code);
    fault_str = std::string(jsd_epd_sil_fault_code_to_string(fault));
  } else {
    fault_str = "State is not an Elmo actuator type.";
  }

  return fault_str;
}

std::string fastcat::actuator_utils::GetFastcatFaultCodeAsString(
    const DeviceState& state)
{
  if (state.type == GOLD_ACTUATOR_STATE ||
      state.type == PLATINUM_ACTUATOR_STATE) {
    ActuatorFastcatFault fault;
    if (state.type == GOLD_ACTUATOR_STATE) {
      fault = static_cast<ActuatorFastcatFault>(
          state.gold_actuator_state.fastcat_fault_code);
    } else {
      fault = static_cast<ActuatorFastcatFault>(
          state.platinum_actuator_state.fastcat_fault_code);
    }
    return Actuator::FastcatFaultToString(fault);
  } else if (state.type == PLATINUM_SIL_ACTUATOR_STATE) {
    return PlatinumSilActuator::FastcatFaultToString(
        static_cast<PlatinumSilActuatorFastcatFault>(
            state.platinum_sil_actuator_state.fastcat_fault_code));
  } else {
    return "State is not an Elmo actuator type.";
  }
}

bool fastcat::actuator_utils::IsJsdFaultCodePresent(const DeviceState& state)
{
  bool fault_present = false;

  if (state.type == GOLD_ACTUATOR_STATE) {
    if (state.gold_actuator_state.jsd_fault_code != JSD_EGD_FAULT_OKAY) {
      fault_present = true;
    }
  } else if (state.type == PLATINUM_ACTUATOR_STATE) {
    if (state.platinum_actuator_state.jsd_fault_code != JSD_EPD_FAULT_OKAY) {
      fault_present = true;
    }
  } else if (state.type == PLATINUM_SIL_ACTUATOR_STATE) {
    if (state.platinum_sil_actuator_state.jsd_fault_code !=
        JSD_EPD_FAULT_OKAY) {
      fault_present = true;
    }
  } else {
    ERROR(
        "IsJsdFaultCodePresent must be called on states of Elmo actuator "
        "type.");
    assert(false);
  }

  return fault_present;
}