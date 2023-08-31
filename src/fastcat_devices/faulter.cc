// Include related header (for cc files)
#include "fastcat/fastcat_devices/faulter.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Faulter::Faulter()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = FAULTER_STATE;
}

bool fastcat::Faulter::ConfigFromYaml(const YAML::Node& node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "start_enabled", start_enabled_)) {
    return false;
  }

  state_->faulter_state.enable = start_enabled_;

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly one signal for Faulter");
    return false;
  }
  return true;
}

bool fastcat::Faulter::Read()
{
  // update input signal
  if (!UpdateSignal(signals_[0])) {
    ERROR("Could not extract signal");
    return false;
  }
  state_->faulter_state.fault_active = (fabs(signals_[0].value) > 0);

  return true;
}

fastcat::FaultType fastcat::Faulter::Process()
{
  if (state_->faulter_state.fault_active && state_->faulter_state.enable) {
    return ALL_DEVICE_FAULT;
  }

  return NO_FAULT;
}

bool fastcat::Faulter::Write(DeviceCmd& cmd)
{
  if (cmd.type != FAULTER_ENABLE_CMD) {
    WARNING("Bad command to Faulter device");
    return false;
  }
  state_->faulter_state.enable = cmd.faulter_enable_cmd.enable;

  return true;
}

void fastcat::Faulter::Fault()
{
  DeviceBase::Fault();
  state_->faulter_state.enable = false;
}

void fastcat::Faulter::Reset()
{
  DeviceBase::Reset();
  if (start_enabled_) {
    state_->faulter_state.enable = true;
  }
}
