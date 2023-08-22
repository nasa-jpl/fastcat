// Include related header (for cc files)
#include "fastcat/fastcat_devices/saturation.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Saturation::Saturation()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = SATURATION_STATE;
}

bool fastcat::Saturation::ConfigFromYaml(YAML::Node node, double /*external_time*/)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "lower_limit", lower_limit_)) {
    return false;
  }

  if (!ParseVal(node, "upper_limit", upper_limit_)) {
    return false;
  }

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly one signal for Saturation");
    return false;
  }

  return true;
}

bool fastcat::Saturation::Read()
{
  // update input signal
  if (!UpdateSignal(signals_[0])) {
    ERROR("Could not extract signal");
    return false;
  }

  if (signals_[0].value > upper_limit_) {
    state_->saturation_state.output = upper_limit_;
  } else if (signals_[0].value < lower_limit_) {
    state_->saturation_state.output = lower_limit_;
  } else {
    state_->saturation_state.output = signals_[0].value;
  }

  return true;
}
