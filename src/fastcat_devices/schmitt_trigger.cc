// Include related header (for cc files)
#include "fastcat/fastcat_devices/schmitt_trigger.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::SchmittTrigger::SchmittTrigger()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = SCHMITT_TRIGGER_STATE;
}

bool fastcat::SchmittTrigger::ConfigFromYaml(YAML::Node node, double /*external_time*/)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "low_threshold", low_threshold_)) {
    return false;
  }

  if (!ParseVal(node, "high_threshold", high_threshold_)) {
    return false;
  }

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly 1 signal for Schmitt Trigger");
    return false;
  }

  return true;
}

bool fastcat::SchmittTrigger::Read()
{
  if (!UpdateSignal(signals_[0])) {
    return false;
  }

  if (state_->schmitt_trigger_state.output > 0) {
    if (signals_[0].value < low_threshold_) {
      state_->schmitt_trigger_state.output = 0;
    }
  } else {
    if (signals_[0].value > high_threshold_) {
      state_->schmitt_trigger_state.output = 1;
    }
  }

  return true;
}
