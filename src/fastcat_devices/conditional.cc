// Include related header (for cc files)
#include "fastcat/fastcat_devices/conditional.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::ConditionalType fastcat::ConditionalTypeFromString(
    std::string cond_type)
{
  ConditionalType type;

  if (cond_type.compare("<") == 0) {
    type = LT;
  } else if (cond_type.compare("<=") == 0) {
    type = LE;
  } else if (cond_type.compare(">") == 0) {
    type = GT;
  } else if (cond_type.compare(">=") == 0) {
    type = GE;
  } else if (cond_type.compare("==") == 0) {
    type = EQ;
  } else if (cond_type.compare("!=") == 0) {
    type = NE;

  } else {
    type = BAD_CONDITIONAL_TYPE;
    ERROR("%s is not a known ConditionalType", cond_type.c_str());
  }

  return type;
}

fastcat::Conditional::Conditional()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = CONDITIONAL_STATE;
}

bool fastcat::Conditional::ConfigFromYaml(YAML::Node node, double /*external_time*/)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "conditional_type", conditional_type_string_)) {
    return false;
  }

  conditional_type_ = ConditionalTypeFromString(conditional_type_string_);
  if (conditional_type_ == BAD_CONDITIONAL_TYPE) {
    return false;
  }

  if (!ParseVal(node, "compare_rhs_value", compare_rhs_value_)) {
    return false;
  }

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly one signal for Conditional");
    return false;
  }

  return true;
}

bool fastcat::Conditional::Read()
{
  // update input signal
  if (!UpdateSignal(signals_[0])) {
    ERROR("Could not extract signal");
    return false;
  }

  double signal_value = signals_[0].value;

  switch (conditional_type_) {
    case LT:
      state_->conditional_state.output = signal_value < compare_rhs_value_;
      break;
    case LE:
      state_->conditional_state.output = signal_value <= compare_rhs_value_;
      break;
    case GT:
      state_->conditional_state.output = signal_value > compare_rhs_value_;
      break;
    case GE:
      state_->conditional_state.output = signal_value >= compare_rhs_value_;
      break;
    case EQ:
      state_->conditional_state.output = signal_value == compare_rhs_value_;
      break;
    case NE:
      state_->conditional_state.output = signal_value != compare_rhs_value_;
      break;
    default:
      ERROR("Unhandled conditional type");
      return false;
  }
  return true;
}
