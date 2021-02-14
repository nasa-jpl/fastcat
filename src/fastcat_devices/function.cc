// Include related header (for cc files)
#include "fastcat/fastcat_devices/function.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Function::Function()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = FUNCTION_STATE;
  state_->time = std::chrono::steady_clock::now();
}

bool fastcat::Function::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "function_type", function_type_string_)) {
    return false;
  }

  // TODO if other function types are needed
  // function_type_ = FunctionTypeFromString(function_type_string_);

  if (function_type_string_.compare("POLYNOMIAL") != 0) {
    ERROR("Could not determind function type: %s",
          function_type_string_.c_str());
    return false;
  }
  function_type_ = POLYNOMIAL;

  if (!ParseVal(node, "order", order_)) {
    return false;
  }

  YAML::Node coeff_node;
  if (!ParseList(node, "coefficients", coeff_node)) {
    return false;
  }

  for (auto coeff = coeff_node.begin(); coeff != coeff_node.end(); ++coeff) {
    coefficients_.push_back((*coeff).as<double>());
  }

  if (order_ != static_cast<int>(coefficients_.size()) - 1) {
    ERROR("for a polynomial of %d-order, expecting %d coefficients. %lu found",
          order_, order_ + 1, coefficients_.size());
    return false;
  }
  // print the coefficients
  std::string coeff_str("y = ");
  for (int i = 0; i <= order_; ++i) {
    char term[64];
    if (i == order_) {
      snprintf(term, 64, "(%f)*x^%d ", coefficients_[i], order_ - i);
    } else {
      snprintf(term, 64, "(%f)*x^%d + ", coefficients_[i], order_ - i);
    }
    coeff_str.append(term);
  }
  MSG("Function: %s %s", name_.c_str(), coeff_str.c_str());

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly one signal for Function");
    return false;
  }

  return true;
}

bool fastcat::Function::Read()
{
  state_->time = std::chrono::steady_clock::now();

  // update input signal
  if (!UpdateSignal(signals_[0])) {
    ERROR("Could not extract signal");
    return false;
  }

  if (function_type_ == POLYNOMIAL) {
    state_->function_state.output = 0;
    for (int i = 0; i <= order_; ++i) {
      state_->function_state.output +=
          pow(signals_[0].value, order_ - i) * coefficients_[i];
    }
  } else {
    ERROR("Unhandled function_type");
    return false;
  }

  return true;
}
