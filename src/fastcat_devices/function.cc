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
}

fastcat::FunctionType fastcat::FunctionTypeFromString(const std::string& function_type) {
  if(function_type.compare("POLYNOMIAL") != 0) {
    return POLYNOMIAL;
  } else if(function_type.compare("SUMMATION") != 0) {
    return SUMMATION;
  } else if(function_type.compare("MULTIPLICATION") != 0) {
    return MULTIPLICATION;
  } else if(function_type.compare("INVERSION") != 0) {
    return INVERSION;
  } else {
    return BAD_FUNCTION_TYPE;
  }
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
  
  function_type_ = fastcat::FunctionTypeFromString(function_type_string_);
  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
 
  switch(function_type_) {

    case POLYNOMIAL: { 

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
     
      if (signals_.size() != 1) {
        ERROR("Expecting exactly one signal for Function");
        return false;
      } 
    
      break;
    }

    case SUMMATION:    
      break;

    case MULTIPLICATION:
      break;

    case INVERSION: {
      if (signals_.size() != 1) {
        ERROR("Expecting exactly one signal for Function");
        return false;
      } 
      break;
    }

    case BAD_FUNCTION_TYPE: {

      ERROR("Could not determine function type: %s", 
        function_type_string_.c_str());

      return false;
    }
  }

  return true;
}

bool fastcat::Function::Read()
{
  for(auto& signal : signals_) {
    if (!UpdateSignal(signal)) {
      ERROR("Could not extract signal");
      return false;
    }
  }

  switch(function_type_) {
    case POLYNOMIAL: {
      state_->function_state.output = 0.0;
      for (int i = 0; i <= order_; ++i) {
        state_->function_state.output +=
            pow(signals_[0].value, order_ - i) * coefficients_[i];
      }
      break;
    }
    case SUMMATION: {
      state_->function_state.output = 0.0;
      for(auto& signal : signals_) {
        state_->function_state.output += signal.value;
      }
      break;
    }
    case MULTIPLICATION: {
      state_->function_state.output = 1.0;
      for(auto& signal : signals_) {
        state_->function_state.output *= signal.value;
      }
      break;
    }
    case INVERSION: {
      state_->function_state.output = 1.0 / signals_[0].value;
      break;
    }
    default: {
      ERROR("Unhandled function_type");
      return false;
    }
  }

  return true;

}
