// Include related header (for cc files)
#include "fastcat/fastcat_devices/signal_generator.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::SignalGenerator::SignalGenerator()
{
  MSG_DEBUG("Constructed SignalGenerator");

  state_       = std::make_shared<DeviceState>();
  state_->type = SIGNAL_GENERATOR_STATE;
}

bool fastcat::SignalGenerator::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "signal_generator_type", signal_generator_type_string_)) {
    return false;
  }

  if (signal_generator_type_string_.compare("SINE_WAVE") == 0) {
    signal_generator_type_ = SINE_WAVE;

  } else if (signal_generator_type_string_.compare("SAW_TOOTH") == 0) {
    signal_generator_type_ = SAW_TOOTH;

  } else {
    signal_generator_type_ = BAD_SIGNAL_GENERATOR_TYPE;
    ERROR("signal_generator_type %s is invalid",
          signal_generator_type_string_.c_str());
    return false;
  }

  switch (signal_generator_type_) {
    case SINE_WAVE:

      if (!ParseVal(node, "angular_frequency", sine_wave_.angular_frequency)) {
        return false;
      }

      if (!ParseVal(node, "phase", sine_wave_.phase)) {
        return false;
      }

      if (!ParseVal(node, "amplitude", sine_wave_.amplitude)) {
        return false;
      }

      if (!ParseVal(node, "offset", sine_wave_.offset)) {
        return false;
      }
      break;

    case SAW_TOOTH:

      if (!ParseVal(node, "slope", saw_tooth_.slope)) {
        return false;
      }

      if (!ParseVal(node, "max", saw_tooth_.max)) {
        return false;
      }
      if (!ParseVal(node, "min", saw_tooth_.min)) {
        return false;
      }

      saw_tooth_.range = saw_tooth_.max - saw_tooth_.min;

      saw_tooth_.modulo = 0;

      if (saw_tooth_.max < saw_tooth_.min) {
        ERROR("Sawtooth Max (%lf) is not > Min (%lf)", saw_tooth_.max,
              saw_tooth_.min);
        return false;
      }

      break;

    default:
      ERROR("bad signal generator type");
      return false;
  }

  return true;
}

bool fastcat::SignalGenerator::Read()
{

  if (signal_generator_type_ == SINE_WAVE) {
    state_->signal_generator_state.output =
        sine_wave_.offset +
        sine_wave_.amplitude *
            sin(sine_wave_.angular_frequency * state_->time + sine_wave_.phase);
  } else if (signal_generator_type_ == SAW_TOOTH) {
    // positive slopes
    if (saw_tooth_.slope >= 0) {
      state_->signal_generator_state.output =
          saw_tooth_.min + saw_tooth_.slope * state_->time -
          saw_tooth_.range * saw_tooth_.modulo;

      if (state_->signal_generator_state.output > saw_tooth_.max) {
        saw_tooth_.modulo += 1.0;

        state_->signal_generator_state.output =
            saw_tooth_.min + saw_tooth_.slope * state_->time -
            saw_tooth_.range * saw_tooth_.modulo;
      }
      // negative slopes
    } else {
      state_->signal_generator_state.output =
          saw_tooth_.max + saw_tooth_.slope * state_->time +
          saw_tooth_.range * saw_tooth_.modulo;

      if (state_->signal_generator_state.output < saw_tooth_.min) {
        saw_tooth_.modulo += 1.0;

        state_->signal_generator_state.output =
            saw_tooth_.max + saw_tooth_.slope * state_->time +
            saw_tooth_.range * saw_tooth_.modulo;
      }
    }

  } else {
    ERROR("Unhandled signal_generator_type");
    return false;
  }

  return true;
}
