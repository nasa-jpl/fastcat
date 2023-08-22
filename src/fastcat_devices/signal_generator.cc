// Include related header (for cc files)
#include "fastcat/fastcat_devices/signal_generator.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::SignalGeneratorType fastcat::SignalGeneratorTypeFromString(
    const std::string& signal_generator_type)
{
  if (signal_generator_type.compare("SINE_WAVE") == 0) {
    return SINE_WAVE;
  } else if (signal_generator_type.compare("SAW_TOOTH") == 0) {
    return SAW_TOOTH;
    ;
  } else if (signal_generator_type.compare("GAUSSIAN_RANDOM") == 0) {
    return GAUSSIAN_RANDOM;
  } else if (signal_generator_type.compare("UNIFORM_RANDOM") == 0) {
    return UNIFORM_RANDOM;
  } else {
    return BAD_SIGNAL_GENERATOR_TYPE;
  }
}

fastcat::SignalGenerator::SignalGenerator()
{
  MSG_DEBUG("Constructed SignalGenerator");

  state_       = std::make_shared<DeviceState>();
  state_->type = SIGNAL_GENERATOR_STATE;
}

bool fastcat::SignalGenerator::ConfigFromYaml(YAML::Node node, double external_time)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "signal_generator_type", signal_generator_type_string_)) {
    return false;
  }

  signal_generator_type_ =
      SignalGeneratorTypeFromString(signal_generator_type_string_);
  switch (signal_generator_type_) {
    case SINE_WAVE: {
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
    }
    case SAW_TOOTH: {
      if (!ParseVal(node, "slope", saw_tooth_.slope)) {
        return false;
      }
      if (!ParseVal(node, "max", saw_tooth_.max)) {
        return false;
      }
      if (!ParseVal(node, "min", saw_tooth_.min)) {
        return false;
      }
      saw_tooth_.range  = saw_tooth_.max - saw_tooth_.min;
      saw_tooth_.modulo = 0;
      if (saw_tooth_.max < saw_tooth_.min) {
        ERROR("Sawtooth Max (%lf) is not > Min (%lf)", saw_tooth_.max,
              saw_tooth_.min);
        return false;
      }
      break;
    }
    case GAUSSIAN_RANDOM: {
      if (!ParseVal(node, "seed", gaussian_random_.seed)) {
        WARNING(
            "Key 'seed' not supplied, using default random number generator");
      } else {
        gaussian_random_.seed = 1;
      }
      if (!ParseVal(node, "mean", gaussian_random_.mean)) {
        return false;
      }
      if (!ParseVal(node, "sigma", gaussian_random_.sigma)) {
        return false;
      }
      generator_.seed(gaussian_random_.seed);
      gaussian_random_.distribution = std::normal_distribution<double>(
          gaussian_random_.mean, gaussian_random_.sigma);
      break;
    }
    case UNIFORM_RANDOM: {
      if (!ParseVal(node, "seed", uniform_random_.seed)) {
        WARNING(
            "Key 'seed' not supplied, using default random number generator");
      } else {
        uniform_random_.seed = 1;
      }
      if (!ParseVal(node, "min", uniform_random_.min)) {
        return false;
      }
      if (!ParseVal(node, "max", uniform_random_.max)) {
        return false;
      }
      generator_.seed(uniform_random_.seed);
      uniform_random_.distribution = std::uniform_real_distribution<double>(
          uniform_random_.min, uniform_random_.max);
      break;
    }
    default: {
      ERROR("signal_generator_type %s is invalid",
            signal_generator_type_string_.c_str());
      return false;
    }
  }
  return true;
}

bool fastcat::SignalGenerator::Read()
{
  switch (signal_generator_type_) {
    case SINE_WAVE: {
      state_->signal_generator_state.output =
          sine_wave_.offset +
          sine_wave_.amplitude *
              sin(sine_wave_.angular_frequency * state_->time +
                  sine_wave_.phase);
      break;
    }

    case SAW_TOOTH: {
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
      break;
    }

    case GAUSSIAN_RANDOM: {
      state_->signal_generator_state.output =
          gaussian_random_.distribution(generator_);
      break;
    }

    case UNIFORM_RANDOM: {
      state_->signal_generator_state.output =
          uniform_random_.distribution(generator_);
      break;
    }

    default: {
      ERROR("Unhandled signal_generator_type");
      return false;
    }
  }

  return true;
}
