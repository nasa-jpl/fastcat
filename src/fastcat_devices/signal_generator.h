#ifndef FASTCAT_SIGNAL_GENERATOR_H_
#define FASTCAT_SIGNAL_GENERATOR_H_

// Include related header (for cc files)

// Include c then c++ libraries
#include <random>

// Include external then project includes
#include "fastcat/device_base.h"
#include "fastcat/types.h"

namespace fastcat
{
enum SignalGeneratorType {
  SINE_WAVE,
  SAW_TOOTH,
  GAUSSIAN_RANDOM,
  UNIFORM_RANDOM,
  BAD_SIGNAL_GENERATOR_TYPE
};

SignalGeneratorType SignalGeneratorTypeFromString(const std::string&);

typedef struct {
  double angular_frequency;
  double phase;
  double amplitude;
  double offset;
} SineWaveParams;

typedef struct {
  double slope;
  double max;
  double min;
  double range;  // derived
  double modulo;
} SawToothParams;

typedef struct {
  uint32_t                         seed = 1;
  double                           mean;
  double                           sigma;
  std::normal_distribution<double> distribution;
} GaussianRandomParams;

typedef struct {
  uint32_t                               seed = 1;
  double                                 min;
  double                                 max;
  std::uniform_real_distribution<double> distribution;
} UniformRandomParams;

class SignalGenerator : public DeviceBase
{
 public:
  SignalGenerator();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;

 protected:
  std::string                signal_generator_type_string_;
  enum SignalGeneratorType   signal_generator_type_;
  double                     start_time_ = 0;
  std::default_random_engine generator_;

  union {
    SineWaveParams       sine_wave_;
    SawToothParams       saw_tooth_;
    GaussianRandomParams gaussian_random_;
    UniformRandomParams  uniform_random_;
  };
};

}  // namespace fastcat

#endif
