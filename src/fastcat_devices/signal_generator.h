#ifndef FASTCAT_SIGNAL_GENERATOR_H_
#define FASTCAT_SIGNAL_GENERATOR_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"
#include "fastcat/types.h"

namespace fastcat
{
enum SignalGeneratorType { SINE_WAVE, SAW_TOOTH, BAD_SIGNAL_GENERATOR_TYPE };

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

class SignalGenerator : public DeviceBase
{
 public:
  SignalGenerator();
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;

 protected:
  std::string                           signal_generator_type_string_;
  enum SignalGeneratorType              signal_generator_type_;
  std::chrono::steady_clock::time_point start_time_;

  union {
    SineWaveParams sine_wave_;
    SawToothParams saw_tooth_;
  };
};

}  // namespace fastcat

#endif
