#ifndef FASTCAT_PID_H_
#define FASTCAT_PID_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/device_base.h"

namespace fastcat
{
class Pid : public DeviceBase
{
 public:
  Pid();
  bool ConfigFromYaml(const YAML::Node& node) override;
  bool Read() override;
  bool Write(DeviceCmd& cmd) override;
  void Fault() override;

 protected:
  double kp_           = 0.0;
  double ki_           = 0.0;
  double kd_           = 0.0;
  double windup_limit_ = 0.0;

  PidActivateCmd pid_activate_cmd_ = {0};
  double         activation_time_  = 0.0;

  double  error_               = 0.0;
  double  prev_error_          = 0.0;
  double  integral_error_      = 0.0;
  uint8_t persistence_counter_ = 0;
};

}  // namespace fastcat

#endif
