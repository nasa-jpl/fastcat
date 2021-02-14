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
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
  bool Write(DeviceCmd& cmd) override;
  void Fault() override;

 protected:
  double kp_;
  double ki_;
  double kd_;
  double windup_limit_;

  PidActivateCmd                        pid_activate_cmd_ = {0};
  std::chrono::steady_clock::time_point activation_time_;

  double  error_               = 0;
  double  prev_error_          = 0;
  double  integral_error_      = 0;
  uint8_t persistence_counter_ = 0;
};

}  // namespace fastcat

#endif
