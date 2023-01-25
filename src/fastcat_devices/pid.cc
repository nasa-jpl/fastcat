// Include related header (for cc files)
#include "fastcat/fastcat_devices/pid.h"

// Include c then c++ libraries
#include <cmath>

// Include external then project includes
#include "fastcat/signal_handling.h"
#include "fastcat/yaml_parser.h"
#include "jsd/jsd_print.h"

fastcat::Pid::Pid()
{
  state_       = std::make_shared<DeviceState>();
  state_->type = PID_STATE;
}

bool fastcat::Pid::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }
  state_->name = name_;

  if (!ParseVal(node, "kp", kp_)) {
    return false;
  }

  if (!ParseVal(node, "ki", kp_)) {
    return false;
  }

  if (!ParseVal(node, "kd", kp_)) {
    return false;
  }

  if (!ParseVal(node, "windup_limit", windup_limit_)) {
    return false;
  }

  if (!ConfigSignalsFromYaml(node, signals_, false)) {
    return false;
  }
  if (signals_.size() != 1) {
    ERROR("Expecting exactly one signal for Pid");
    return false;
  }

  return true;
}

bool fastcat::Pid::Read()
{
  if (!UpdateSignal(signals_[0])) {
    ERROR("Could not extract signal");
    return false;
  }

  if (state_->pid_state.active) {
    double active_time = state_->time - activation_time_;
    if (active_time > pid_activate_cmd_.max_duration) {
      MSG("PID controller reached max duration %lf sec, deactivating "
          "controller",
          pid_activate_cmd_.max_duration);
      state_->pid_state.active = false;
    }
  }

  error_ = pid_activate_cmd_.setpoint - signals_[0].value;

  if (state_->pid_state.active && fabs(error_) < pid_activate_cmd_.deadband) {
    persistence_counter_++;
    if ((persistence_counter_ * loop_period_) >
        pid_activate_cmd_.persistence_duration) {
      MSG("Pid controller converged, deactivating controller");
      state_->pid_state.active = false;
    }
  } else {
    persistence_counter_ = 0;
  }

  integral_error_ += error_ * loop_period_;
  if (integral_error_ > windup_limit_) {
    integral_error_ = windup_limit_;
  }

  if (state_->pid_state.active) {
    state_->pid_state.kp_term = kp_ * error_;
    state_->pid_state.ki_term = ki_ * integral_error_;
    state_->pid_state.kd_term = kd_ * (error_ - prev_error_) / loop_period_;

    state_->pid_state.output = state_->pid_state.kp_term +
                               state_->pid_state.ki_term +
                               state_->pid_state.kd_term;

  } else {
    state_->pid_state.kp_term = 0;
    state_->pid_state.ki_term = 0;
    state_->pid_state.kd_term = 0;

    state_->pid_state.output = 0;

    integral_error_ = 0;
  }
  prev_error_ = error_;

  return true;
}

bool fastcat::Pid::Write(DeviceCmd& cmd)
{
  if (cmd.type != PID_ACTIVATE_CMD) {
    WARNING("Bad command to Commander device");
    return false;
  }

  if(device_fault_active_){
    ERROR("Unable to Activate PID (%s) with an active fault, reset first", name_.c_str());
    return false;
  }

  state_->pid_state.active = true;
  pid_activate_cmd_        = cmd.pid_activate_cmd;
  activation_time_         = state_->time;

  return true;
}

void fastcat::Pid::Fault()
{
  state_->pid_state.active = false;
  DeviceBase::Fault();
}
