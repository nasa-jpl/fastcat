// Include related headers
#include "fastcat/jsd/platinum_actuator_offline.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>

// Include external then project includes
#include "jsd/jsd.h"
#include "jsd/jsd_time.h"

fastcat::PlatinumActuatorOffline::PlatinumActuatorOffline()
{
  MSG_DEBUG("Constructed PlatinumActuatorOffline");
  memset(&jsd_epd_state_, 0, sizeof(jsd_epd_nominal_state_t));
  jsd_epd_state_.fault_code = JSD_EPD_FAULT_OKAY;
}

bool fastcat::PlatinumActuatorOffline::ConfigFromYaml(const YAML::Node& node)
{
  motor_on_start_time_ = initialization_time_sec_;
  return fastcat::PlatinumActuator::ConfigFromYaml(node);
}

bool fastcat::PlatinumActuatorOffline::HandleNewProfPosCmdImpl(
    const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->platinum_actuator_state.servo_enabled) {
    TransitionToState(ACTUATOR_SMS_PROF_POS_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  fastcat_trap_generate(
      &trap_, state_->time, state_->platinum_actuator_state.actual_position,
      ComputeTargetPosProfPosCmd(cmd),
      state_->platinum_actuator_state.cmd_velocity,
      cmd.actuator_prof_pos_cmd.end_velocity,
      cmd.actuator_prof_pos_cmd.profile_velocity,  // consider abs()
      cmd.actuator_prof_pos_cmd.profile_accel);

  TransitionToState(ACTUATOR_SMS_PROF_POS);

  return true;
}

bool fastcat::PlatinumActuatorOffline::HandleNewProfVelCmdImpl(
    const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->platinum_actuator_state.servo_enabled) {
    TransitionToState(ACTUATOR_SMS_PROF_VEL_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  fastcat_trap_generate_vel(&trap_, state_->time,
                            state_->platinum_actuator_state.actual_position,
                            state_->platinum_actuator_state.cmd_velocity,
                            cmd.actuator_prof_vel_cmd.target_velocity,
                            cmd.actuator_prof_vel_cmd.profile_accel,
                            cmd.actuator_prof_vel_cmd.max_duration);

  TransitionToState(ACTUATOR_SMS_PROF_VEL);

  return true;
}

bool fastcat::PlatinumActuatorOffline::HandleNewProfTorqueCmdImpl(
    const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->platinum_actuator_state.servo_enabled) {
    TransitionToState(ACTUATOR_SMS_PROF_TORQUE_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  fastcat_trap_generate_vel(&trap_, state_->time, 0, 0,
                            cmd.actuator_prof_torque_cmd.target_torque_amps,
                            params_.torque_slope_amps_per_sec,
                            cmd.actuator_prof_torque_cmd.max_duration);

  TransitionToState(ACTUATOR_SMS_PROF_TORQUE);

  return true;
}

fastcat::FaultType fastcat::PlatinumActuatorOffline::ProcessProfPos()
{
  return ProcessProfPosTrapImpl();
}

fastcat::FaultType fastcat::PlatinumActuatorOffline::ProcessProfVel()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_csv_t jsd_cmd;

  double pos_eu = 0.0;
  double vel = 0.0;
  int complete = fastcat_trap_update_vel(&trap_, state_->time, &pos_eu, &vel);

  jsd_cmd.target_velocity    = EuToCnts(vel);
  jsd_cmd.velocity_offset    = 0;
  jsd_cmd.torque_offset_amps = 0;

  if (!complete) {
    ElmoCSV(jsd_cmd);
  } else {
    jsd_cmd.target_velocity = 0;
    ElmoCSV(jsd_cmd);
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::PlatinumActuatorOffline::ProcessProfTorque()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_cst_t jsd_cmd;

  double dummy_pos_eu, current;
  int    complete =
      fastcat_trap_update_vel(&trap_, state_->time, &dummy_pos_eu, &current);

  jsd_cmd.target_torque_amps = current;
  jsd_cmd.torque_offset_amps = 0;

  ElmoCST(jsd_cmd);

  if (complete) {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::PlatinumActuatorOffline::ProcessProfPosDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->platinum_actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    fastcat_trap_generate(
        &trap_, state_->time, state_->platinum_actuator_state.actual_position,
        ComputeTargetPosProfPosCmd(last_cmd_),
        state_->platinum_actuator_state.cmd_velocity,
        last_cmd_.actuator_prof_pos_cmd.end_velocity,
        last_cmd_.actuator_prof_pos_cmd.profile_velocity,  // consider abs()
        last_cmd_.actuator_prof_pos_cmd.profile_accel);

    TransitionToState(ACTUATOR_SMS_PROF_POS);

  } else {
    // Otherwise, command the current position to trigger the transition and
    // wait

    jsd_elmo_motion_command_csp_t jsd_cmd;

    jsd_cmd.target_position =
        PosEuToCnts(state_->platinum_actuator_state.actual_position);
    jsd_cmd.position_offset    = 0;
    jsd_cmd.velocity_offset    = 0;
    jsd_cmd.torque_offset_amps = 0;

    ElmoCSP(jsd_cmd);

    // Check runout timer here, brake engage/disengage time cannot exceed 1
    // second per MAN-G-CR Section BP - Brake Parameters
    if ((state_->monotonic_time - last_transition_time_) >
        (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::PlatinumActuatorOffline::ProcessProfVelDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->platinum_actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    fastcat_trap_generate_vel(&trap_, state_->time,
                              state_->platinum_actuator_state.actual_position,
                              state_->platinum_actuator_state.cmd_velocity,
                              last_cmd_.actuator_prof_vel_cmd.target_velocity,
                              last_cmd_.actuator_prof_vel_cmd.profile_accel,
                              last_cmd_.actuator_prof_vel_cmd.max_duration);

    TransitionToState(ACTUATOR_SMS_PROF_VEL);

  } else {
    // Otherwise, command the current position to trigger the transition and
    // wait
    jsd_elmo_motion_command_csv_t jsd_cmd;

    jsd_cmd.target_velocity    = 0;
    jsd_cmd.velocity_offset    = 0;
    jsd_cmd.torque_offset_amps = 0;

    ElmoCSV(jsd_cmd);

    // Check runout timer here, brake engage/disengage time cannot exceed 1
    // second per MAN-G-CR Section BP - Brake Parameters
    if ((state_->monotonic_time - last_transition_time_) >
        (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType
fastcat::PlatinumActuatorOffline::ProcessProfTorqueDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->platinum_actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    fastcat_trap_generate_vel(
        &trap_, state_->time, 0, 0,
        last_cmd_.actuator_prof_torque_cmd.target_torque_amps,
        params_.torque_slope_amps_per_sec,
        last_cmd_.actuator_prof_torque_cmd.max_duration);

    TransitionToState(ACTUATOR_SMS_PROF_TORQUE);

  } else {
    // Otherwise, command the current position to trigger the transition and
    // wait
    jsd_elmo_motion_command_cst_t jsd_cmd;

    jsd_cmd.target_torque_amps = 0;
    jsd_cmd.torque_offset_amps = 0;

    ElmoCST(jsd_cmd);

    // Check runout timer here, brake engage/disengage time cannot exceed 1
    // second per MAN-G-CR Section BP - Brake Parameters
    if ((state_->monotonic_time - last_transition_time_) >
        (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

void fastcat::PlatinumActuatorOffline::ElmoSetConfig()
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoRead()
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoProcess()
{
  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
    case ACTUATOR_SMS_HALTED:
      jsd_epd_state_.motor_on      = 0;
      jsd_epd_state_.servo_enabled = 0;
      break;

    case ACTUATOR_SMS_HOLDING:
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_CSP:
    case ACTUATOR_SMS_CSV:
    case ACTUATOR_SMS_CST:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
    default:
      jsd_epd_state_.motor_on = 1;
      break;
  }

  // reset motor_on timer on rising edge
  if (!last_motor_on_state_ && jsd_epd_state_.motor_on) {
    motor_on_start_time_ = state_->time;
  }
  last_motor_on_state_ = jsd_epd_state_.motor_on;

  if (!jsd_epd_state_.servo_enabled && jsd_epd_state_.motor_on) {
    double brake_on_dur = state_->time - motor_on_start_time_;
    if (brake_on_dur > params_.elmo_brake_disengage_msec / 1000.0) {
      jsd_epd_state_.servo_enabled = 1;
    }
  }
}

void fastcat::PlatinumActuatorOffline::ElmoFault()
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoReset()
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoHalt()
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoClearErrors()
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoSetPeakCurrent(double /* current */)
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoSetUnitMode(int32_t  mode,
                                                       uint16_t app_id)
{
  MSG("Commanding new UM[1] = %d app_id = %u", mode, app_id);
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoSetGainSchedulingMode(
    jsd_elmo_gain_scheduling_mode_t /* mode */, uint16_t /* app_id */)
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoSetGainSchedulingIndex(
    uint16_t /* index */)
{
  // no-op
}

void fastcat::PlatinumActuatorOffline::ElmoCSP(
    const jsd_elmo_motion_command_csp_t& jsd_csp_cmd)
{
  jsd_epd_state_.cmd_position = jsd_csp_cmd.target_position;
  jsd_epd_state_.cmd_velocity = 0;
  jsd_epd_state_.cmd_current  = 0;

  jsd_epd_state_.cmd_ff_position = jsd_csp_cmd.position_offset;
  jsd_epd_state_.cmd_ff_velocity = jsd_csp_cmd.velocity_offset;
  jsd_epd_state_.cmd_ff_current  = jsd_csp_cmd.torque_offset_amps;

  // Differentiate position to get actual_velocity
  double vel = (jsd_epd_state_.cmd_position + jsd_epd_state_.cmd_ff_position -
                jsd_epd_state_.actual_position) /
               (state_->monotonic_time - last_monotonic_time_);

  // simulate actuals
  jsd_epd_state_.actual_position =
      jsd_epd_state_.cmd_position + jsd_epd_state_.cmd_ff_position;
  jsd_epd_state_.actual_current =
      jsd_epd_state_.cmd_current + jsd_epd_state_.cmd_ff_current;

  jsd_epd_state_.actual_velocity = vel;  // "sure, why not"
}

void fastcat::PlatinumActuatorOffline::ElmoCSV(
    const jsd_elmo_motion_command_csv_t& jsd_csv_cmd)
{
  jsd_epd_state_.cmd_position = 0;
  jsd_epd_state_.cmd_velocity = jsd_csv_cmd.target_velocity;
  jsd_epd_state_.cmd_current  = 0;

  jsd_epd_state_.cmd_ff_position = 0;
  jsd_epd_state_.cmd_ff_velocity = jsd_csv_cmd.velocity_offset;
  jsd_epd_state_.cmd_ff_current  = jsd_csv_cmd.torque_offset_amps;

  // simulate actuals
  jsd_epd_state_.actual_velocity =
      jsd_epd_state_.cmd_velocity + jsd_epd_state_.cmd_ff_velocity;
  jsd_epd_state_.actual_current =
      jsd_epd_state_.cmd_current + jsd_epd_state_.cmd_ff_current;
  jsd_epd_state_.actual_position +=
      jsd_epd_state_.actual_velocity * (state_->monotonic_time - last_monotonic_time_);  // integrated
}

void fastcat::PlatinumActuatorOffline::ElmoCST(
    const jsd_elmo_motion_command_cst_t& jsd_cst_cmd)
{
  jsd_epd_state_.cmd_position = 0;
  jsd_epd_state_.cmd_velocity = 0;
  jsd_epd_state_.cmd_current  = jsd_cst_cmd.target_torque_amps;

  jsd_epd_state_.cmd_ff_position = 0;
  jsd_epd_state_.cmd_ff_velocity = 0;
  jsd_epd_state_.cmd_ff_current  = jsd_cst_cmd.torque_offset_amps;

  // simulate actuals
  jsd_epd_state_.actual_position =
      jsd_epd_state_.cmd_position + jsd_epd_state_.cmd_ff_position;
  jsd_epd_state_.actual_current =
      jsd_epd_state_.cmd_current + jsd_epd_state_.cmd_ff_current;

  double pct =
      jsd_epd_state_.actual_current / params_.continuous_current_limit_amps;
  jsd_epd_state_.actual_velocity =
      pct * params_.max_speed_eu_per_sec;  // sure, why not
  jsd_epd_state_.actual_position +=
      jsd_epd_state_.actual_velocity * (state_->monotonic_time - last_monotonic_time_);  // integrated
}
