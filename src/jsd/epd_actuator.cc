// Include related header (for cc files)
#include "fastcat/jsd/epd_actuator.h"

// Include C then C++ libraries
#include <cmath>
#include <cstring>

// Include external then project includes

fastcat::EpdActuator::EpdActuator() { state_->type = EPD_ACTUATOR_STATE; }

void fastcat::EpdActuator::PopulateJsdSlaveConfig()
{
  jsd_slave_config_.product_code = JSD_EPD_PRODUCT_CODE;

  jsd_slave_config_.epd.max_motor_speed   = EuToCnts(params_.max_speed_eu_per_sec);
  jsd_slave_config_.epd.loop_period_ms    = lround(loop_period_ * 1000.0);
  jsd_slave_config_.epd.torque_slope      = params_.torque_slope_amps_per_sec;
  jsd_slave_config_.epd.max_profile_accel = EuToCnts(params_.max_accel_eu_per_sec2);
  jsd_slave_config_.epd.max_profile_decel = EuToCnts(params_.max_accel_eu_per_sec2);
  jsd_slave_config_.epd.velocity_tracking_error =
      EuToCnts(params_.vel_tracking_error_eu_per_sec);
  jsd_slave_config_.epd.position_tracking_error =
      EuToCnts(params_.pos_tracking_error_eu);
  jsd_slave_config_.epd.peak_current_limit = params_.peak_current_limit_amps;
  jsd_slave_config_.epd.peak_current_time  = params_.peak_current_time_sec;
  jsd_slave_config_.epd.continuous_current_limit =
      params_.continuous_current_limit_amps;
  jsd_slave_config_.epd.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection
  jsd_slave_config_.epd.motor_stuck_velocity_threshold =
      0.0f;  // Motor stuck protection is disabled
  jsd_slave_config_.epd.motor_stuck_timeout =
      0.0f;  // Motor stuck protection is disabled
  jsd_slave_config_.epd.over_speed_threshold =
      params_.over_speed_multiplier * EuToCnts(params_.max_speed_eu_per_sec);
  jsd_slave_config_.epd.low_position_limit =
      0.0;  // Disable out of position limits protection
  jsd_slave_config_.epd.high_position_limit =
      0.0;  // Disable out of position limits protection
  jsd_slave_config_.epd.brake_engage_msec    = params_.elmo_brake_engage_msec;
  jsd_slave_config_.epd.brake_disengage_msec = params_.elmo_brake_disengage_msec;
  jsd_slave_config_.epd.crc                  = params_.elmo_crc;
  jsd_slave_config_.epd.smooth_factor        = params_.smooth_factor;
  jsd_slave_config_.epd.ctrl_gain_scheduling_mode = ctrl_gs_mode_;
}

void fastcat::EpdActuator::PopulateState()
{
  state_->epd_actuator_state.actual_position =
      PosCntsToEu(jsd_epd_state_.actual_position);
  state_->epd_actuator_state.actual_velocity =
      CntsToEu(jsd_epd_state_.actual_velocity);
  state_->epd_actuator_state.actual_current = jsd_epd_state_.actual_current;

  state_->epd_actuator_state.cmd_position =
      PosCntsToEu(jsd_epd_state_.cmd_position + jsd_epd_state_.cmd_ff_position);
  state_->epd_actuator_state.cmd_velocity =
      CntsToEu(jsd_epd_state_.cmd_velocity + jsd_epd_state_.cmd_ff_velocity);
  state_->epd_actuator_state.cmd_current =
      jsd_epd_state_.cmd_current + jsd_epd_state_.cmd_ff_current;

  state_->epd_actuator_state.cmd_prof_velocity =
      jsd_epd_state_.cmd_prof_velocity;
  state_->epd_actuator_state.cmd_prof_end_velocity =
      jsd_epd_state_.cmd_prof_end_velocity;
  state_->epd_actuator_state.cmd_prof_accel = jsd_epd_state_.cmd_prof_accel;

  state_->epd_actuator_state.cmd_max_current = jsd_epd_state_.cmd_max_current;

  state_->epd_actuator_state.elmo_state_machine_state =
      static_cast<uint32_t>(jsd_epd_state_.actual_state_machine_state);
  state_->epd_actuator_state.elmo_mode_of_operation =
      static_cast<uint32_t>(jsd_epd_state_.actual_mode_of_operation);

  state_->epd_actuator_state.sto_engaged    = jsd_epd_state_.sto_engaged;
  state_->epd_actuator_state.hall_state     = jsd_epd_state_.hall_state;
  state_->epd_actuator_state.target_reached = jsd_epd_state_.target_reached;
  state_->epd_actuator_state.setpoint_ack_rise =
      jsd_epd_state_.setpoint_ack_rise;
  state_->epd_actuator_state.motor_on      = jsd_epd_state_.motor_on;
  state_->epd_actuator_state.servo_enabled = jsd_epd_state_.servo_enabled;

  state_->epd_actuator_state.jsd_fault_code =
      static_cast<uint32_t>(jsd_epd_state_.fault_code);
  state_->epd_actuator_state.fastcat_fault_code =
      static_cast<uint32_t>(fastcat_fault_);
  state_->epd_actuator_state.emcy_error_code = jsd_epd_state_.emcy_error_code;
  state_->epd_actuator_state.faulted = (actuator_sms_ == ACTUATOR_SMS_FAULTED);

  state_->epd_actuator_state.bus_voltage = jsd_epd_state_.bus_voltage;
  state_->epd_actuator_state.drive_temperature =
      static_cast<uint32_t>(jsd_epd_state_.drive_temperature);

  state_->epd_actuator_state.elmo_actual_position =
      jsd_epd_state_.actual_position;
  state_->epd_actuator_state.elmo_cmd_position = jsd_epd_state_.cmd_position;

  state_->epd_actuator_state.actuator_state_machine_state =
      static_cast<uint32_t>(actuator_sms_);

  if (compute_power_) {
    state_->epd_actuator_state.power =
        ComputePower(state_->epd_actuator_state.actual_velocity,
                     state_->epd_actuator_state.actual_current,
                     state_->epd_actuator_state.motor_on);
  }
}

bool fastcat::EpdActuator::HandleNewProfPosCmdImpl(const DeviceCmd& cmd)
{
  // Save the command so that it can be sent continuously until the drive
  // acknowledges its reception.
  last_cmd_ = cmd;

  TransitionToState(ACTUATOR_SMS_PROF_POS_DISENGAGING);

  return true;
}

bool fastcat::EpdActuator::HandleNewProfVelCmdImpl(const DeviceCmd& cmd)
{
  // The corresponding JSD command should not be set here because it could be
  // missed by the JSD driver. This happens if the Fastcat's state machine is
  // HALTED when Fastcat handles a new Profiled command. In this case, Fastcat
  // requests a drive reset which indirectly sets the mode of operation to
  // Profiled Position for that cycle. Hence, the JSD command should be set when
  // *processing* the corresponding state in Fastcat's state machine so that the
  // mode of operation requested does not get overwritten in JSD.
  last_cmd_ = cmd;

  TransitionToState(ACTUATOR_SMS_PROF_VEL);

  return true;
}

bool fastcat::EpdActuator::HandleNewProfTorqueCmdImpl(const DeviceCmd& cmd)
{
  // The corresponding JSD command should not be set here because it could be
  // missed by the JSD driver. This happens if the Fastcat's state machine is
  // HALTED when Fastcat handles a new Profiled command. In this case, Fastcat
  // requests a drive reset which indirectly sets the mode of operation to
  // Profiled Position for that cycle. Hence, the JSD command should be set when
  // *processing* the corresponding state in Fastcat's state machine so that the
  // mode of operation requested does not get overwritten in JSD.
  last_cmd_ = cmd;

  TransitionToState(ACTUATOR_SMS_PROF_TORQUE);

  return true;
}

fastcat::FaultType fastcat::EpdActuator::ProcessProfPosDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_prof_pos_t jsd_cmd;
  jsd_cmd.relative = last_cmd_.actuator_prof_pos_cmd.relative;

  if (jsd_cmd.relative) {
    jsd_cmd.target_position =
        EuToCnts(last_cmd_.actuator_prof_pos_cmd.target_position);
  } else {
    jsd_cmd.target_position =
        PosEuToCnts(last_cmd_.actuator_prof_pos_cmd.target_position);
  }
  jsd_cmd.profile_velocity =
      EuToCnts(last_cmd_.actuator_prof_pos_cmd.profile_velocity);
  jsd_cmd.end_velocity = EuToCnts(last_cmd_.actuator_prof_pos_cmd.end_velocity);
  jsd_cmd.profile_accel =
      EuToCnts(last_cmd_.actuator_prof_pos_cmd.profile_accel);
  jsd_cmd.profile_decel =
      EuToCnts(last_cmd_.actuator_prof_pos_cmd.profile_accel);

  jsd_epd_set_motion_command_prof_pos((jsd_t*)context_, slave_id_, jsd_cmd);

  // Transition to ACTUATOR_SMS_PROF_POS once the drive acknowledges reception
  // of the command. Otherwise, the state variables used to check for the
  // completion of the command (i.e. target_reached) might refer to a previous
  // command. If the drive does not acknowledge the command within 1 second,
  // fault.
  if (state_->epd_actuator_state.setpoint_ack_rise) {
    TransitionToState(ACTUATOR_SMS_PROF_POS);
  } else if ((cycle_mono_time_ - last_transition_time_) >
             (1.0 + 2.0 * loop_period_)) {
    ERROR(
        "Act %s: Profiled Position command was not acknowledged by drive "
        "before timeout, faulting",
        name_.c_str());
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_PROF_POS_CMD_ACK_TIMEOUT_EXCEEDED;
    return ALL_DEVICE_FAULT;
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::EpdActuator::ProcessProfPos()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  // Transition to ACTUATOR_SMS_HOLDING once profile execution is complete if
  // the option to actively hold position is not on.
  if (state_->epd_actuator_state.target_reached && !params_.prof_pos_hold) {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::EpdActuator::ProcessProfVel()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_prof_vel_t jsd_cmd;
  // If max_duration is greater than zero, zero out the target after the
  // commanded duration.
  if (last_cmd_.actuator_prof_vel_cmd.max_duration > 1e-9 &&
      (cycle_mono_time_ - last_transition_time_) >
          last_cmd_.actuator_prof_vel_cmd.max_duration) {
    TransitionToState(ACTUATOR_SMS_HOLDING);

    jsd_cmd.target_velocity = 0;
    jsd_cmd.profile_accel =
        EuToCnts(last_cmd_.actuator_prof_vel_cmd.profile_accel);
    jsd_cmd.profile_decel =
        EuToCnts(last_cmd_.actuator_prof_vel_cmd.profile_accel);
  } else {
    jsd_cmd.target_velocity =
        EuToCnts(last_cmd_.actuator_prof_vel_cmd.target_velocity);
    jsd_cmd.profile_accel =
        EuToCnts(last_cmd_.actuator_prof_vel_cmd.profile_accel);
    jsd_cmd.profile_decel =
        EuToCnts(last_cmd_.actuator_prof_vel_cmd.profile_accel);
  }

  jsd_epd_set_motion_command_prof_vel((jsd_t*)context_, slave_id_, jsd_cmd);

  return NO_FAULT;
}

fastcat::FaultType fastcat::EpdActuator::ProcessProfTorque()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_prof_torque_t jsd_cmd;
  // If max_duration is greater than zero, zero out the target after the
  // commanded duration.
  if (last_cmd_.actuator_prof_torque_cmd.max_duration > 1e-9 &&
      (cycle_mono_time_ - last_transition_time_) >
          last_cmd_.actuator_prof_torque_cmd.max_duration) {
    TransitionToState(ACTUATOR_SMS_HOLDING);

    jsd_cmd.target_torque_amps = 0.0;
  } else {
    jsd_cmd.target_torque_amps =
        last_cmd_.actuator_prof_torque_cmd.target_torque_amps;
  }

  jsd_epd_set_motion_command_prof_torque((jsd_t*)context_, slave_id_, jsd_cmd);

  return NO_FAULT;
}

void fastcat::EpdActuator::ElmoRead()
{
  jsd_epd_read((jsd_t*)context_, slave_id_);
  memcpy(&jsd_epd_state_, jsd_epd_get_state((jsd_t*)context_, slave_id_),
         sizeof(jsd_epd_state_t));
}

void fastcat::EpdActuator::ElmoReset()
{
  jsd_epd_reset((jsd_t*)context_, slave_id_);
}

void fastcat::EpdActuator::ElmoSetPeakCurrent(double current)
{
  jsd_epd_set_peak_current((jsd_t*)context_, slave_id_, current);
}

void fastcat::EpdActuator::ElmoSetUnitMode(int32_t mode, uint16_t app_id)
{
  jsd_epd_async_sdo_set_unit_mode((jsd_t*)context_, slave_id_,
                                  static_cast<int16_t>(mode), app_id);
}

void fastcat::EpdActuator::ElmoSetGainSchedulingMode(
    jsd_elmo_gain_scheduling_mode_t mode, uint16_t app_id)
{
  jsd_epd_async_sdo_set_ctrl_gain_scheduling_mode((jsd_t*)context_, slave_id_,
                                                  mode, app_id);
}

void fastcat::EpdActuator::ElmoSetGainSchedulingIndex(uint16_t index)
{
  jsd_epd_set_gain_scheduling_index((jsd_t*)context_, slave_id_, true, index);
}

void fastcat::EpdActuator::ElmoCSP(
    const jsd_elmo_motion_command_csp_t& jsd_csp_cmd)
{
  jsd_epd_set_motion_command_csp((jsd_t*)context_, slave_id_, jsd_csp_cmd);
}

void fastcat::EpdActuator::ElmoCSV(
    const jsd_elmo_motion_command_csv_t& jsd_csv_cmd)
{
  jsd_epd_set_motion_command_csv((jsd_t*)context_, slave_id_, jsd_csv_cmd);
}

void fastcat::EpdActuator::ElmoCST(
    const jsd_elmo_motion_command_cst_t& jsd_cst_cmd)
{
  jsd_epd_set_motion_command_cst((jsd_t*)context_, slave_id_, jsd_cst_cmd);
}

void fastcat::EpdActuator::ElmoHalt()
{
  jsd_epd_halt((jsd_t*)context_, slave_id_);
}

void fastcat::EpdActuator::ElmoProcess()
{
  jsd_epd_process((jsd_t*)context_, slave_id_);
}

double fastcat::EpdActuator::GetActualVelocity()
{
  return state_->epd_actuator_state.actual_velocity;
}

double fastcat::EpdActuator::GetElmoActualPosition()
{
  return state_->epd_actuator_state.elmo_actual_position;
}

jsd_elmo_state_machine_state_t fastcat::EpdActuator::GetElmoStateMachineState()
{
  return static_cast<jsd_elmo_state_machine_state_t>(
      state_->epd_actuator_state.elmo_state_machine_state);
}

bool fastcat::EpdActuator::IsStoEngaged()
{
  return state_->epd_actuator_state.sto_engaged;
}
