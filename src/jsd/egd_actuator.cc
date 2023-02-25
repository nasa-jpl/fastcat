// Include related header (for cc files)
#include "fastcat/jsd/egd_actuator.h"

// Include C then C++ libraries
#include <cstring>

// Include external then project includes
#include "jsd/jsd.h"
#include "jsd/jsd_egd_pub.h"

void fastcat::EgdActuator::PopulateJsdSlaveConfig()
{
  jsd_slave_config_.product_code = JSD_EGD_PRODUCT_CODE;

  jsd_slave_config_.egd.drive_cmd_mode    = JSD_EGD_DRIVE_CMD_MODE_CS;
  jsd_slave_config_.egd.max_motor_speed   = EuToCnts(max_speed_eu_per_sec_);
  jsd_slave_config_.egd.loop_period_ms    = loop_period_ * 1000.0;
  jsd_slave_config_.egd.torque_slope      = torque_slope_amps_per_sec_;
  jsd_slave_config_.egd.max_profile_accel = EuToCnts(max_accel_eu_per_sec2_);
  jsd_slave_config_.egd.max_profile_decel = EuToCnts(max_accel_eu_per_sec2_);
  jsd_slave_config_.egd.velocity_tracking_error =
      EuToCnts(vel_tracking_error_eu_per_sec_);
  jsd_slave_config_.egd.position_tracking_error =
      EuToCnts(pos_tracking_error_eu_);
  jsd_slave_config_.egd.peak_current_limit = peak_current_limit_amps_;
  jsd_slave_config_.egd.peak_current_time  = peak_current_time_sec_;
  jsd_slave_config_.egd.continuous_current_limit =
      continuous_current_limit_amps_;
  jsd_slave_config_.egd.motor_stuck_current_level_pct  = 0;  // disable
  jsd_slave_config_.egd.motor_stuck_velocity_threshold = 0;  // disable
  jsd_slave_config_.egd.motor_stuck_timeout            = 0;  // disable
  jsd_slave_config_.egd.over_speed_threshold =
      over_speed_multiplier_ * EuToCnts(max_speed_eu_per_sec_);
  jsd_slave_config_.egd.low_position_limit   = 0;  // disable
  jsd_slave_config_.egd.high_position_limit  = 0;  // disable
  jsd_slave_config_.egd.brake_engage_msec    = elmo_brake_engage_msec_;
  jsd_slave_config_.egd.brake_disengage_msec = elmo_brake_disengage_msec_;
  jsd_slave_config_.egd.crc                  = elmo_crc_;
  jsd_slave_config_.egd.drive_max_current_limit =
      elmo_drive_max_cur_limit_amps_;
  jsd_slave_config_.egd.smooth_factor = smooth_factor_;
}

void fastcat::EgdActuator::ElmoRead()
{
  jsd_egd_read((jsd_t*)context_, slave_id_);
  memcpy(&jsd_egd_state_, jsd_egd_get_state((jsd_t*)context_, slave_id_),
         sizeof(jsd_egd_state_t));
}

void fastcat::EgdActuator::PopulateState()
{
  state_->actuator_state.egd_actual_position = jsd_egd_state_.actual_position;
  state_->actuator_state.egd_cmd_position    = jsd_egd_state_.cmd_position;

  state_->actuator_state.actual_position =
      PosCntsToEu(jsd_egd_state_.actual_position);
  state_->actuator_state.actual_velocity =
      CntsToEu(jsd_egd_state_.actual_velocity);
  state_->actuator_state.actual_current = jsd_egd_state_.actual_current;

  state_->actuator_state.cmd_position =
      PosCntsToEu(jsd_egd_state_.cmd_position + jsd_egd_state_.cmd_ff_position);
  state_->actuator_state.cmd_velocity =
      CntsToEu(jsd_egd_state_.cmd_velocity + jsd_egd_state_.cmd_ff_velocity);
  state_->actuator_state.cmd_current =
      (jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current);

  state_->actuator_state.cmd_max_current = jsd_egd_state_.cmd_max_current;

  state_->actuator_state.egd_state_machine_state =
      static_cast<uint32_t>(jsd_egd_state_.actual_state_machine_state);
  state_->actuator_state.egd_mode_of_operation =
      static_cast<uint32_t>(jsd_egd_state_.actual_mode_of_operation);

  state_->actuator_state.sto_engaged    = jsd_egd_state_.sto_engaged;
  state_->actuator_state.hall_state     = jsd_egd_state_.hall_state;
  state_->actuator_state.target_reached = jsd_egd_state_.target_reached;
  state_->actuator_state.motor_on       = jsd_egd_state_.motor_on;
  state_->actuator_state.servo_enabled  = jsd_egd_state_.servo_enabled;

  state_->actuator_state.bus_voltage       = jsd_egd_state_.bus_voltage;
  state_->actuator_state.drive_temperature = jsd_egd_state_.drive_temperature;

  state_->actuator_state.actuator_state_machine_state =
      static_cast<uint32_t>(actuator_sms_);

  state_->actuator_state.fastcat_fault_code =
      static_cast<uint32_t>(fastcat_fault_);
  state_->actuator_state.jsd_fault_code =
      static_cast<uint32_t>(jsd_egd_state_.fault_code);
  state_->actuator_state.emcy_error_code = jsd_egd_state_.emcy_error_code;
  state_->actuator_state.faulted = (actuator_sms_ == ACTUATOR_SMS_FAULTED);
}

void fastcat::EgdActuator::ElmoClearErrors()
{
  jsd_egd_clear_errors((jsd_t*)context_, slave_id_);
}

void fastcat::EgdActuator::ElmoSetPeakCurrent(double current)
{
  jsd_egd_set_peak_current((jsd_t*)context_, slave_id_, current);
}

void fastcat::EgdActuator::ElmoSetUnitMode(int32_t mode, uint16_t app_id)
{
  MSG("Commanding new UM[1] = %d app_id = %u", mode, app_id);
  jsd_egd_async_sdo_set_unit_mode((jsd_t*)context_, slave_id_, mode, app_id);
}

void fastcat::EgdActuator::ElmoSetGainSchedulingMode(
    jsd_elmo_gain_scheduling_mode_t mode, uint16_t app_id)
{
  jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode((jsd_t*)context_, slave_id_,
                                                  mode, app_id);
}

void fastcat::EgdActuator::ElmoSetGainSchedulingIndex(uint16_t index)
{
  jsd_egd_set_gain_scheduling_index((jsd_t*)context_, slave_id_, true, index);
}

void fastcat::EgdActuator::ElmoReset()
{
  MSG("Resetting EGD through JSD: %s", name_.c_str());
  jsd_egd_reset((jsd_t*)context_, slave_id_);
}

void fastcat::EgdActuator::ElmoCSP(
    const jsd_elmo_motion_command_csp_t& jsd_csp_cmd)
{
  jsd_egd_set_motion_command_csp((jsd_t*)context_, slave_id_, jsd_csp_cmd);
}

void fastcat::EgdActuator::ElmoCSV(
    const jsd_elmo_motion_command_csv_t& jsd_csv_cmd)
{
  jsd_egd_set_motion_command_csv((jsd_t*)context_, slave_id_, jsd_csv_cmd);
}

void fastcat::EgdActuator::ElmoCST(
    const jsd_elmo_motion_command_cst_t& jsd_cst_cmd)
{
  jsd_egd_set_motion_command_cst((jsd_t*)context_, slave_id_, jsd_cst_cmd);
}

void fastcat::EgdActuator::ElmoHalt()
{
  jsd_egd_halt((jsd_t*)context_, slave_id_);
}

void fastcat::EgdActuator::ElmoProcess()
{
  jsd_egd_process((jsd_t*)context_, slave_id_);
}

bool fastcat::EgdActuator::HandleNewProfPosCmdImpl(const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->actuator_state.servo_enabled) {
    TransitionToState(ACTUATOR_SMS_PROF_POS_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  trap_generate(&trap_, state_->time, state_->actuator_state.actual_position,
                ComputeTargetPosProfPosCmd(cmd),
                state_->actuator_state.cmd_velocity,
                cmd.actuator_prof_pos_cmd.end_velocity,
                cmd.actuator_prof_pos_cmd.profile_velocity,  // consider abs()
                cmd.actuator_prof_pos_cmd.profile_accel);

  TransitionToState(ACTUATOR_SMS_PROF_POS);

  return true;
}

bool fastcat::EgdActuator::HandleNewProfVelCmdImpl(const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->actuator_state.servo_enabled) {
    TransitionToState(ACTUATOR_SMS_PROF_VEL_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  trap_generate_vel(&trap_, state_->time,
                    state_->actuator_state.actual_position,
                    state_->actuator_state.cmd_velocity,
                    cmd.actuator_prof_vel_cmd.target_velocity,
                    cmd.actuator_prof_vel_cmd.profile_accel,
                    cmd.actuator_prof_vel_cmd.max_duration);

  TransitionToState(ACTUATOR_SMS_PROF_VEL);

  return true;
}

bool fastcat::EgdActuator::HandleNewProfTorqueCmdImpl(const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->actuator_state.servo_enabled) {
    TransitionToState(ACTUATOR_SMS_PROF_TORQUE_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  trap_generate_vel(&trap_, state_->time, 0, 0,
                    cmd.actuator_prof_torque_cmd.target_torque_amps,
                    torque_slope_amps_per_sec_,
                    cmd.actuator_prof_torque_cmd.max_duration);

  TransitionToState(ACTUATOR_SMS_PROF_TORQUE);

  return true;
}

fastcat::FaultType fastcat::EgdActuator::ProcessProfPos()
{
  return ProcessProfPosTrapImpl();
}

fastcat::FaultType fastcat::EgdActuator::ProcessProfVel()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_csv_t jsd_cmd;

  double pos_eu, vel;
  int    complete = trap_update_vel(&trap_, state_->time, &pos_eu, &vel);

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

fastcat::FaultType fastcat::EgdActuator::ProcessProfTorque()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_cst_t jsd_cmd;

  double dummy_pos_eu, current;
  int complete = trap_update_vel(&trap_, state_->time, &dummy_pos_eu, &current);

  jsd_cmd.target_torque_amps = current;
  jsd_cmd.torque_offset_amps = 0;

  ElmoCST(jsd_cmd);

  if (complete) {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::EgdActuator::ProcessProfPosDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    trap_generate(
        &trap_, state_->time, state_->actuator_state.actual_position,
        ComputeTargetPosProfPosCmd(last_cmd_),
        state_->actuator_state.cmd_velocity,
        last_cmd_.actuator_prof_pos_cmd.end_velocity,
        last_cmd_.actuator_prof_pos_cmd.profile_velocity,  // consider abs()
        last_cmd_.actuator_prof_pos_cmd.profile_accel);

    TransitionToState(ACTUATOR_SMS_PROF_POS);

  } else {
    // Otherwise, command the current position to trigger the transition and
    // wait

    jsd_elmo_motion_command_csp_t jsd_cmd;

    jsd_cmd.target_position =
        PosEuToCnts(state_->actuator_state.actual_position);
    jsd_cmd.position_offset    = 0;
    jsd_cmd.velocity_offset    = 0;
    jsd_cmd.torque_offset_amps = 0;

    ElmoCSP(jsd_cmd);

    // Check runout timer here, brake engage/disengage time cannot exceed 1
    // second per MAN-G-CR Section BP - Brake Parameters
    if ((jsd_time_get_time_sec() - last_transition_time_) >
        (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::EgdActuator::ProcessProfVelDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    trap_generate_vel(&trap_, state_->time,
                      state_->actuator_state.actual_position,
                      state_->actuator_state.cmd_velocity,
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
    if ((jsd_time_get_time_sec() - last_transition_time_) >
        (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::EgdActuator::ProcessProfTorqueDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    trap_generate_vel(&trap_, state_->time, 0, 0,
                      last_cmd_.actuator_prof_torque_cmd.target_torque_amps,
                      torque_slope_amps_per_sec_,
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
    if ((jsd_time_get_time_sec() - last_transition_time_) >
        (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}