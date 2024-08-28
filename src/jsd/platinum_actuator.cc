// Include related header (for cc files)
#include "fastcat/jsd/platinum_actuator.h"

// Include C then C++ libraries
#include <cmath>
#include <cstring>

// Include external then project includes

fastcat::PlatinumActuator::PlatinumActuator()
{
  state_->type = PLATINUM_ACTUATOR_STATE;
}

void fastcat::PlatinumActuator::PopulateJsdSlaveConfig()
{
  jsd_slave_config_.driver_type = JSD_DRIVER_TYPE_EPD_NOMINAL;

  jsd_slave_config_.epd_nominal.max_motor_speed =
      EuToCnts(params_.max_speed_eu_per_sec);
  jsd_slave_config_.epd_nominal.loop_period_ms = lround(loop_period_ * 1000.0);
  jsd_slave_config_.epd_nominal.torque_slope =
      params_.torque_slope_amps_per_sec;
  jsd_slave_config_.epd_nominal.max_profile_accel =
      EuToCnts(params_.max_accel_eu_per_sec2);
  jsd_slave_config_.epd_nominal.max_profile_decel =
      EuToCnts(params_.max_accel_eu_per_sec2);
  jsd_slave_config_.epd_nominal.velocity_tracking_error =
      EuToCnts(params_.vel_tracking_error_eu_per_sec);
  jsd_slave_config_.epd_nominal.position_tracking_error =
      EuToCnts(params_.pos_tracking_error_eu);
  jsd_slave_config_.epd_nominal.peak_current_limit =
      params_.peak_current_limit_amps;
  jsd_slave_config_.epd_nominal.peak_current_time =
      params_.peak_current_time_sec;
  jsd_slave_config_.epd_nominal.continuous_current_limit =
      params_.continuous_current_limit_amps;
  jsd_slave_config_.epd_nominal.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection
  jsd_slave_config_.epd_nominal.motor_stuck_velocity_threshold =
      0.0f;  // Motor stuck protection is disabled
  jsd_slave_config_.epd_nominal.motor_stuck_timeout =
      0.0f;  // Motor stuck protection is disabled
  jsd_slave_config_.epd_nominal.over_speed_threshold =
      params_.over_speed_multiplier * EuToCnts(params_.max_speed_eu_per_sec);
  jsd_slave_config_.epd_nominal.low_position_limit =
      0.0;  // Disable out of position limits protection
  jsd_slave_config_.epd_nominal.high_position_limit =
      0.0;  // Disable out of position limits protection
  jsd_slave_config_.epd_nominal.brake_engage_msec =
      params_.elmo_brake_engage_msec;
  jsd_slave_config_.epd_nominal.brake_disengage_msec =
      params_.elmo_brake_disengage_msec;
  jsd_slave_config_.epd_nominal.crc           = params_.elmo_crc;
  jsd_slave_config_.epd_nominal.smooth_factor = params_.smooth_factor;
  jsd_slave_config_.epd_nominal.ctrl_gain_scheduling_mode = ctrl_gs_mode_;
}

void fastcat::PlatinumActuator::PopulateState()
{
  state_->platinum_actuator_state.actual_position =
      PosCntsToEu(jsd_epd_state_.actual_position);
  state_->platinum_actuator_state.actual_velocity =
      CntsToEu(jsd_epd_state_.actual_velocity);
  state_->platinum_actuator_state.actual_current =
      jsd_epd_state_.actual_current;

  state_->platinum_actuator_state.cmd_position =
      PosCntsToEu(jsd_epd_state_.cmd_position + jsd_epd_state_.cmd_ff_position);
  state_->platinum_actuator_state.cmd_velocity =
      CntsToEu(jsd_epd_state_.cmd_velocity + jsd_epd_state_.cmd_ff_velocity);
  state_->platinum_actuator_state.cmd_current =
      jsd_epd_state_.cmd_current + jsd_epd_state_.cmd_ff_current;

  state_->platinum_actuator_state.cmd_prof_velocity =
      jsd_epd_state_.cmd_prof_velocity;
  state_->platinum_actuator_state.cmd_prof_end_velocity =
      jsd_epd_state_.cmd_prof_end_velocity;
  state_->platinum_actuator_state.cmd_prof_accel =
      jsd_epd_state_.cmd_prof_accel;

  state_->platinum_actuator_state.cmd_max_current =
      jsd_epd_state_.cmd_max_current;

  state_->platinum_actuator_state.elmo_state_machine_state =
      static_cast<uint32_t>(jsd_epd_state_.actual_state_machine_state);
  state_->platinum_actuator_state.elmo_mode_of_operation =
      static_cast<uint32_t>(jsd_epd_state_.actual_mode_of_operation);

  state_->platinum_actuator_state.sto_engaged = jsd_epd_state_.sto_engaged;
  state_->platinum_actuator_state.hall_state  = jsd_epd_state_.hall_state;
  state_->platinum_actuator_state.target_reached =
      jsd_epd_state_.target_reached;
  state_->platinum_actuator_state.setpoint_ack_rise =
      jsd_epd_state_.setpoint_ack_rise;
  state_->platinum_actuator_state.motor_on      = jsd_epd_state_.motor_on;
  state_->platinum_actuator_state.servo_enabled = jsd_epd_state_.servo_enabled;

  state_->platinum_actuator_state.jsd_fault_code =
      static_cast<uint32_t>(jsd_epd_state_.fault_code);
  state_->platinum_actuator_state.fastcat_fault_code =
      static_cast<uint32_t>(fastcat_fault_);
  state_->platinum_actuator_state.emcy_error_code =
      jsd_epd_state_.emcy_error_code;
  state_->platinum_actuator_state.faulted =
      (actuator_sms_ == ACTUATOR_SMS_FAULTED);

  state_->platinum_actuator_state.bus_voltage = jsd_epd_state_.bus_voltage;
  state_->platinum_actuator_state.drive_temperature =
      static_cast<uint32_t>(jsd_epd_state_.drive_temperature);

  state_->platinum_actuator_state.elmo_actual_position =
      jsd_epd_state_.actual_position;
  state_->platinum_actuator_state.elmo_cmd_position =
      jsd_epd_state_.cmd_position;

  state_->platinum_actuator_state.actuator_state_machine_state =
      static_cast<uint32_t>(actuator_sms_);

  if (compute_power_) {
    state_->platinum_actuator_state.power =
        ComputePower(state_->platinum_actuator_state.actual_velocity,
                     state_->platinum_actuator_state.actual_current,
                     state_->platinum_actuator_state.motor_on);
  }
}

bool fastcat::PlatinumActuator::HandleNewProfPosCmdImpl(const DeviceCmd& cmd)
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
      ComputeTargetPosProfPosCmd(cmd), state_->platinum_actuator_state.cmd_velocity,
      cmd.actuator_prof_pos_cmd.end_velocity,
      cmd.actuator_prof_pos_cmd.profile_velocity,  // consider abs()
      cmd.actuator_prof_pos_cmd.profile_accel);

  TransitionToState(ACTUATOR_SMS_PROF_POS);

  return true;
}

bool fastcat::PlatinumActuator::HandleNewProfVelCmdImpl(const DeviceCmd& cmd)
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

bool fastcat::PlatinumActuator::HandleNewProfTorqueCmdImpl(const DeviceCmd& cmd)
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

fastcat::FaultType fastcat::PlatinumActuator::ProcessProfPosDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->gold_actuator_state.servo_enabled) {
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


    if ((state_->monotonic_time - last_transition_time_) >
        (prof_disengaging_timeout_ + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage %lf sec timeout expired, faulting",
            name_.c_str(), prof_disengaging_timeout_);
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::PlatinumActuator::ProcessProfVelDisengaging()
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


    if ((state_->monotonic_time - last_transition_time_) >
        (prof_disengaging_timeout_ + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage %lf sec timeout expired, faulting",
            name_.c_str(), prof_disengaging_timeout_);
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::PlatinumActuator::ProcessProfTorqueDisengaging()
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

    if ((state_->monotonic_time - last_transition_time_) >
        (prof_disengaging_timeout_ + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage %lf sec timeout expired, faulting",
            name_.c_str(), prof_disengaging_timeout_);

      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::PlatinumActuator::ProcessProfPos()
{
  return ProcessProfPosTrapImpl();
}

fastcat::FaultType fastcat::PlatinumActuator::ProcessProfVel()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_csv_t jsd_cmd;

  double pos_eu, vel;
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

fastcat::FaultType fastcat::PlatinumActuator::ProcessProfTorque()
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

void fastcat::PlatinumActuator::ElmoRead()
{
  jsd_epd_nominal_read((jsd_t*)context_, slave_id_);
  memcpy(&jsd_epd_state_,
         jsd_epd_nominal_get_state((jsd_t*)context_, slave_id_),
         sizeof(jsd_epd_nominal_state_t));
}

void fastcat::PlatinumActuator::ElmoClearErrors()
{
  jsd_epd_nominal_clear_errors((jsd_t*)context_, slave_id_);
}

void fastcat::PlatinumActuator::ElmoReset()
{
  jsd_epd_nominal_reset((jsd_t*)context_, slave_id_);
}

void fastcat::PlatinumActuator::ElmoSetPeakCurrent(double current)
{
  jsd_epd_nominal_set_peak_current((jsd_t*)context_, slave_id_, current);
}

void fastcat::PlatinumActuator::ElmoSetDigitalOutput(
    uint8_t digital_output_index, uint8_t output_level)
{
  jsd_epd_nominal_set_digital_output((jsd_t*)context_, slave_id_,
                                     digital_output_index, output_level);
}

void fastcat::PlatinumActuator::ElmoSetUnitMode(int32_t mode, uint16_t app_id)
{
  jsd_epd_nominal_async_sdo_set_unit_mode((jsd_t*)context_, slave_id_,
                                          static_cast<int16_t>(mode), app_id);
}

void fastcat::PlatinumActuator::ElmoSetGainSchedulingMode(
    jsd_elmo_gain_scheduling_mode_t mode, uint16_t app_id)
{
  jsd_epd_nominal_async_sdo_set_ctrl_gain_scheduling_mode(
      (jsd_t*)context_, slave_id_, mode, app_id);
}

void fastcat::PlatinumActuator::ElmoSetGainSchedulingIndex(uint16_t index)
{
  jsd_epd_nominal_set_gain_scheduling_index((jsd_t*)context_, slave_id_, true,
                                            index);
}

void fastcat::PlatinumActuator::ElmoCSP(
    const jsd_elmo_motion_command_csp_t& jsd_csp_cmd)
{
  jsd_epd_nominal_set_motion_command_csp((jsd_t*)context_, slave_id_,
                                         jsd_csp_cmd);
}

void fastcat::PlatinumActuator::ElmoCSV(
    const jsd_elmo_motion_command_csv_t& jsd_csv_cmd)
{
  jsd_epd_nominal_set_motion_command_csv((jsd_t*)context_, slave_id_,
                                         jsd_csv_cmd);
}

void fastcat::PlatinumActuator::ElmoCST(
    const jsd_elmo_motion_command_cst_t& jsd_cst_cmd)
{
  jsd_epd_nominal_set_motion_command_cst((jsd_t*)context_, slave_id_,
                                         jsd_cst_cmd);
}

void fastcat::PlatinumActuator::ElmoHalt()
{
  jsd_epd_nominal_halt((jsd_t*)context_, slave_id_);
}

void fastcat::PlatinumActuator::ElmoProcess()
{
  jsd_epd_nominal_process((jsd_t*)context_, slave_id_);
}

double fastcat::PlatinumActuator::GetActualVelocity()
{
  return state_->platinum_actuator_state.actual_velocity;
}

double fastcat::PlatinumActuator::GetElmoActualPosition()
{
  return state_->platinum_actuator_state.elmo_actual_position;
}

jsd_elmo_state_machine_state_t
fastcat::PlatinumActuator::GetElmoStateMachineState()
{
  return static_cast<jsd_elmo_state_machine_state_t>(
      state_->platinum_actuator_state.elmo_state_machine_state);
}

bool fastcat::PlatinumActuator::IsStoEngaged()
{
  return state_->platinum_actuator_state.sto_engaged;
}
