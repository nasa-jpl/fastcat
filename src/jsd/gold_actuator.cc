// Include related header (for cc files)
#include "fastcat/jsd/gold_actuator.h"

// Include C then C++ libraries
#include <cmath>
#include <cstring>

// Include external then project includes
#include "jsd/jsd.h"

fastcat::GoldActuator::GoldActuator() { state_->type = GOLD_ACTUATOR_STATE; }

void fastcat::GoldActuator::PopulateJsdSlaveConfig()
{
  jsd_slave_config_.product_code = JSD_EGD_PRODUCT_CODE;

  jsd_slave_config_.egd.drive_cmd_mode    = JSD_EGD_DRIVE_CMD_MODE_CS;
  jsd_slave_config_.egd.max_motor_speed   = EuToCnts(params_.max_speed_eu_per_sec);
  jsd_slave_config_.egd.loop_period_ms    = lround(loop_period_ * 1000.0);
  jsd_slave_config_.egd.torque_slope      = params_.torque_slope_amps_per_sec;
  jsd_slave_config_.egd.max_profile_accel = EuToCnts(params_.max_accel_eu_per_sec2);
  jsd_slave_config_.egd.max_profile_decel = EuToCnts(params_.max_accel_eu_per_sec2);
  jsd_slave_config_.egd.velocity_tracking_error =
      EuToCnts(params_.vel_tracking_error_eu_per_sec);
  jsd_slave_config_.egd.position_tracking_error =
      EuToCnts(params_.pos_tracking_error_eu);
  jsd_slave_config_.egd.peak_current_limit = params_.peak_current_limit_amps;
  jsd_slave_config_.egd.peak_current_time  = params_.peak_current_time_sec;
  jsd_slave_config_.egd.continuous_current_limit =
      params_.continuous_current_limit_amps;
  jsd_slave_config_.egd.motor_stuck_current_level_pct  = 0.0f;  // disable
  jsd_slave_config_.egd.motor_stuck_velocity_threshold = 0.0f;  // disable
  jsd_slave_config_.egd.motor_stuck_timeout            = 0.0f;  // disable
  jsd_slave_config_.egd.over_speed_threshold =
      params_.over_speed_multiplier * EuToCnts(params_.max_speed_eu_per_sec);
  jsd_slave_config_.egd.low_position_limit   = 0;  // disable
  jsd_slave_config_.egd.high_position_limit  = 0;  // disable
  jsd_slave_config_.egd.brake_engage_msec    = params_.elmo_brake_engage_msec;
  jsd_slave_config_.egd.brake_disengage_msec = params_.elmo_brake_disengage_msec;
  jsd_slave_config_.egd.crc                  = params_.elmo_crc;
  jsd_slave_config_.egd.drive_max_current_limit =
      params_.elmo_drive_max_cur_limit_amps;
  jsd_slave_config_.egd.smooth_factor = params_.smooth_factor;
  jsd_slave_config_.egd.ctrl_gain_scheduling_mode = ctrl_gs_mode_;
}

void fastcat::GoldActuator::ElmoRead()
{
  jsd_egd_read((jsd_t*)context_, slave_id_);
  memcpy(&jsd_egd_state_, jsd_egd_get_state((jsd_t*)context_, slave_id_),
         sizeof(jsd_egd_state_t));
}

void fastcat::GoldActuator::PopulateState()
{
  state_->gold_actuator_state.elmo_actual_position =
      jsd_egd_state_.actual_position;
  state_->gold_actuator_state.elmo_cmd_position = jsd_egd_state_.cmd_position;

  state_->gold_actuator_state.actual_position =
      PosCntsToEu(jsd_egd_state_.actual_position);
  state_->gold_actuator_state.actual_velocity =
      CntsToEu(jsd_egd_state_.actual_velocity);
  state_->gold_actuator_state.actual_current = jsd_egd_state_.actual_current;

  state_->gold_actuator_state.cmd_position =
      PosCntsToEu(jsd_egd_state_.cmd_position + jsd_egd_state_.cmd_ff_position);
  state_->gold_actuator_state.cmd_velocity =
      CntsToEu(jsd_egd_state_.cmd_velocity + jsd_egd_state_.cmd_ff_velocity);
  state_->gold_actuator_state.cmd_current =
      (jsd_egd_state_.cmd_current + jsd_egd_state_.cmd_ff_current);

  state_->gold_actuator_state.cmd_max_current = jsd_egd_state_.cmd_max_current;

  state_->gold_actuator_state.elmo_state_machine_state =
      static_cast<uint32_t>(jsd_egd_state_.actual_state_machine_state);
  state_->gold_actuator_state.elmo_mode_of_operation =
      static_cast<uint32_t>(jsd_egd_state_.actual_mode_of_operation);

  state_->gold_actuator_state.sto_engaged    = jsd_egd_state_.sto_engaged;
  state_->gold_actuator_state.hall_state     = jsd_egd_state_.hall_state;
  state_->gold_actuator_state.target_reached = jsd_egd_state_.target_reached;
  state_->gold_actuator_state.motor_on       = jsd_egd_state_.motor_on;
  state_->gold_actuator_state.servo_enabled  = jsd_egd_state_.servo_enabled;

  state_->gold_actuator_state.bus_voltage = jsd_egd_state_.bus_voltage;
  state_->gold_actuator_state.drive_temperature =
      jsd_egd_state_.drive_temperature;

  state_->gold_actuator_state.actuator_state_machine_state =
      static_cast<uint32_t>(actuator_sms_);

  state_->gold_actuator_state.fastcat_fault_code =
      static_cast<uint32_t>(fastcat_fault_);
  state_->gold_actuator_state.jsd_fault_code =
      static_cast<uint32_t>(jsd_egd_state_.fault_code);
  state_->gold_actuator_state.emcy_error_code = jsd_egd_state_.emcy_error_code;
  state_->gold_actuator_state.faulted = (actuator_sms_ == ACTUATOR_SMS_FAULTED);

  if (compute_power_) {
    state_->gold_actuator_state.power =
        ComputePower(state_->gold_actuator_state.actual_velocity,
                     state_->gold_actuator_state.actual_current,
                     state_->gold_actuator_state.motor_on);
  }
}

void fastcat::GoldActuator::ElmoClearErrors()
{
  jsd_egd_clear_errors((jsd_t*)context_, slave_id_);
}

void fastcat::GoldActuator::ElmoSetPeakCurrent(double current)
{
  jsd_egd_set_peak_current((jsd_t*)context_, slave_id_, current);
}

void fastcat::GoldActuator::ElmoSetDigitalOutput(uint8_t digital_output_index, uint8_t output_level)
{
  jsd_egd_set_digital_output((jsd_t*)context_, slave_id_, digital_output_index, output_level);
}

void fastcat::GoldActuator::ElmoSetUnitMode(int32_t mode, uint16_t app_id)
{
  MSG("Commanding new UM[1] = %d app_id = %u", mode, app_id);
  jsd_egd_async_sdo_set_unit_mode((jsd_t*)context_, slave_id_, mode, app_id);
}

void fastcat::GoldActuator::ElmoSetGainSchedulingMode(
    jsd_elmo_gain_scheduling_mode_t mode, uint16_t app_id)
{
  jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode((jsd_t*)context_, slave_id_,
                                                  mode, app_id);
}

void fastcat::GoldActuator::ElmoSetGainSchedulingIndex(uint16_t index)
{
  jsd_egd_set_gain_scheduling_index((jsd_t*)context_, slave_id_, true, index);
}

void fastcat::GoldActuator::ElmoFault()
{
  MSG("Faulting EGD through JSD: %s", name_.c_str());
  jsd_egd_fault((jsd_t*)context_, slave_id_);

  // TODO review with david
  // need to clear so that old commands are not left over 
  //  for new commands
  jsd_egd_state_.cmd_position = 0;
  jsd_egd_state_.cmd_velocity = 0;
  jsd_egd_state_.cmd_current  = 0;
}

void fastcat::GoldActuator::ElmoReset()
{
  MSG("Resetting EGD through JSD: %s", name_.c_str());
  jsd_egd_reset((jsd_t*)context_, slave_id_);
}

void fastcat::GoldActuator::ElmoCSP(
    const jsd_elmo_motion_command_csp_t& jsd_csp_cmd)
{
  jsd_egd_set_motion_command_csp((jsd_t*)context_, slave_id_, jsd_csp_cmd);
}

void fastcat::GoldActuator::ElmoCSV(
    const jsd_elmo_motion_command_csv_t& jsd_csv_cmd)
{
  jsd_egd_set_motion_command_csv((jsd_t*)context_, slave_id_, jsd_csv_cmd);
}

void fastcat::GoldActuator::ElmoCST(
    const jsd_elmo_motion_command_cst_t& jsd_cst_cmd)
{
  jsd_egd_set_motion_command_cst((jsd_t*)context_, slave_id_, jsd_cst_cmd);
}

void fastcat::GoldActuator::ElmoHalt()
{
  ERROR("Doing the elmo halt for gold actuator!");
  jsd_egd_halt((jsd_t*)context_, slave_id_);
}

void fastcat::GoldActuator::ElmoProcess()
{
  jsd_egd_process((jsd_t*)context_, slave_id_);
}

bool fastcat::GoldActuator::HandleNewProfPosCmdImpl(const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->gold_actuator_state.servo_enabled) {
    TransitionToState(ACTUATOR_SMS_PROF_POS_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  fastcat_trap_generate(
      &trap_, state_->time, state_->gold_actuator_state.actual_position,
      ComputeTargetPosProfPosCmd(cmd), state_->gold_actuator_state.cmd_velocity,
      cmd.actuator_prof_pos_cmd.end_velocity,
      cmd.actuator_prof_pos_cmd.profile_velocity,  // consider abs()
      cmd.actuator_prof_pos_cmd.profile_accel);

  TransitionToState(ACTUATOR_SMS_PROF_POS);

  return true;
}

bool fastcat::GoldActuator::HandleNewProfVelCmdImpl(const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->gold_actuator_state.servo_enabled) {
    TransitionToState(ACTUATOR_SMS_PROF_VEL_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  fastcat_trap_generate_vel(&trap_, state_->time,
                    state_->gold_actuator_state.actual_position,
                    state_->gold_actuator_state.cmd_velocity,
                    cmd.actuator_prof_vel_cmd.target_velocity,
                    cmd.actuator_prof_vel_cmd.profile_accel,
                    cmd.actuator_prof_vel_cmd.max_duration);

  TransitionToState(ACTUATOR_SMS_PROF_VEL);

  return true;
}

bool fastcat::GoldActuator::HandleNewProfTorqueCmdImpl(const DeviceCmd& cmd)
{
  // Only transition to disengaging if it is needed (i.e. brakes are still
  // engaged)
  if (!state_->gold_actuator_state.servo_enabled) {
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

fastcat::FaultType fastcat::GoldActuator::ProcessProfPos()
{
  return ProcessProfPosTrapImpl();
}

fastcat::FaultType fastcat::GoldActuator::ProcessProfVel()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_csv_t jsd_cmd;

  double pos_eu, vel;
  int    complete = fastcat_trap_update_vel(&trap_, state_->time, &pos_eu, &vel);

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

fastcat::FaultType fastcat::GoldActuator::ProcessProfTorque()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_cst_t jsd_cmd;

  double dummy_pos_eu, current;
  int complete = fastcat_trap_update_vel(&trap_, state_->time, &dummy_pos_eu, &current);

  jsd_cmd.target_torque_amps = current;
  jsd_cmd.torque_offset_amps = 0;

  ElmoCST(jsd_cmd);

  if (complete) {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::GoldActuator::ProcessProfPosDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->gold_actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    fastcat_trap_generate(
        &trap_, state_->time, state_->gold_actuator_state.actual_position,
        ComputeTargetPosProfPosCmd(last_cmd_),
        state_->gold_actuator_state.cmd_velocity,
        last_cmd_.actuator_prof_pos_cmd.end_velocity,
        last_cmd_.actuator_prof_pos_cmd.profile_velocity,  // consider abs()
        last_cmd_.actuator_prof_pos_cmd.profile_accel);

    TransitionToState(ACTUATOR_SMS_PROF_POS);

  } else {
    // Otherwise, command the current position to trigger the transition and
    // wait

    jsd_elmo_motion_command_csp_t jsd_cmd;

    jsd_cmd.target_position =
        PosEuToCnts(state_->gold_actuator_state.actual_position);
    jsd_cmd.position_offset    = 0;
    jsd_cmd.velocity_offset    = 0;
    jsd_cmd.torque_offset_amps = 0;

    ElmoCSP(jsd_cmd);

    // Check runout timer here, brake engage/disengage time cannot exceed 1
    // second per MAN-G-CR Section BP - Brake Parameters
    if ((cycle_mono_time_ - last_transition_time_) > (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::GoldActuator::ProcessProfVelDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->gold_actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    fastcat_trap_generate_vel(&trap_, state_->time,
                      state_->gold_actuator_state.actual_position,
                      state_->gold_actuator_state.cmd_velocity,
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
    if ((cycle_mono_time_ - last_transition_time_) > (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::GoldActuator::ProcessProfTorqueDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if (state_->gold_actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    fastcat_trap_generate_vel(&trap_, state_->time, 0, 0,
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
    if ((cycle_mono_time_ - last_transition_time_) > (1.0 + 2 * loop_period_)) {
      ERROR("Act %s: Brake Disengage 1.0 sec runout timer expired, faulting",
            name_.c_str());
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
  }

  return NO_FAULT;
}

double fastcat::GoldActuator::GetActualVelocity()
{
  return state_->gold_actuator_state.actual_velocity;
}

double fastcat::GoldActuator::GetElmoActualPosition()
{
  return state_->gold_actuator_state.elmo_actual_position;
}

jsd_elmo_state_machine_state_t fastcat::GoldActuator::GetElmoStateMachineState()
{
  return static_cast<jsd_elmo_state_machine_state_t>(
      state_->gold_actuator_state.elmo_state_machine_state);
}

bool fastcat::GoldActuator::IsStoEngaged()
{
  return state_->gold_actuator_state.sto_engaged;
}
