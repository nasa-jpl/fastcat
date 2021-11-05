// Include related header (for cc files)
#include "fastcat/jsd/actuator.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>

// Include external then project includes
#include "jsd/jsd.h"

/*

  switch(actuator_sms_){
    case ACTUATOR_SMS_FAULTED:
    case ACTUATOR_SMS_HALTED:
    case ACTUATOR_SMS_HOLDING:
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_CS:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
    default:
      ERROR("Act %s: %s: %d", name_.c_str(),
         "Bad Act State ", actuator_sms_);
      return false;
  }

*/

bool fastcat::Actuator::CheckStateMachineMotionCmds()
{
  // This check is the same for both CS* and PROF* commands.
  //
  // If special state handling is ever desired, it is recommended that
  // this be broken between CS and PROF cmds. Like so:
  //   CheckStateMachineCSCmds();
  //   CheckStateMachineProfCmds(;)

  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot call a Motion cmd from FAULTED, reset Actuator first");
      return false;
      break;

    case ACTUATOR_SMS_HALTED:
      EgdReset();  // This will open the brake, then fallthrough
    case ACTUATOR_SMS_HOLDING:
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_CS:
      // All of these commands can be safely preempted with CS cmds
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      // Currently, this faults the cal command
      //   Can be configured to ignore the offending CSP command
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot call a Motion cmd during Calibration");
      return false;
      break;

    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      return false;
  }
  return true;
}

bool fastcat::Actuator::HandleNewCSPCmd(DeviceCmd& cmd)
{
  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CSP Command");
    return false;
  }

  // Validate the command arguments
  if (PosExceedsCmdLimits(cmd.actuator_csp_cmd.target_position +
                          cmd.actuator_csp_cmd.position_offset) ||
      VelExceedsCmdLimits(cmd.actuator_csp_cmd.velocity_offset)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CSP Command");
    return false;
  }

  jsd_egd_motion_command_csp_t jsd_cmd;
  jsd_cmd.target_position = PosEuToCnts(cmd.actuator_csp_cmd.target_position);
  jsd_cmd.position_offset = EuToCnts(cmd.actuator_csp_cmd.position_offset);
  jsd_cmd.velocity_offset = EuToCnts(cmd.actuator_csp_cmd.velocity_offset);
  jsd_cmd.torque_offset_amps = cmd.actuator_csp_cmd.torque_offset_amps;

  EgdCSP(jsd_cmd);
  EgdSetPeakCurrent(peak_current_limit_amps_);

  TransitionToState(ACTUATOR_SMS_CS);

  return true;
}

bool fastcat::Actuator::HandleNewCSVCmd(DeviceCmd& cmd)
{
  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CSV Command");
    return false;
  }

  // Validate command arguments
  if (VelExceedsCmdLimits(cmd.actuator_csv_cmd.target_velocity +
                          cmd.actuator_csv_cmd.velocity_offset)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CSV Command");
    return false;
  }

  jsd_egd_motion_command_csv_t jsd_cmd;
  jsd_cmd.target_velocity    = EuToCnts(cmd.actuator_csv_cmd.target_velocity);
  jsd_cmd.velocity_offset    = EuToCnts(cmd.actuator_csv_cmd.velocity_offset);
  jsd_cmd.torque_offset_amps = cmd.actuator_csv_cmd.torque_offset_amps;

  EgdCSV(jsd_cmd);
  EgdSetPeakCurrent(peak_current_limit_amps_);

  TransitionToState(ACTUATOR_SMS_CS);

  return true;
}

bool fastcat::Actuator::HandleNewCSTCmd(DeviceCmd& cmd)
{
  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CST Command");
    return false;
  }

  // Validate command arguments
  if (CurrentExceedsCmdLimits(cmd.actuator_cst_cmd.target_torque_amps +
                              cmd.actuator_cst_cmd.torque_offset_amps)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CST Command");
    return false;
  }

  jsd_egd_motion_command_cst_t jsd_cmd;
  jsd_cmd.target_torque_amps = cmd.actuator_cst_cmd.target_torque_amps;
  jsd_cmd.torque_offset_amps = cmd.actuator_cst_cmd.torque_offset_amps;

  EgdCST(jsd_cmd);
  EgdSetPeakCurrent(peak_current_limit_amps_);

  TransitionToState(ACTUATOR_SMS_CS);

  return true;
}

bool fastcat::Actuator::HandleNewProfPosCmd(DeviceCmd& cmd)
{
  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Pos Command");
    return false;
  }

  double target_position = 0;
  if (cmd.actuator_prof_pos_cmd.relative) {
    // may be more stable to use cmd than actual position for relative motion
    target_position = cmd.actuator_prof_pos_cmd.target_position +
                      state_->actuator_state.actual_position;
  } else {
    target_position = cmd.actuator_prof_pos_cmd.target_position;
  }

  // Validate command arguments
  if (PosExceedsCmdLimits(target_position) ||
      VelExceedsCmdLimits(cmd.actuator_prof_pos_cmd.profile_velocity) ||
      VelExceedsCmdLimits(cmd.actuator_prof_pos_cmd.end_velocity) ||
      AccExceedsCmdLimits(cmd.actuator_prof_pos_cmd.profile_accel)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Pos Command");
    return false;
  }

  EgdSetPeakCurrent(peak_current_limit_amps_);

  trap_generate(
      &trap_, state_->time,
      state_->actuator_state.actual_position,  // consider cmd position
      target_position,
      state_->actuator_state.actual_velocity,  // consider cmd vel
      cmd.actuator_prof_pos_cmd.end_velocity,
      cmd.actuator_prof_pos_cmd.profile_velocity, // consider abs()
      cmd.actuator_prof_pos_cmd.profile_accel);

  TransitionToState(ACTUATOR_SMS_PROF_POS);
  return true;
}

bool fastcat::Actuator::HandleNewProfVelCmd(DeviceCmd& cmd)
{
  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Vel Command");
    return false;
  }

  if (VelExceedsCmdLimits(cmd.actuator_prof_vel_cmd.target_velocity) ||
      AccExceedsCmdLimits(cmd.actuator_prof_vel_cmd.profile_accel)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Vel Command");
    return false;
  }

  EgdSetPeakCurrent(peak_current_limit_amps_);

  trap_generate_vel(
      &trap_, state_->time,
      state_->actuator_state.actual_position,  // consider cmd position
      state_->actuator_state.actual_velocity,  // consider cmd velocity
      cmd.actuator_prof_vel_cmd.target_velocity,
      cmd.actuator_prof_vel_cmd.profile_accel,
      cmd.actuator_prof_vel_cmd.max_duration);

  TransitionToState(ACTUATOR_SMS_PROF_VEL);
  return true;
}

bool fastcat::Actuator::HandleNewProfTorqueCmd(DeviceCmd& cmd)
{
  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Torque Command");
    return false;
  }

  if (CurrentExceedsCmdLimits(
          cmd.actuator_prof_torque_cmd.target_torque_amps)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Torque Command");
    return false;
  }

  EgdSetPeakCurrent(peak_current_limit_amps_);

  trap_generate_vel(
      &trap_, state_->time, 0, 0, cmd.actuator_prof_torque_cmd.target_torque_amps,
      torque_slope_amps_per_sec_, cmd.actuator_prof_torque_cmd.max_duration);

  TransitionToState(ACTUATOR_SMS_PROF_TORQUE);
  return true;
}

bool fastcat::Actuator::HandleNewHaltCmd()
{
  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot HALT from FAULTED state, reset actuator first");
      return false;
      break;

    case ACTUATOR_SMS_HALTED:
      return true;
      break;

    case ACTUATOR_SMS_HOLDING:
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_CS:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      break;

    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      return false;
  }

  EgdHalt();
  TransitionToState(ACTUATOR_SMS_HALTED);

  return true;
}

bool fastcat::Actuator::HandleNewSetOutputPositionCmd(DeviceCmd& cmd)
{
  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
    case ACTUATOR_SMS_HALTED:
    case ACTUATOR_SMS_HOLDING:
      break;

    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_CS:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot set output position, motion command is active");
      return false;

    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      return false;
  }

  SetOutputPosition(cmd.actuator_set_output_position_cmd.position);

  // Do not change the state of the Actuator

  return true;
}

bool fastcat::Actuator::HandleNewCalibrationCmd(DeviceCmd& cmd)
{
  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR(
          "Act %s: %s", name_.c_str(),
          "Cannot start calibration from FAULTED state, reset actuator first");
      return false;

    case ACTUATOR_SMS_HALTED:
      EgdReset();
    case ACTUATOR_SMS_HOLDING:
      break;
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_CS:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR("Act %s: %s", name_.c_str(),
            "Calibration requested during active motion command, faulting");
      return false;
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR("Act %s: %s", name_.c_str(),
            "Calibration requested but calibration already in progress, "
            "faulting");
      return false;

    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      return false;
  }

  if (VelExceedsCmdLimits(cmd.actuator_calibrate_cmd.velocity) ||
      AccExceedsCmdLimits(cmd.actuator_calibrate_cmd.accel)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Calibration Command");
    return false;
  }

  cal_cmd_ = cmd.actuator_calibrate_cmd;

  double cal_range =
      (high_pos_cal_limit_eu_ - low_pos_cal_limit_eu_) + pos_tracking_error_eu_;

  double target_position;
  if (cmd.actuator_calibrate_cmd.velocity > 0) {
    target_position = state_->actuator_state.actual_position + cal_range;
  } else {
    target_position = state_->actuator_state.actual_position - cal_range;
  }

  EgdSetPeakCurrent(cal_cmd_.max_current);

  trap_generate(
      &trap_, state_->time,
      state_->actuator_state.actual_position,  // consider cmd position
      target_position,
      state_->actuator_state.actual_velocity,  // consider cmd velocity
      0,  // pt2pt motion always uses terminating traps
      fabs(cmd.actuator_calibrate_cmd.velocity),
      cmd.actuator_calibrate_cmd.accel);

  TransitionToState(ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP);

  return true;
}

bool fastcat::Actuator::IsIdleFaultConditionMet()
{
  if (state_->actuator_state.sto_engaged) {
    ERROR("%s: STO Engaged", name_.c_str());
    return true;
  }

  if (state_->actuator_state.fault_code != 0) {
    ERROR("%s: fault_code indicates active fault", name_.c_str());
    return true;
  }
  return false;
}

bool fastcat::Actuator::IsMotionFaultConditionMet()
{
  if (state_->actuator_state.sto_engaged) {
    ERROR("%s: STO Engaged", name_.c_str());
    return true;
  }

  if (state_->actuator_state.fault_code != 0) {
    ERROR("%s: fault_code indicates active fault", name_.c_str());
    return true;
  }
  if (state_->actuator_state.egd_state_machine_state ==
          JSD_EGD_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE ||
      state_->actuator_state.egd_state_machine_state ==
          JSD_EGD_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE ||
      state_->actuator_state.egd_state_machine_state ==
          JSD_EGD_STATE_MACHINE_STATE_FAULT) {
    ERROR("%s: EGD state machine state is off nominal", name_.c_str());
    return true;
  }
  return false;
}

fastcat::FaultType fastcat::Actuator::ProcessFaulted()
{
  // if (state_->actuator_state.fault_code == 0 &&
  //    !device_fault_active_) {
  //  EgdReset();
  //  TransitionToState(ACTUATOR_SMS_HOLDING);
  //}else{
  //  device_fault_active_ = true;
  //}
  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessHalted()
{
  if (IsIdleFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessHolding()
{
  if (IsIdleFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if ((state_->time - last_transition_time_) > holding_duration_sec_) {
    EgdHalt();
    TransitionToState(ACTUATOR_SMS_HALTED);
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessProfPos()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_egd_motion_command_csp_t jsd_cmd;

  double pos_eu, vel;
  int    complete = trap_update(&trap_, state_->time, &pos_eu, &vel);

  jsd_cmd.target_position    = PosEuToCnts(pos_eu);
  jsd_cmd.position_offset    = 0;
  jsd_cmd.velocity_offset    = EuToCnts(vel);
  jsd_cmd.torque_offset_amps = 0;

  if (!complete) {
    EgdCSP(jsd_cmd);
  } else {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessProfVel()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_egd_motion_command_csv_t jsd_cmd;

  double pos_eu, vel;
  int    complete = trap_update_vel(&trap_, state_->time, &pos_eu, &vel);

  jsd_cmd.target_velocity    = EuToCnts(vel);
  jsd_cmd.velocity_offset    = 0;
  jsd_cmd.torque_offset_amps = 0;

  if (!complete) {
    EgdCSV(jsd_cmd);
  } else {
    jsd_cmd.target_velocity = 0;
    EgdCSV(jsd_cmd);
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessProfTorque()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_egd_motion_command_cst_t jsd_cmd;

  double dummy_pos_eu, current;
  int    complete = trap_update_vel(&trap_, state_->time, &dummy_pos_eu, &current);

  jsd_cmd.target_torque_amps = current;
  jsd_cmd.torque_offset_amps = 0;

  EgdCST(jsd_cmd);

  if (complete) {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCS()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if ((state_->time - last_transition_time_) > 5 * loop_period_) {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCalMoveToHardstop()
{
  // add other reasons as needed...
  if (state_->actuator_state.sto_engaged) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  // assume pos/vel tracking fault
  if (state_->actuator_state.fault_code != 0) {
    EgdHalt();
    ERROR("Act %s: %s: %d", name_.c_str(), "Detected Hardstop, EGD fault_code",
          state_->actuator_state.fault_code);
    TransitionToState(ACTUATOR_SMS_CAL_AT_HARDSTOP);
  }

  jsd_egd_motion_command_csp_t jsd_cmd;

  double pos_eu, vel;
  int    complete = trap_update(&trap_, state_->time, &pos_eu, &vel);

  jsd_cmd.target_position    = PosEuToCnts(pos_eu);
  jsd_cmd.position_offset    = 0;
  jsd_cmd.velocity_offset    = EuToCnts(vel);
  jsd_cmd.torque_offset_amps = 0;

  EgdCSP(jsd_cmd);

  if (complete) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(),
          "Moved Full Range and did not encounter hard stop");
    return ALL_DEVICE_FAULT;
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCalAtHardstop()
{
  // no need to check faults in this state
  // And clear the EGD fault that is generated from contacting the hardstop
  // Loop here until the drive is no longer faulted

  if (state_->actuator_state.egd_state_machine_state !=
          JSD_EGD_STATE_MACHINE_STATE_OPERATION_ENABLED &&
      state_->actuator_state.fault_code != 0) {
    // We have waited too long, fault
    if ((state_->time - last_transition_time_) > 5.0) {
      ERROR("Act %s: %s: %lf", name_.c_str(),
            "Waited too long for drive to reset in CAL_AT_HARDSTOP state",
            (state_->time - last_transition_time_));
      return ALL_DEVICE_FAULT;
    }

    // still waiting on the drive to reset...
    EgdReset();
    return NO_FAULT;
  }

  double cal_position     = 0;
  double backoff_position = 0;
  if (cal_cmd_.velocity > 0) {
    cal_position     = high_pos_cal_limit_eu_;
    backoff_position = high_pos_cmd_limit_eu_;
  } else {
    cal_position     = low_pos_cal_limit_eu_;
    backoff_position = low_pos_cmd_limit_eu_;
  }
  SetOutputPosition(cal_position);

  // Restore Current to nominal value
  EgdSetPeakCurrent(peak_current_limit_amps_);

  trap_generate(&trap_, state_->time, cal_position, backoff_position, 0,
                0,  // pt2pt motion always uses terminating traps
                fabs(cal_cmd_.velocity), cal_cmd_.accel);

  TransitionToState(ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP);

  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCalMoveToSoftstop()
{
  return ProcessProfPos();
}
