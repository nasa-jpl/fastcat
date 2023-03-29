// Include related header (for cc files)
#include "fastcat/jsd/actuator.h"

// Include c then c++ libraries
#include <string.h>

#include <cmath>

// Include external then project includes
#include "jsd/jsd.h"
#include "jsd/jsd_time.h"

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
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
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
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_CAL;
      return false;
      break;

    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      return false;
  }
  return true;
}

bool fastcat::Actuator::CheckStateMachineGainSchedulingCmds()
{
  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
    case ACTUATOR_SMS_HALTED:
    case ACTUATOR_SMS_HOLDING:
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
    case ACTUATOR_SMS_CS:
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot execute gain scheduling commands, calibration is in "
            "progress");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_CAL;
      return false;

    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      return false;
  }
  return true;
}

bool fastcat::Actuator::HandleNewCSPCmd(DeviceCmd& cmd)
{
  // Validate the command arguments
  if (PosExceedsCmdLimits(cmd.actuator_csp_cmd.target_position +
                          cmd.actuator_csp_cmd.position_offset) ||
      VelExceedsCmdLimits(cmd.actuator_csp_cmd.velocity_offset)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CSP Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CSP Command");
    return false;
  }

  jsd_elmo_motion_command_csp_t jsd_cmd;
  jsd_cmd.target_position = PosEuToCnts(cmd.actuator_csp_cmd.target_position);
  jsd_cmd.position_offset = EuToCnts(cmd.actuator_csp_cmd.position_offset);
  jsd_cmd.velocity_offset = EuToCnts(cmd.actuator_csp_cmd.velocity_offset);
  jsd_cmd.torque_offset_amps = cmd.actuator_csp_cmd.torque_offset_amps;

  EgdCSP(jsd_cmd);

  TransitionToState(ACTUATOR_SMS_CS);

  return true;
}

bool fastcat::Actuator::HandleNewCSVCmd(DeviceCmd& cmd)
{
  // Validate command arguments
  if (VelExceedsCmdLimits(cmd.actuator_csv_cmd.target_velocity +
                          cmd.actuator_csv_cmd.velocity_offset)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CSV Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CSV Command");
    return false;
  }

  jsd_elmo_motion_command_csv_t jsd_cmd;
  jsd_cmd.target_velocity    = EuToCnts(cmd.actuator_csv_cmd.target_velocity);
  jsd_cmd.velocity_offset    = EuToCnts(cmd.actuator_csv_cmd.velocity_offset);
  jsd_cmd.torque_offset_amps = cmd.actuator_csv_cmd.torque_offset_amps;

  EgdCSV(jsd_cmd);

  TransitionToState(ACTUATOR_SMS_CS);

  return true;
}

bool fastcat::Actuator::HandleNewCSTCmd(DeviceCmd& cmd)
{
  // Validate command arguments
  if (CurrentExceedsCmdLimits(cmd.actuator_cst_cmd.target_torque_amps +
                              cmd.actuator_cst_cmd.torque_offset_amps)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CST Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing CST Command");
    return false;
  }

  jsd_elmo_motion_command_cst_t jsd_cmd;
  jsd_cmd.target_torque_amps = cmd.actuator_cst_cmd.target_torque_amps;
  jsd_cmd.torque_offset_amps = cmd.actuator_cst_cmd.torque_offset_amps;

  EgdCST(jsd_cmd);

  TransitionToState(ACTUATOR_SMS_CS);

  return true;
}

bool fastcat::Actuator::HandleNewProfPosCmd(DeviceCmd& cmd)
{
  double target_position = 0;
  if (cmd.actuator_prof_pos_cmd.relative) {
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

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Pos Command");
    return false;
  }

  // Only transition to disengaging if its needed
  if (state_->actuator_state.servo_enabled) {
    // Bypassing wait since brakes are disengaged
  } else {
    TransitionToState(ACTUATOR_SMS_PROF_POS_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  trap_generate(&trap_, state_->time, state_->actuator_state.actual_position,
                target_position, state_->actuator_state.cmd_velocity,
                cmd.actuator_prof_pos_cmd.end_velocity,
                cmd.actuator_prof_pos_cmd.profile_velocity,  // consider abs()
                cmd.actuator_prof_pos_cmd.profile_accel);

  TransitionToState(ACTUATOR_SMS_PROF_POS);
  return true;
}

bool fastcat::Actuator::HandleNewProfVelCmd(DeviceCmd& cmd)
{
  if (VelExceedsCmdLimits(cmd.actuator_prof_vel_cmd.target_velocity) ||
      AccExceedsCmdLimits(cmd.actuator_prof_vel_cmd.profile_accel)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Vel Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Vel Command");
    return false;
  }

  // Only transition to disengaging if its needed
  if (state_->actuator_state.servo_enabled) {
    // Bypassing wait since brakes are disengaged
  } else {
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

bool fastcat::Actuator::HandleNewProfTorqueCmd(DeviceCmd& cmd)
{
  if (CurrentExceedsCmdLimits(
          cmd.actuator_prof_torque_cmd.target_torque_amps)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Torque Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Act %s: %s", name_.c_str(), "Failing Prof Torque Command");
    return false;
  }

  // Only transition to disengaging if its needed
  if (state_->actuator_state.servo_enabled) {
    // Bypassing wait since brakes are disengaged
  } else {
    TransitionToState(ACTUATOR_SMS_PROF_TORQUE_DISENGAGING);
    last_cmd_ = cmd;
    return true;
  }

  trap_generate_vel(&trap_, state_->time, 0, 0,
                    cmd.actuator_prof_torque_cmd.target_torque_amps,
                    params_.torque_slope_amps_per_sec,
                    cmd.actuator_prof_torque_cmd.max_duration);

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
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
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
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
    case ACTUATOR_SMS_CS:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot set output position, motion command is active");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION;
      return false;

    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      return false;
  }

  SetOutputPosition(cmd.actuator_set_output_position_cmd.position);

  // Do not change the state of the Actuator

  return true;
}

bool fastcat::Actuator::HandleNewSetUnitModeCmd(DeviceCmd& cmd)
{
  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
    case ACTUATOR_SMS_HALTED:
    case ACTUATOR_SMS_HOLDING:
      break;

    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
    case ACTUATOR_SMS_CS:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      ERROR("Act %s: %s", name_.c_str(),
            "Cannot set unit mode now, motion command is active");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION;
      return false;

    default:
      ERROR("Act %s: %s: %d", name_.c_str(), "Bad Act State ", actuator_sms_);
      return false;
  }

  EgdSetUnitMode(cmd.actuator_sdo_set_unit_mode_cmd.mode,
                 cmd.actuator_sdo_set_unit_mode_cmd.app_id);

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
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
    case ACTUATOR_SMS_CS:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR("Act %s: %s", name_.c_str(),
            "Calibration requested during active motion command, faulting");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION;
      return false;
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR("Act %s: %s", name_.c_str(),
            "Calibration requested but calibration already in progress, "
            "faulting");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_CAL;
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

  double rom =
      fabs(params_.high_pos_cal_limit_eu - params_.low_pos_cal_limit_eu);
  double cal_range = rom + params_.pos_tracking_error_eu;

  // Since the hardstop calibration feature depends on a position tracking
  // fault,
  //   sanity check drive settings to prevent unexpected outcomes.
  //   At the very least, the Range-of-Motion must be > the position tracking
  //   fault tolerance. (In practice, ROM should be MUCH > than the full
  //   tracking tol)
  if (rom < params_.pos_tracking_error_eu) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR(
        "Act %s: Calibration cannot succeed when Range-of-motion (%lf) "
        "< Pos tracking error (%lf). "
        "Check Drive parameters for excessive pos tracking fault",
        name_.c_str(), rom, params_.pos_tracking_error_eu);
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CAL_MOTION_RANGE;
    return false;
  }

  cal_cmd_ = cmd.actuator_calibrate_cmd;

  double target_position;
  if (cmd.actuator_calibrate_cmd.velocity > 0) {
    target_position = state_->actuator_state.actual_position + cal_range;
  } else {
    target_position = state_->actuator_state.actual_position - cal_range;
  }

  MSG("Setting Peak Current to calibration level: %lf", cal_cmd_.max_current);
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
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_STO_ENGAGED;
    return true;
  }

  if (state_->actuator_state.jsd_fault_code != 0) {
    ERROR("%s: jsd_fault_code indicates active fault", name_.c_str());
    return true;
  }
  return false;
}

bool fastcat::Actuator::IsMotionFaultConditionMet()
{
  if (state_->actuator_state.sto_engaged) {
    ERROR("%s: STO Engaged", name_.c_str());
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_STO_ENGAGED;
    return true;
  }

  if (state_->actuator_state.jsd_fault_code != 0) {
    ERROR("%s: jsd_fault_code indicates active fault", name_.c_str());
    return true;
  }
  if (state_->actuator_state.egd_state_machine_state ==
          JSD_ELMO_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE ||
      state_->actuator_state.egd_state_machine_state ==
          JSD_ELMO_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE ||
      state_->actuator_state.egd_state_machine_state ==
          JSD_ELMO_STATE_MACHINE_STATE_FAULT) {
    ERROR("%s: EGD state machine state is off nominal", name_.c_str());
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_EGD_SMS_DURING_MOTION;
    return true;
  }
  return false;
}

fastcat::FaultType fastcat::Actuator::ProcessFaulted() { return NO_FAULT; }

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

  if ((state_->time - last_transition_time_) > params_.holding_duration_sec) {
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

  jsd_elmo_motion_command_csp_t jsd_cmd;

  double pos_eu, vel;
  int    complete = trap_update(&trap_, state_->time, &pos_eu, &vel);

  jsd_cmd.target_position    = PosEuToCnts(pos_eu);
  jsd_cmd.position_offset    = 0;
  jsd_cmd.velocity_offset    = EuToCnts(vel);
  jsd_cmd.torque_offset_amps = 0;

  if (!complete || prof_pos_hold_) {
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

  jsd_elmo_motion_command_csv_t jsd_cmd;

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

  jsd_elmo_motion_command_cst_t jsd_cmd;

  double dummy_pos_eu, current;
  int complete = trap_update_vel(&trap_, state_->time, &dummy_pos_eu, &current);

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
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_STO_ENGAGED;

    MSG("Restoring Current after calibration: %lf",
        params_.peak_current_limit_amps);
    EgdSetPeakCurrent(params_.peak_current_limit_amps);

    return ALL_DEVICE_FAULT;
  }

  // assume pos/vel tracking fault
  if (state_->actuator_state.jsd_fault_code != 0) {
    EgdHalt();
    MSG("Act %s: %s: %d", name_.c_str(),
        "Detected Hardstop, EGD jsd_fault_code",
        state_->actuator_state.jsd_fault_code);

    MSG("Restoring Current after calibration: %lf",
        params_.peak_current_limit_amps);
    EgdSetPeakCurrent(params_.peak_current_limit_amps);

    TransitionToState(ACTUATOR_SMS_CAL_AT_HARDSTOP);
  }

  jsd_elmo_motion_command_csp_t jsd_cmd;

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
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_NO_HARDSTOP_DURING_CAL;

    MSG("Restoring Current after calibration: %lf",
        params_.peak_current_limit_amps);
    EgdSetPeakCurrent(params_.peak_current_limit_amps);

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
          JSD_ELMO_STATE_MACHINE_STATE_OPERATION_ENABLED &&
      state_->actuator_state.jsd_fault_code != 0) {
    // We have waited too long, fault
    if ((state_->time - last_transition_time_) > 5.0) {
      ERROR("Act %s: %s: %lf", name_.c_str(),
            "Waited too long for drive to reset in CAL_AT_HARDSTOP state",
            (state_->time - last_transition_time_));
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_CAL_RESET_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }

    // still waiting on the drive to reset...
    EgdReset();
    return NO_FAULT;
  }

  double cal_position     = 0;
  double backoff_position = 0;
  if (cal_cmd_.velocity > 0) {
    cal_position     = params_.high_pos_cal_limit_eu;
    backoff_position = params_.high_pos_cmd_limit_eu;
  } else {
    cal_position     = params_.low_pos_cal_limit_eu;
    backoff_position = params_.low_pos_cmd_limit_eu;
  }
  SetOutputPosition(cal_position);

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

fastcat::FaultType fastcat::Actuator::ProcessProfPosDisengaging()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Act %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  double target_position = 0;
  if (last_cmd_.actuator_prof_pos_cmd.relative) {
    target_position = last_cmd_.actuator_prof_pos_cmd.target_position +
                      state_->actuator_state.actual_position;
  } else {
    target_position = last_cmd_.actuator_prof_pos_cmd.target_position;
  }

  if (state_->actuator_state.servo_enabled) {
    // If brakes are disengaged, setup the traps and transition to the execution
    // state
    trap_generate(
        &trap_, state_->time, state_->actuator_state.actual_position,
        target_position, state_->actuator_state.cmd_velocity,
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

    EgdCSP(jsd_cmd);

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

fastcat::FaultType fastcat::Actuator::ProcessProfVelDisengaging()
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

    EgdCSV(jsd_cmd);

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

fastcat::FaultType fastcat::Actuator::ProcessProfTorqueDisengaging()
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
                      params_.torque_slope_amps_per_sec,
                      last_cmd_.actuator_prof_torque_cmd.max_duration);

    TransitionToState(ACTUATOR_SMS_PROF_TORQUE);

  } else {
    // Otherwise, command the current position to trigger the transition and
    // wait
    jsd_elmo_motion_command_cst_t jsd_cmd;

    jsd_cmd.target_torque_amps = 0;
    jsd_cmd.torque_offset_amps = 0;

    EgdCST(jsd_cmd);

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
