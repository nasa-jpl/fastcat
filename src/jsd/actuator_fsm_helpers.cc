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
  //   CheckStateMachineProfCmds();

  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
      ERROR("Actuator %s: %s", name_.c_str(),
            "Cannot call a Motion cmd from FAULTED, reset Actuator first");
      return false;
      break;

    case ACTUATOR_SMS_HALTED:
      ElmoReset();  // This will open the brake, then fallthrough
    case ACTUATOR_SMS_HOLDING:
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
    case ACTUATOR_SMS_CSP:
    case ACTUATOR_SMS_CSV:
    case ACTUATOR_SMS_CST:
      // All of these commands can be safely preempted with CS* cmds
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_UPDATE_POSITION:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      // Currently, this faults the cal command
      //   Can be configured to ignore the offending CSP command
      ERROR("Actuator %s: %s", name_.c_str(),
            "Cannot call a Motion cmd during Calibration");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_CAL;
      return false;
      break;

    default:
      ERROR("Actuator %s: %s: %d", name_.c_str(), "Bad Actuator State ", actuator_sms_);
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
    case ACTUATOR_SMS_CSP:
    case ACTUATOR_SMS_CSV:
    case ACTUATOR_SMS_CST:
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_UPDATE_POSITION:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      ERROR("Actuator %s: %s", name_.c_str(),
            "Cannot execute gain scheduling commands, calibration is in "
            "progress");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_CAL;
      return false;

    default:
      ERROR("Actuator %s: %s: %d", name_.c_str(), "Bad Actuator State ", actuator_sms_);
      return false;
  }
  return true;
}

bool fastcat::Actuator::HandleNewCSPCmd(const DeviceCmd& cmd)
{
  // Validate the command arguments
  if (PosExceedsCmdLimits(cmd.actuator_csp_cmd.target_position +
                          cmd.actuator_csp_cmd.position_offset) ||
      VelExceedsCmdLimits(cmd.actuator_csp_cmd.velocity_offset) ||
      AccExceedsCmdLimits(cmd.actuator_csp_cmd.acceleration_offset)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing CSP Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing CSP Command");
    return false;
  }

  // detect latency between time when CSP message was generated
  // and when it is processed here
  double dt = fmax((state_->time - cmd.actuator_csp_cmd.request_time), 0.0);
  
  // reject command if request_time > 10 * loop_period, which indicates request
  // is stale, clocks are out of sync, or request_time was not correctly
  // populated by calling module
  if (dt > (10.0 * loop_period_)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(),
          "Failing CSP Command due to stale request_time; "
          "the request may be stale or the clocks may not be synchronized");
    ERROR("Request time: %f, actual time: %f, dt: %f", 
          cmd.actuator_csp_cmd.request_time, state_->time, dt);
    return false;
  }
   
  // if we were not previously in ACTUATOR_SMS_CSP state, then this request
  // is the first in a batch of CSP messages, clear the count of received
  // messages
  if(actuator_sms_ != ACTUATOR_SMS_CSP) {
    last_device_cmd_.clear();
  }

  // Cache the incoming command to `last_device_cmd_` and
  // defer its execution to the Process() function, which is always called after
  // new commands are handled for JSD devices
  last_device_cmd_.store(cmd);
  TransitionToState(ACTUATOR_SMS_CSP);
  return true;
}

bool fastcat::Actuator::HandleNewCSVCmd(const DeviceCmd& cmd)
{
  // Validate command arguments
  if (VelExceedsCmdLimits(cmd.actuator_csv_cmd.target_velocity +
                          cmd.actuator_csv_cmd.velocity_offset)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing CSV Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing CSV Command");
    return false;
  }

  jsd_elmo_motion_command_csv_t jsd_cmd;
  jsd_cmd.target_velocity    = EuToCnts(cmd.actuator_csv_cmd.target_velocity);
  jsd_cmd.velocity_offset    = EuToCnts(cmd.actuator_csv_cmd.velocity_offset);
  jsd_cmd.torque_offset_amps = cmd.actuator_csv_cmd.torque_offset_amps;

  ElmoCSV(jsd_cmd);

  TransitionToState(ACTUATOR_SMS_CSV);

  return true;
}

bool fastcat::Actuator::HandleNewCSTCmd(const DeviceCmd& cmd)
{
  // Validate command arguments
  if (CurrentExceedsCmdLimits(cmd.actuator_cst_cmd.target_torque_amps +
                              cmd.actuator_cst_cmd.torque_offset_amps)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing CST Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing CST Command");
    return false;
  }

  jsd_elmo_motion_command_cst_t jsd_cmd;
  jsd_cmd.target_torque_amps = cmd.actuator_cst_cmd.target_torque_amps;
  jsd_cmd.torque_offset_amps = cmd.actuator_cst_cmd.torque_offset_amps;

  ElmoCST(jsd_cmd);

  TransitionToState(ACTUATOR_SMS_CST);

  return true;
}

bool fastcat::Actuator::HandleNewProfPosCmd(const DeviceCmd& cmd)
{
  double target_position = ComputeTargetPosProfPosCmd(cmd);

  // Validate command arguments
  if (PosExceedsCmdLimits(target_position) ||
      VelExceedsCmdLimits(cmd.actuator_prof_pos_cmd.profile_velocity) ||
      VelExceedsCmdLimits(cmd.actuator_prof_pos_cmd.end_velocity) ||
      AccExceedsCmdLimits(cmd.actuator_prof_pos_cmd.profile_accel)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing Prof Pos Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing Prof Pos Command");
    return false;
  }

  return HandleNewProfPosCmdImpl(cmd);
}

bool fastcat::Actuator::HandleNewProfVelCmd(const DeviceCmd& cmd)
{
  if (VelExceedsCmdLimits(cmd.actuator_prof_vel_cmd.target_velocity) ||
      AccExceedsCmdLimits(cmd.actuator_prof_vel_cmd.profile_accel)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing Prof Vel Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing Prof Vel Command");
    return false;
  }

  return HandleNewProfVelCmdImpl(cmd);
}

bool fastcat::Actuator::HandleNewProfTorqueCmd(const DeviceCmd& cmd)
{
  if (CurrentExceedsCmdLimits(
          cmd.actuator_prof_torque_cmd.target_torque_amps)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing Prof Torque Command");
    return false;
  }

  // Check that the command can be honored within FSM state
  if (!CheckStateMachineMotionCmds()) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing Prof Torque Command");
    return false;
  }

  return HandleNewProfTorqueCmdImpl(cmd);
}

bool fastcat::Actuator::HandleNewHaltCmd()
{
  // clear device command queue
  last_device_cmd_.clear();

  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR("Actuator %s: %s", name_.c_str(),
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
    case ACTUATOR_SMS_CSP:
    case ACTUATOR_SMS_CSV:
    case ACTUATOR_SMS_CST:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_UPDATE_POSITION:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      break;

    default:
      ERROR("Actuator %s: %s: %d", name_.c_str(), "Bad Actuator State ", actuator_sms_);
      return false;
  }

  ElmoHalt();
  TransitionToState(ACTUATOR_SMS_HALTED);

  return true;
}

bool fastcat::Actuator::HandleNewSetOutputPositionCmd(const DeviceCmd& cmd)
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
    case ACTUATOR_SMS_CSP:
    case ACTUATOR_SMS_CSV:
    case ACTUATOR_SMS_CST:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_UPDATE_POSITION:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      ERROR("Actuator %s: %s", name_.c_str(),
            "Cannot set output position, motion command is active");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION;
      return false;

    default:
      ERROR("Actuator %s: %s: %d", name_.c_str(), "Bad Actuator State ", actuator_sms_);
      return false;
  }

  SetOutputPosition(cmd.actuator_set_output_position_cmd.position);

  // Do not change the state of the Actuator

  return true;
}

bool fastcat::Actuator::HandleNewSetUnitModeCmd(const DeviceCmd& cmd)
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
    case ACTUATOR_SMS_CSP:
    case ACTUATOR_SMS_CSV:
    case ACTUATOR_SMS_CST:
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_UPDATE_POSITION:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      ERROR("Actuator %s: %s", name_.c_str(),
            "Cannot set unit mode now, motion command is active");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION;
      return false;

    default:
      ERROR("Actuator %s: %s: %d", name_.c_str(), "Bad Actuator State ", actuator_sms_);
      return false;
  }

  ElmoSetUnitMode(cmd.actuator_sdo_set_unit_mode_cmd.mode,
                  cmd.actuator_sdo_set_unit_mode_cmd.app_id);

  return true;
}

bool fastcat::Actuator::HandleNewCalibrationCmd(const DeviceCmd& cmd)
{
  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR(
          "Actuator %s: %s", name_.c_str(),
          "Cannot start calibration from FAULTED state, reset actuator first");
      return false;

    case ACTUATOR_SMS_HALTED:
      ElmoReset();
    case ACTUATOR_SMS_HOLDING:
      break;
    case ACTUATOR_SMS_PROF_POS:
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
    case ACTUATOR_SMS_PROF_VEL:
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
    case ACTUATOR_SMS_PROF_TORQUE:
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
    case ACTUATOR_SMS_CSP:
    case ACTUATOR_SMS_CSV:
    case ACTUATOR_SMS_CST:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR("Actuator %s: %s", name_.c_str(),
            "Calibration requested during active motion command, faulting");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION;
      return false;
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
    case ACTUATOR_SMS_CAL_UPDATE_POSITION:
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      TransitionToState(ACTUATOR_SMS_FAULTED);
      ERROR("Actuator %s: %s", name_.c_str(),
            "Calibration requested but calibration already in progress, "
            "faulting");
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_CAL;
      return false;

    default:
      ERROR("Actuator %s: %s: %d", name_.c_str(), "Bad Actuator State ", actuator_sms_);
      return false;
  }

  if (VelExceedsCmdLimits(cmd.actuator_calibrate_cmd.velocity) ||
      AccExceedsCmdLimits(cmd.actuator_calibrate_cmd.accel)) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(), "Failing Calibration Command");
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
        "Actuator %s: Calibration cannot succeed when Range-of-motion (%lf) "
        "< Pos tracking error (%lf). "
        "Check Drive parameters for excessive pos tracking fault",
        name_.c_str(), rom, params_.pos_tracking_error_eu);
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_CAL_MOTION_RANGE;
    return false;
  }

  cal_cmd_ = cmd.actuator_calibrate_cmd;

  double target_position;
  if (cmd.actuator_calibrate_cmd.velocity > 0) {
    target_position = GetActualPosition(*state_) + cal_range;
  } else {
    target_position = GetActualPosition(*state_) - cal_range;
  }

  MSG("Setting Peak Current to calibration level: %lf", cal_cmd_.max_current);
  ElmoSetPeakCurrent(cal_cmd_.max_current);

  fastcat_trap_generate(&trap_, state_->time,
                        GetActualPosition(*state_),
                        target_position,
                        GetActualVelocity(),
                        0,
                        fabs(cmd.actuator_calibrate_cmd.velocity),
                        cmd.actuator_calibrate_cmd.accel);

  TransitionToState(ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP);

  return true;
}

bool fastcat::Actuator::IsIdleFaultConditionMet()
{
  if (IsStoEngaged()) {
    ERROR("%s: STO Engaged", name_.c_str());
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_STO_ENGAGED;
    return true;
  }

  if (IsJsdFaultCodePresent(*state_)) {
    ERROR("%s: jsd_fault_code indicates active fault", name_.c_str());
    return true;
  }
  return false;
}

bool fastcat::Actuator::IsMotionFaultConditionMet()
{
  if (IsStoEngaged()) {
    ERROR("%s: STO Engaged", name_.c_str());
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_STO_ENGAGED;
    return true;
  }

  if (IsJsdFaultCodePresent(*state_)) {
    ERROR("%s: jsd_fault_code indicates active fault", name_.c_str());
    return true;
  }
  auto elmo_state_machine_state = GetElmoStateMachineState();
  if (elmo_state_machine_state ==
          JSD_ELMO_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE ||
      elmo_state_machine_state ==
          JSD_ELMO_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE ||
      elmo_state_machine_state == JSD_ELMO_STATE_MACHINE_STATE_FAULT) {
    ERROR("%s: Elmo drive state machine state is off nominal", name_.c_str());
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_INVALID_ELMO_SMS_DURING_MOTION;
    return true;
  }
  return false;
}

fastcat::FaultType fastcat::Actuator::ProcessFaulted() { return NO_FAULT; }

fastcat::FaultType fastcat::Actuator::ProcessHalted()
{
  if (IsIdleFaultConditionMet()) {
    ERROR("Actuator %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessHolding()
{
  if (IsIdleFaultConditionMet()) {
    ERROR("Actuator %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  if ((state_->monotonic_time - last_transition_time_) >
      params_.holding_duration_sec) {
    ElmoHalt();
    TransitionToState(ACTUATOR_SMS_HALTED);
  }
  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessProfPosTrapImpl()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Actuator %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  jsd_elmo_motion_command_csp_t jsd_cmd;

  double pos_eu, vel;
  int    complete = fastcat_trap_update(&trap_, state_->time, &pos_eu, &vel);

  jsd_cmd.target_position    = PosEuToCnts(pos_eu);
  jsd_cmd.position_offset    = 0;
  jsd_cmd.velocity_offset    = EuToCnts(vel);
  jsd_cmd.torque_offset_amps = 0;

  if (!complete || params_.prof_pos_hold) {
    ElmoCSP(jsd_cmd);
  } else {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCS()
{
  if (IsMotionFaultConditionMet()) {
    ERROR("Actuator %s: %s", name_.c_str(), "Fault Condition present, faulting");
    return ALL_DEVICE_FAULT;
  }

  switch (actuator_sms_) {
    case ACTUATOR_SMS_CSP: {
      auto last_device_cmd = last_device_cmd_.load();
      switch(last_device_cmd.actuator_csp_cmd.interpolation_mode) { 
        case 0: { 
          // interpolation mode 0 is "implicit" 2nd order forward interpolation:
          // This method uses the last setpoint and an "implicit" next setpoint to
          // perform 2nd order interpolation. The implicit next setpoint is determined
          // using the provided feed-forward velocity and acceleration
          // * if a zero acceleration is provided, the method reduces to linear 
          //   extrapolation
          // * if a zero acceleration and velocity are provided, the method reduces
          //   to a zero-order hold
          double dt = fmax(
              (state_->time - last_device_cmd.actuator_csp_cmd.request_time), 0.0);
          double offset_target_position =
              last_device_cmd.actuator_csp_cmd.target_position +
              last_device_cmd.actuator_csp_cmd.velocity_offset * dt +
              0.5 * last_device_cmd.actuator_csp_cmd.acceleration_offset * dt * dt;
          double offset_target_velocity = 
              last_device_cmd.actuator_csp_cmd.velocity_offset +
              last_device_cmd.actuator_csp_cmd.acceleration_offset * dt;
          jsd_elmo_motion_command_csp_t jsd_cmd;
          jsd_cmd.target_position = PosEuToCnts(offset_target_position);
          jsd_cmd.position_offset =
              EuToCnts(last_device_cmd.actuator_csp_cmd.position_offset);
          jsd_cmd.velocity_offset = EuToCnts(offset_target_velocity);
          jsd_cmd.torque_offset_amps =
              last_device_cmd.actuator_csp_cmd.torque_offset_amps;
          ElmoCSP(jsd_cmd);
        } break;
        case 1: { 
          // "explicit" 3rd order backwards interpolation:
          // This method delays execution of the motion profile until a number of
          // CSP setpoints equal to the member variable `csp_cycles_delay_` have
          // accumulated in the `last_device_cmd_` buffer. Once sufficient
          // commands have been received, the time offset is recorded. The
          // time offset is used to look up two knot-points in the CSP command history.
          // Cubic interpolation is used to obtain the position and velocity for the
          // current timestamp. The method is explicit in the sense that it intepolates
          // between two CSP setpoints that have been explicitly received from the
          // calling module
          size_t num_received = last_device_cmd_.get_num_received();
          if(num_received == csp_cycles_delay_) {
            auto first_device_cmd = last_device_cmd_.load(csp_cycles_delay_ - 1);
            csp_interpolation_offset_time_ = 
              state_->time - first_device_cmd.actuator_csp_cmd.request_time;
          }
          if(num_received >= csp_cycles_delay_) {
            double sample_time = 
              state_->time - csp_interpolation_offset_time_;
            size_t index = 0;
            while(index < 10) {
               auto device_cmd = last_device_cmd_.load(++index);
               if(device_cmd.actuator_csp_cmd.request_time <= sample_time) {
                 break;
               }
            }

            if(index >= 10) {
              ERROR(
                "Error in logic for finding knots for CSP interpolation: index >=10");
              return ALL_DEVICE_FAULT;
            }
            
            auto knot_0 = last_device_cmd_.load(index);
            auto knot_1 = last_device_cmd_.load(index - 1);
            
            double t0 = knot_0.actuator_csp_cmd.request_time;
            double t1 = knot_1.actuator_csp_cmd.request_time;
            double p0 = knot_0.actuator_csp_cmd.target_position;
            double p1 = knot_1.actuator_csp_cmd.target_position;
            double v0 = knot_0.actuator_csp_cmd.velocity_offset;
            double v1 = knot_1.actuator_csp_cmd.velocity_offset;
            
            double dt = t1 - t0;
            
            double x = (sample_time - t0) / dt;
            if(x < 0 || x > 1.0) {
              ERROR("Error in logic for finding knots for CSP interpolation x=%f", x);
              ERROR("knot 0 index: %ld, knot 1 index: %ld", index, index - 1);
              ERROR("knot 0 time: %f, knot 1 time: %f", t0, t1);
              ERROR("knot 0 position: %f, knot 1 position: %f", p0, p1);
              ERROR("knot 0 velocity: %f, knot 1 velocity: %f", v0, v1);
              ERROR("sample_time: %f, csp_interp_offset_time: %f, time: %f", 
                    sample_time, csp_interpolation_offset_time_, state_->time);
              return ALL_DEVICE_FAULT;
            }

            v0 *= dt;
            v1 *= dt;

            double b0 = p0;
            double b1 = v0;
            double b2 = 3.0 * p1 - 3.0 * p0 - 2.0 * v0 - v1;
            double b3 = -2.0 * p1 + 2.0 * p0 + v0 + v1;

            double x2 = x * x;
            double x3 = x2 * x;
            double p = b0 + (b1 * x) + (b2 * x2) + (b3 * x3);
            double v = (b1 + 2.0 * b2 * x + 3.0 * b3 * x2) / dt;

            jsd_elmo_motion_command_csp_t jsd_cmd;
            jsd_cmd.target_position = PosEuToCnts(p);
            jsd_cmd.position_offset =
              EuToCnts(knot_1.actuator_csp_cmd.position_offset);
            jsd_cmd.velocity_offset = EuToCnts(v);
            jsd_cmd.torque_offset_amps = 
              knot_1.actuator_csp_cmd.torque_offset_amps;
            ElmoCSP(jsd_cmd);
          }
        } break;
        default: {
          ERROR("Invalid CSP interpolation mode specified: (%d)",
                last_device_cmd.actuator_csp_cmd.interpolation_mode
          );
          return ALL_DEVICE_FAULT;
        }
      } 
    } break;
    case ACTUATOR_SMS_CSV:
    case ACTUATOR_SMS_CST:
      break;

    default:
      // this criteria should never be met, we should only be in one of
      // {CSP,CSV,CST} modes when ProcessCS() function is called
      ERROR("Invalid device state found for ProcessCS() function");
      return ALL_DEVICE_FAULT;
  }

  if ((state_->monotonic_time - last_transition_time_) > (7.0 * loop_period_)) {
    TransitionToState(ACTUATOR_SMS_HOLDING);
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCalMoveToHardstop()
{
  // add other reasons as needed...
  if (IsStoEngaged()) {
    ERROR("Actuator %s: %s", name_.c_str(), "Fault Condition present, faulting");
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_STO_ENGAGED;

    MSG("Restoring Current after calibration: %lf",
        params_.peak_current_limit_amps);
    ElmoSetPeakCurrent(params_.peak_current_limit_amps);

    return ALL_DEVICE_FAULT;
  }

  // assume pos/vel tracking fault
  if (IsJsdFaultCodePresent(*state_)) {
    ElmoHalt();
    MSG("Actuator %s: %s: %s", name_.c_str(),
        "Detected Hardstop, Elmo jsd_fault_code",
        GetJSDFaultCodeAsString(*state_).c_str());

    MSG("Restoring Current after calibration: %lf",
        params_.peak_current_limit_amps);
    ElmoSetPeakCurrent(params_.peak_current_limit_amps);

    // update position before transitioning to CAL_AT_HARDSTOP state
    TransitionToState(ACTUATOR_SMS_CAL_UPDATE_POSITION);
  }

  jsd_elmo_motion_command_csp_t jsd_cmd;

  double pos_eu, vel;
  int    complete = fastcat_trap_update(&trap_, state_->time, &pos_eu, &vel);
  jsd_cmd.target_position    = PosEuToCnts(pos_eu);
  jsd_cmd.position_offset    = 0;
  jsd_cmd.velocity_offset    = EuToCnts(vel);
  jsd_cmd.torque_offset_amps = 0;
  ElmoCSP(jsd_cmd);

  if (complete) {
    TransitionToState(ACTUATOR_SMS_FAULTED);
    ERROR("Actuator %s: %s", name_.c_str(),
          "Moved Full Range and did not encounter hard stop");
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_NO_HARDSTOP_DURING_CAL;

    MSG("Restoring Current after calibration: %lf",
        params_.peak_current_limit_amps);
    ElmoSetPeakCurrent(params_.peak_current_limit_amps);

    return ALL_DEVICE_FAULT;
  }

  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCalUpdatePosition()
{
  // spin in this state for one second to allow for any motor dynamics to settle out
  // from previous tracking error fault
  if((state_->monotonic_time - last_transition_time_) < 1.0) {
    return NO_FAULT;
  } 

  // Update calibration position before sending Reset() to actuators, as sending actuator
  // Reset() can cause some minor movement in actuators and/or a change in preload
  // against the hardstop position
  double cal_position     = 0;
  if (cal_cmd_.velocity > 0) {
    cal_position     = params_.high_pos_cal_limit_eu;
  } else {
    cal_position     = params_.low_pos_cal_limit_eu;
  }
  SetOutputPosition(cal_position);
  TransitionToState(ACTUATOR_SMS_CAL_AT_HARDSTOP);
  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCalAtHardstop() 
{
  // no need to check faults in this state
  // And clear the Elmo fault that is generated from contacting the hardstop
  // Loop here until the drive is no longer faulted
  if (GetElmoStateMachineState() !=
          JSD_ELMO_STATE_MACHINE_STATE_OPERATION_ENABLED &&
      IsJsdFaultCodePresent(*state_)) {

    // We have waited too long, fault
    if ((state_->monotonic_time - last_transition_time_) > 5.0) {
      ERROR("Actuator %s: %s: %lf", name_.c_str(),
            "Waited too long for drive to reset in CAL_AT_HARDSTOP state",
            (state_->monotonic_time - last_transition_time_));
      fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_CAL_RESET_TIMEOUT_EXCEEDED;
      return ALL_DEVICE_FAULT;
    }
    
    ElmoReset();
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

  // generate trap to move back to soft-stop position
  fastcat_trap_generate(&trap_, state_->time, cal_position, backoff_position, 0, 0,
                        fabs(cal_cmd_.velocity), cal_cmd_.accel);
  TransitionToState(ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP);
  return NO_FAULT;
}

fastcat::FaultType fastcat::Actuator::ProcessCalMoveToSoftstop()
{
  return ProcessProfPosTrapImpl();
}
