// Include related header (for cc files)
#include "fastcat/jsd/actuator.h"

// Include c then c++ libraries
#include <string.h>
#include <sys/stat.h>

#include <cmath>
#include <fstream>
#include <iostream>

// Include external then project includes
#include "fastcat/yaml_parser.h"
#include "jsd/jsd.h"

fastcat::Actuator::Actuator()
{
  MSG_DEBUG("Constructed Actuator");

  state_                = std::make_shared<DeviceState>();
  state_->type          = ACTUATOR_STATE;
  actuator_sms_         = ACTUATOR_SMS_HALTED;
  last_transition_time_ = jsd_time_get_time_sec();
  last_egd_reset_time_  = jsd_time_get_time_sec();
}

bool fastcat::Actuator::ConfigFromYaml(YAML::Node node)
{
  if (!ParseVal(node, "name", name_)) {
    return false;
  }

  if (!ParseVal(node, "actuator_type", actuator_type_str_)) {
    return false;
  }

  if (0 == actuator_type_str_.compare("revolute")) {
    actuator_type_ = ACTUATOR_TYPE_REVOLUTE;

  } else if (0 == actuator_type_str_.compare("prismatic")) {
    actuator_type_ = ACTUATOR_TYPE_PRISMATIC;

  } else {
    ERROR("Failed to parse actuator_type string: %s must be %s or %s",
          actuator_type_str_.c_str(), "revolute", "prismatic");
    return false;
  }

  if (!ParseValCheckRange(node, "gear_ratio", gear_ratio_, 0, 1.0e12)) {
    return false;
  }

  if (!ParseValCheckRange(node, "counts_per_rev", counts_per_rev_, 0, 1.0e12)) {
    return false;
  }

  if (!ParseValCheckRange(node, "max_speed_eu_per_sec", max_speed_eu_per_sec_,
                          0, (double)INT32_MAX + 1.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "max_accel_eu_per_sec2", max_accel_eu_per_sec2_,
                          0, (double)INT32_MAX + 1.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "over_speed_multiplier", over_speed_multiplier_,
                          0.0, 1000.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "vel_tracking_error_eu_per_sec",
                          vel_tracking_error_eu_per_sec_, 0,
                          (double)INT32_MAX + 1.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "pos_tracking_error_eu", pos_tracking_error_eu_,
                          0, (double)INT32_MAX + 1.0)) {
    return false;
  }

  if (!ParseValCheckRange(node, "peak_current_limit_amps",
                          peak_current_limit_amps_, 0, 100)) {
    return false;
  }
  if (!ParseValCheckRange(node, "peak_current_time_sec", peak_current_time_sec_,
                          0, 60.0)) {
    return false;
  }
  if (!ParseValCheckRange(node, "continuous_current_limit_amps",
                          continuous_current_limit_amps_, 0, 100)) {
    return false;
  }
  if (!ParseValCheckRange(node, "torque_slope_amps_per_sec",
                          torque_slope_amps_per_sec_, 0, 1000.0)) {
    return false;
  }

  if (!ParseVal(node, "low_pos_cal_limit_eu", low_pos_cal_limit_eu_)) {
    return false;
  }
  if (!ParseVal(node, "low_pos_cmd_limit_eu", low_pos_cmd_limit_eu_)) {
    return false;
  }

  if (!ParseVal(node, "high_pos_cal_limit_eu", high_pos_cal_limit_eu_)) {
    return false;
  }
  if (!ParseVal(node, "high_pos_cmd_limit_eu", high_pos_cmd_limit_eu_)) {
    return false;
  }

  if (!ParseValCheckRange(node, "holding_duration_sec", holding_duration_sec_,
                          0, 1000.0)) {
    return false;
  }

  if (!ParseVal(node, "egd_brake_engage_msec", egd_brake_engage_msec_)) {
    return false;
  }
  if (!ParseVal(node, "egd_brake_disengage_msec", egd_brake_disengage_msec_)) {
    return false;
  }
  if (!ParseVal(node, "egd_crc", egd_crc_)) {
    return false;
  }
  if (!ParseVal(node, "egd_drive_max_current_limit",
                egd_drive_max_cur_limit_amps_)) {
    return false;
  }
  if (!ParseValCheckRange(node, "smooth_factor", smooth_factor_, -1, 64)) {
    return false;
  }
  
  if (ParseOptValCheckRange(node, "torque_constant", torque_constant_, 0.0, 999999.0) &&
      ParseOptValCheckRange(node, "winding_resistance", winding_resistance_, 0.0, 999999.0) ) {
    
    // Only compute power if we received both torque_constant and winding_resistance parameters
    compute_power_ = true;
    
    // Read in brake power (if provided) to add to actuator power
    if (!ParseOptValCheckRange(node, "brake_power", brake_power_, 0.0, 9999.0)) {
      // If not found then set to zero
      brake_power_ = 0.0;
    }
    
    // Read in any gear ratio between the motor and encoder for power calculation
    if (!ParseOptValCheckRange(node, "motor_encoder_gear_ratio", motor_encoder_gear_ratio_, 0.0, 9999.0)) {
      // If not found then set to 1.0
      motor_encoder_gear_ratio_ = 1.0;
    }
  }

  std::string ctrl_gs_mode_string;
  if (ParseOptVal(node, "ctrl_gain_scheduling_mode", ctrl_gs_mode_string)) {
    if (!GSModeFromString(ctrl_gs_mode_string,
                          jsd_slave_config_.egd.ctrl_gain_scheduling_mode)) {
      return false;
    }
  } else {
    // Use mode saved in driver's non-volatile memory.
    jsd_slave_config_.egd.ctrl_gain_scheduling_mode =
        JSD_EGD_GAIN_SCHEDULING_MODE_PRELOADED;
  }

  if (!ParseOptVal(node, "absolute_encoder", actuator_absolute_encoder_)) {
    // If we do not find this parameter then set it to false
    actuator_absolute_encoder_ = false;
  }

  // Whether position should be actively controlled after a profile position
  // command is concluded.
  if (!ParseOptVal(node, "prof_pos_hold", prof_pos_hold_)) {
    prof_pos_hold_ = false;
  }

  // overall_reduction must be set before using EuToCnts/CntsToEu
  if (actuator_type_ == ACTUATOR_TYPE_REVOLUTE) {
    overall_reduction_ = counts_per_rev_ * gear_ratio_ / (2.0 * M_PI);

  } else if (actuator_type_ == ACTUATOR_TYPE_PRISMATIC) {
    overall_reduction_ = counts_per_rev_ * gear_ratio_;

  } else {
    ERROR("Bad actuator_type: %d", actuator_type_);
    return false;
  }
  MSG("Overall Reduction: %lf", overall_reduction_);

  state_->name = name_;

  jsd_slave_config_.configuration_active = true;
  jsd_slave_config_.product_code         = JSD_EGD_PRODUCT_CODE;
  snprintf(jsd_slave_config_.name, JSD_NAME_LEN, "%s", name_.c_str());

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
  jsd_slave_config_.egd.low_position_limit      = 0;  // disable
  jsd_slave_config_.egd.high_position_limit     = 0;  // disable
  jsd_slave_config_.egd.brake_engage_msec       = egd_brake_engage_msec_;
  jsd_slave_config_.egd.brake_disengage_msec    = egd_brake_disengage_msec_;
  jsd_slave_config_.egd.crc                     = egd_crc_;
  jsd_slave_config_.egd.drive_max_current_limit = egd_drive_max_cur_limit_amps_;
  jsd_slave_config_.egd.smooth_factor           = smooth_factor_;

  EgdSetConfig();

  return true;
}

bool fastcat::Actuator::Read()
{
  EgdRead();

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

  if (compute_power_) {
    double motor_velocity =
      fabs(state_->actuator_state.actual_velocity) *
      gear_ratio_ *
      motor_encoder_gear_ratio_;
    
    double current = fabs(jsd_egd_state_.actual_current);

    // P = R I^2 + K_T * I * \omega
    state_->actuator_state.power = current *
      (winding_resistance_ * current +
       torque_constant_ * motor_velocity);

    // Should check, but assuming motor_on > 0 means brakes powered/disengaged
    if (state_->actuator_state.motor_on)
      state_->actuator_state.power += brake_power_;
  }
  // else
  // state_->actuator_state.power = 0;

  return true;
}

bool fastcat::Actuator::Write(DeviceCmd& cmd)
{

  // If device supports async SDO requests
  AsyncSdoRetVal sdoResult = WriteAsyncSdoRequest(cmd);
  if(sdoResult != SDO_RET_VAL_NOT_APPLICABLE){
    return (sdoResult == SDO_RET_VAL_SUCCESS);
  }

  // Honor these Non-motion Commands even when faulted
  switch (cmd.type) {
    case ACTUATOR_RESET_CMD:
      Reset();
      return true;
      break;

    case ACTUATOR_HALT_CMD:
      if (!HandleNewHaltCmd()) {
        ERROR("Failed to handle Halt Command");
        return false;
      }
      break;

    case ACTUATOR_SET_OUTPUT_POSITION_CMD:
      if (!HandleNewSetOutputPositionCmd(cmd)) {
        ERROR("Failed to handle Set Output Position Command");
        return false;
      }
      break;

    case ACTUATOR_SET_MAX_CURRENT_CMD:
      // This application may choose to set this during motions
      // in order to boost current during accelration/decel
      // phases so don't check the state machine
      peak_current_limit_amps_ = cmd.actuator_set_max_current_cmd.current;
      EgdSetPeakCurrent(peak_current_limit_amps_);
      return true;
      break;

    case ACTUATOR_SDO_SET_UNIT_MODE_CMD:
      if (!HandleNewSetUnitModeCmd(cmd)) {
        ERROR("Failed to handle Set Unit Mode Command");
        return false;
      }
      break;

    case ACTUATOR_SDO_DISABLE_GAIN_SCHEDULING_CMD: {
      if (!CheckStateMachineGainSchedulingCmds()) {
        ERROR("Failed to handle SDO Disable Gain Scheduling Command");
        return false;
      }
      EgdSetGainSchedulingMode(
          JSD_EGD_GAIN_SCHEDULING_MODE_DISABLED,
          cmd.actuator_sdo_disable_gain_scheduling_cmd.app_id);
      return true;
      break;
    }

    case ACTUATOR_SDO_ENABLE_SPEED_GAIN_SCHEDULING_CMD: {
      if (!CheckStateMachineGainSchedulingCmds()) {
        ERROR("Failed to handle SDO Enable Speed Gain Scheduling Command");
        return false;
      }
      EgdSetGainSchedulingMode(
          JSD_EGD_GAIN_SCHEDULING_MODE_SPEED,
          cmd.actuator_sdo_enable_speed_gain_scheduling_cmd.app_id);
      return true;
      break;
    }

    case ACTUATOR_SDO_ENABLE_POSITION_GAIN_SCHEDULING_CMD: {
      if (!CheckStateMachineGainSchedulingCmds()) {
        ERROR("Failed to handle SDO Enable Position Gain Scheduling Command");
        return false;
      }
      EgdSetGainSchedulingMode(
          JSD_EGD_GAIN_SCHEDULING_MODE_POSITION,
          cmd.actuator_sdo_enable_position_gain_scheduling_cmd.app_id);
      return true;
      break;
    }

    case ACTUATOR_SDO_ENABLE_MANUAL_GAIN_SCHEDULING_CMD: {
      if (!CheckStateMachineGainSchedulingCmds()) {
        ERROR("Failed to handle SDO Enable Manual Gain Scheduling Command");
        return false;
      }
      EgdSetGainSchedulingMode(
          JSD_EGD_GAIN_SCHEDULING_MODE_MANUAL_LOW, 
          cmd.actuator_sdo_enable_manual_gain_scheduling_cmd.app_id);
      return true;
      break;
    }

    case ACTUATOR_SET_GAIN_SCHEDULING_INDEX_CMD: {
      if (!CheckStateMachineGainSchedulingCmds()) {
        ERROR("Failed to handle Set Gain Scheduling Index Command");
        return false;
      }
      EgdSetGainSchedulingIndex( 
          cmd.actuator_set_gain_scheduling_index_cmd.gain_scheduling_index);
      return true;
      break;
    }
  }
  

  // Return early if a fault is active
  // So as to not honor these motion commands when faulted 
  if(device_fault_active_){
    return false;
  }
  
  switch (cmd.type) {
    case ACTUATOR_CSP_CMD:
      if (!HandleNewCSPCmd(cmd)) {
        ERROR("Failed to handle Actuator CSP command");
        return false;
      }
      break;

    case ACTUATOR_CSV_CMD:
      if (!HandleNewCSVCmd(cmd)) {
        ERROR("Failed to handle Actuator CSV command");
        return false;
      }
      break;

    case ACTUATOR_CST_CMD:
      if (!HandleNewCSTCmd(cmd)) {
        ERROR("Failed to handle Actuator CST command");
        return false;
      }
      break;

    case ACTUATOR_PROF_POS_CMD:
      if (!HandleNewProfPosCmd(cmd)) {
        ERROR("Failed to setup Actuator Profiled Pos command");
        return false;
      }
      break;

    case ACTUATOR_PROF_VEL_CMD:
      if (!HandleNewProfVelCmd(cmd)) {
        ERROR("Failed to setup Actuator Profiled Pos command");
        return false;
      }
      break;

    case ACTUATOR_PROF_TORQUE_CMD:
      if (!HandleNewProfTorqueCmd(cmd)) {
        ERROR("Failed to setup Actuator Profiled Pos command");
        return false;
      }
      break;

    case ACTUATOR_CALIBRATE_CMD:
      if (!HandleNewCalibrationCmd(cmd)) {
        ERROR("Failed to handle Calibrate Command");
        return false;
      }
      break;

    default:
      WARNING("That command type is not supported in this mode!");
      return false;
  }

  return true;
}

fastcat::FaultType fastcat::Actuator::Process()
{
  fastcat::FaultType retval = NO_FAULT;

  switch (actuator_sms_) {
    case ACTUATOR_SMS_FAULTED:
      break;

    case ACTUATOR_SMS_HALTED:
      retval = ProcessHalted();
      break;

    case ACTUATOR_SMS_HOLDING:
      retval = ProcessHolding();
      break;

    case ACTUATOR_SMS_PROF_POS:
      retval = ProcessProfPos();
      break;
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
      retval = ProcessProfPosDisengaging();
      break;

    case ACTUATOR_SMS_PROF_VEL:
      retval = ProcessProfVel();
      break;
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
      retval = ProcessProfVelDisengaging();
      break;

    case ACTUATOR_SMS_PROF_TORQUE:
      retval = ProcessProfTorque();
      break;
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
      retval = ProcessProfTorqueDisengaging();
      break;

    case ACTUATOR_SMS_CS:
      retval = ProcessCS();
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
      retval = ProcessCalMoveToHardstop();
      break;

    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
      retval = ProcessCalAtHardstop();
      break;

    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      retval = ProcessCalMoveToSoftstop();
      break;

    default:
      ERROR("Bad Actuator State Machine State: %d", actuator_sms_);
      retval = ALL_DEVICE_FAULT;
      break;
  }

  EgdProcess();
  return retval;
}

void fastcat::Actuator::Fault()
{
  WARNING("Faulting Actuator %s", name_.c_str());

  TransitionToState(ACTUATOR_SMS_FAULTED);
  EgdHalt();
}

void fastcat::Actuator::Reset()
{
  WARNING("Resetting Actuator device %s", name_.c_str());
  if (actuator_sms_ == ACTUATOR_SMS_FAULTED) {
    // Resetting here would open brakes so we explicitly do not reset the EGD
    // and instead only clear latched errors 
    EgdClearErrors();
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_OKAY;
    TransitionToState(ACTUATOR_SMS_HALTED);
  }
}

bool fastcat::Actuator::SetOutputPosition(double position)
{
  MSG("Act %s: %s%lf %s%lf", name_.c_str(),
      "Changing Position from: ", state_->actuator_state.actual_position,
      "to : ", position);

  egd_pos_offset_cnts_ =
      jsd_egd_state_.actual_position - (int32_t)(position * overall_reduction_);
  return true;
}

bool fastcat::Actuator::HasAbsoluteEncoder(){
  return actuator_absolute_encoder_;
}

double fastcat::Actuator::CntsToEu(int32_t cnts)
{
  return cnts / overall_reduction_;
}
int32_t fastcat::Actuator::EuToCnts(double eu)
{
  return eu * overall_reduction_;
}

double fastcat::Actuator::PosCntsToEu(int32_t cnts)
{
  return CntsToEu(cnts - egd_pos_offset_cnts_);
}
int32_t fastcat::Actuator::PosEuToCnts(double eu)
{
  return EuToCnts(eu) + egd_pos_offset_cnts_;
}

bool fastcat::Actuator::PosExceedsCmdLimits(double pos_eu)
{
  if (pos_eu > high_pos_cmd_limit_eu_) {
    ERROR("Commanded Position (%lf) exceeds high cmd limit (%lf)", pos_eu,
          high_pos_cmd_limit_eu_);
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_CMD_LIMIT_EXCEEDED;
    return true;
  }
  if (pos_eu < low_pos_cmd_limit_eu_) {
    ERROR("Commanded Position (%lf) exceeds low cmd limit: (%lf)", pos_eu,
          low_pos_cmd_limit_eu_);
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_CMD_LIMIT_EXCEEDED;
    return true;
  }
  return false;
}

bool fastcat::Actuator::VelExceedsCmdLimits(double vel_eu)
{
  if (fabs(vel_eu) > max_speed_eu_per_sec_) {
    ERROR("Commanded Velocity (%lf) exceeds max speed limit (%lf)", vel_eu,
          max_speed_eu_per_sec_);
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_CMD_LIMIT_EXCEEDED;
    return true;
  }
  return false;
}

bool fastcat::Actuator::AccExceedsCmdLimits(double acc_eu)
{
  if (fabs(acc_eu) > max_accel_eu_per_sec2_) {
    ERROR("Commanded Acceleration (%lf) exceeds max limit (%lf)", acc_eu,
          max_accel_eu_per_sec2_);
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_CMD_LIMIT_EXCEEDED;
    return true;
  }
  return false;
}

bool fastcat::Actuator::CurrentExceedsCmdLimits(double current)
{
  if (fabs(current) > continuous_current_limit_amps_) {
    WARNING(
        "Commanded current (%lf) exceeds max continuous current limit (%lf)",
        current, continuous_current_limit_amps_);
    // Not issuing fault for now
  }

  if (fabs(current) > peak_current_limit_amps_) {
    ERROR("Commanded current (%lf) exceeds max peak current limit (%lf)",
          current, peak_current_limit_amps_);
    fastcat_fault_ = ACTUATOR_FASTCAT_FAULT_CMD_LIMIT_EXCEEDED;
    return true;
  }

  return false;
}

void fastcat::Actuator::TransitionToState(ActuatorStateMachineState sms)
{
  if (actuator_sms_ == sms) {
    last_transition_time_ = state_->time;
    return;
  }

  MSG("Requested Actuator %s state transition from %s to %s", name_.c_str(),
      StateMachineStateToString(actuator_sms_).c_str(),
      StateMachineStateToString(sms).c_str());

  last_transition_time_ = state_->time;
  actuator_sms_         = sms;
}

std::string fastcat::Actuator::StateMachineStateToString(
    ActuatorStateMachineState sms)
{
  std::string str;
  switch (sms) {
    case ACTUATOR_SMS_FAULTED:
      str = std::string("FAULTED");
      break;
    case ACTUATOR_SMS_HALTED:
      str = std::string("HALTED");
      break;
    case ACTUATOR_SMS_HOLDING:
      str = std::string("HOLDING");
      break;
    case ACTUATOR_SMS_PROF_POS:
      str = std::string("PROF_POS");
      break;
    case ACTUATOR_SMS_PROF_POS_DISENGAGING:
      str = std::string("PROF_POS_DISENGAGING");
      break;
    case ACTUATOR_SMS_PROF_VEL:
      str = std::string("PROF_VEL");
      break;
    case ACTUATOR_SMS_PROF_VEL_DISENGAGING:
      str = std::string("PROF_VEL_DISENGAGING");
      break;
    case ACTUATOR_SMS_PROF_TORQUE:
      str = std::string("PROF_TORQUE");
      break;
    case ACTUATOR_SMS_PROF_TORQUE_DISENGAGING:
      str = std::string("PROF_TORQUE_DISENGAGING");
      break;
    case ACTUATOR_SMS_CS:
      str = std::string("CS");
      break;
    case ACTUATOR_SMS_CAL_MOVE_TO_HARDSTOP:
      str = std::string("CAL_MOVE_TO_HARDSTOP");
      break;
    case ACTUATOR_SMS_CAL_AT_HARDSTOP:
      str = std::string("CAL_AT_HARDSTOP");
      break;
    case ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP:
      str = std::string("CAL_MOVE_TO_SOFTSTOP");
      break;
    default:
      ERROR("Bad Actuator State Machine State: %d", static_cast<int>(sms));
  }
  return str;
}

void fastcat::Actuator::EgdRead()
{
  jsd_egd_read((jsd_t*)context_, slave_id_);
  memcpy(&jsd_egd_state_, jsd_egd_get_state((jsd_t*)context_, slave_id_),
         sizeof(jsd_egd_state_t));
}

void fastcat::Actuator::EgdSetConfig()
{
  MSG_DEBUG("Setting jsd slave config");
  jsd_set_slave_config((jsd_t*)context_, slave_id_, jsd_slave_config_);
}
void fastcat::Actuator::EgdProcess()
{
  jsd_egd_process((jsd_t*)context_, slave_id_);
}

void fastcat::Actuator::EgdClearErrors()
{
  jsd_egd_clear_errors((jsd_t*)context_, slave_id_);
}

void fastcat::Actuator::EgdReset()
{
  MSG("Resetting EGD through JSD: %s", name_.c_str());
  jsd_egd_reset((jsd_t*)context_, slave_id_);
}

void fastcat::Actuator::EgdHalt() { jsd_egd_halt((jsd_t*)context_, slave_id_); }

void fastcat::Actuator::EgdSetPeakCurrent(double current)
{
  jsd_egd_set_peak_current((jsd_t*)context_, slave_id_, current);
}

void fastcat::Actuator::EgdSetUnitMode(int32_t mode, uint16_t app_id)
{
  MSG("Commanding new UM[1] = %d app_id = %u", mode, app_id);
  jsd_egd_async_sdo_set_unit_mode((jsd_t*)context_, slave_id_, mode, app_id);
}

void fastcat::Actuator::EgdCSP(jsd_egd_motion_command_csp_t jsd_csp_cmd)
{
  jsd_egd_set_motion_command_csp((jsd_t*)context_, slave_id_, jsd_csp_cmd);
}

void fastcat::Actuator::EgdCSV(jsd_egd_motion_command_csv_t jsd_csv_cmd)
{
  jsd_egd_set_motion_command_csv((jsd_t*)context_, slave_id_, jsd_csv_cmd);
}

void fastcat::Actuator::EgdCST(jsd_egd_motion_command_cst_t jsd_cst_cmd)
{
  jsd_egd_set_motion_command_cst((jsd_t*)context_, slave_id_, jsd_cst_cmd);
}

void fastcat::Actuator::EgdSetGainSchedulingMode(
    jsd_egd_gain_scheduling_mode_t mode, 
    uint16_t app_id)
{
  jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
      (jsd_t*)context_, slave_id_, mode, app_id);
}

void fastcat::Actuator::EgdSetGainSchedulingIndex(uint16_t index)
{
  jsd_egd_set_gain_scheduling_index(
      (jsd_t*)context_, slave_id_, true, index);
}

bool fastcat::Actuator::GSModeFromString(
    std::string gs_mode_string, jsd_egd_gain_scheduling_mode_t& gs_mode)
{
  MSG("Converting gain scheduling mode to string.");
  if (gs_mode_string.compare("DISABLED") == 0) {
    gs_mode = JSD_EGD_GAIN_SCHEDULING_MODE_DISABLED;
  } else if (gs_mode_string.compare("SPEED") == 0) {
    gs_mode = JSD_EGD_GAIN_SCHEDULING_MODE_SPEED;
  } else if (gs_mode_string.compare("POSITION") == 0) {
    gs_mode = JSD_EGD_GAIN_SCHEDULING_MODE_POSITION;
  } else if (gs_mode_string.compare("MANUAL") == 0) {
    gs_mode = JSD_EGD_GAIN_SCHEDULING_MODE_MANUAL_LOW;
  } else {
    ERROR("Gain scheduling mode %s is invalid", gs_mode_string.c_str());
    return false;
  }

  return true;
}

std::string fastcat::Actuator::GetFastcatFaultCodeAsString(
    const DeviceState& state)
{
  std::string fault_str;

  if (state.type == ACTUATOR_STATE) {
    auto fault = static_cast<ActuatorFastcatFault>(
        state.actuator_state.fastcat_fault_code);

    switch (fault) {
      case ACTUATOR_FASTCAT_FAULT_OKAY:
        fault_str = "FASTCAT_FAULT_OKAY";
        break;
      case ACTUATOR_FASTCAT_FAULT_CMD_LIMIT_EXCEEDED:
        fault_str = "FASTCAT_FAULT_CMD_LIMIT_EXCEEDED";
        break;
      case ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_MOTION:
        fault_str = "FASTCAT_FAULT_INVALID_CMD_DURING_MOTION";
        break;
      case ACTUATOR_FASTCAT_FAULT_INVALID_CMD_DURING_CAL:
        fault_str = "FASTCAT_FAULT_INVALID_CMD_DURING_CAL";
        break;
      case ACTUATOR_FASTCAT_FAULT_INVALID_CAL_MOTION_RANGE:
        fault_str = "FASTCAT_FAULT_INVALID_CAL_MOTION_RANGE";
        break;
      case ACTUATOR_FASTCAT_FAULT_STO_ENGAGED:
        fault_str = "FASTCAT_FAULT_STO_ENGAGED";
        break;
      case ACTUATOR_FASTCAT_FAULT_INVALID_EGD_SMS_DURING_MOTION:
        fault_str = "FASTCAT_FAULT_INVALID_EGD_SMS_DURING_MOTION";
        break;
      case ACTUATOR_FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED:
        fault_str = "FASTCAT_FAULT_BRAKE_DISENGAGE_TIMEOUT_EXCEEDED";
        break;
      case ACTUATOR_FASTCAT_FAULT_NO_HARDSTOP_DURING_CAL:
        fault_str = "FASTCAT_FAULT_NO_HARDSTOP_DURING_CAL";
        break;
      case ACTUATOR_FASTCAT_FAULT_CAL_RESET_TIMEOUT_EXCEEDED:
        fault_str = "FASTCAT_FAULT_CAL_RESET_TIMEOUT_EXCEEDED";
        break;
      default:
        fault_str = "Bad Fastcat fault code: " +
                    std::to_string(state.actuator_state.fastcat_fault_code);
    }
  } else {
    fault_str = "State is not type ACTUATOR_STATE";
  }

  return fault_str;
}

std::string fastcat::Actuator::GetJSDFaultCodeAsString(const DeviceState& state)
{
  std::string fault_str;

  if (state.type == ACTUATOR_STATE) {
    auto fault =
        static_cast<jsd_egd_fault_code_t>(state.actuator_state.jsd_fault_code);
    fault_str = std::string(jsd_egd_fault_code_to_string(fault));
  } else {
    fault_str = "State is not type ACTUATOR_STATE";
  }

  return fault_str;
}
